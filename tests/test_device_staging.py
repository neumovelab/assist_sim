"""Staging a device XML must not write inside the installed package.

``prepare_device_xml`` writes a massaged copy of the device XML that ``combine``
attaches. That copy used to go next to the original, which is
``site-packages/assist_sim/models/<Device>/`` for a pip install. That directory is
read-only or root-owned in containers and on shared HPC nodes, and staging runs on
the critical path of *every* combine, so the whole pipeline failed there. It also
left stray XML files in the package when a process died before the cleanup ran.

The copy now goes to the system temp dir, which only works while every asset path
in it stays absolute. These tests pin both halves of that contract.
"""

from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco as mj
import pytest

from assist_sim import preprocess
from assist_sim.preprocess import prepare_device_xml

from .conftest import needs_myo_sim

DEVICE_XML = "DephyExoBoot/L1model.xml"


def _write_device(tmp_path: Path, compiler_attrs: str = 'angle="radian"', body: str = "") -> Path:
    """A minimal but valid device XML, so the staging path can be tested alone."""
    src = tmp_path / "L9model.xml"
    src.write_text(
        f"<mujoco model='dev'><compiler {compiler_attrs}/>{body}"
        "<worldbody><body name='dev_base'><geom type='box' size='.01 .01 .01'/></body></worldbody>"
        "</mujoco>",
        encoding="utf-8",
    )
    return src


def test_staged_copy_is_not_written_into_the_package(models_dir: Path) -> None:
    src = models_dir / DEVICE_XML
    staged = Path(prepare_device_xml(str(src)))
    try:
        assert staged.parent != src.parent
        assert Path(tempfile.gettempdir()).resolve() in staged.resolve().parents
    finally:
        staged.unlink(missing_ok=True)


def test_staged_copy_still_resolves_its_meshes(models_dir: Path) -> None:
    """The point of the absolute-path rewrite: meshes load from the temp location."""
    src = models_dir / DEVICE_XML
    staged = Path(prepare_device_xml(str(src)))
    try:
        model = mj.MjSpec.from_file(str(staged)).compile()
        assert model.nmesh > 0
    finally:
        staged.unlink(missing_ok=True)


def test_meshdir_is_absolute_even_when_the_device_authors_none(tmp_path: Path) -> None:
    src = _write_device(tmp_path)
    staged = Path(prepare_device_xml(str(src)))
    try:
        compiler = ET.parse(str(staged)).getroot().find("compiler")
        assert compiler is not None
        assert Path(compiler.get("meshdir", "")) == tmp_path
        assert Path(compiler.get("texturedir", "")) == tmp_path
    finally:
        staged.unlink(missing_ok=True)


def test_authored_relative_meshdir_is_resolved_against_the_source(tmp_path: Path) -> None:
    (tmp_path / "mesh").mkdir()
    src = _write_device(tmp_path, compiler_attrs='angle="radian" meshdir="mesh"')
    staged = Path(prepare_device_xml(str(src)))
    try:
        compiler = ET.parse(str(staged)).getroot().find("compiler")
        assert compiler is not None
        assert Path(compiler.get("meshdir", "")) == tmp_path / "mesh"
    finally:
        staged.unlink(missing_ok=True)


def test_authored_assetdir_keeps_its_precedence(tmp_path: Path) -> None:
    """``assetdir`` supplies the default for both dirs, so neither gets overwritten."""
    (tmp_path / "assets").mkdir()
    src = _write_device(tmp_path, compiler_attrs='angle="radian" assetdir="assets"')
    staged = Path(prepare_device_xml(str(src)))
    try:
        compiler = ET.parse(str(staged)).getroot().find("compiler")
        assert compiler is not None
        assert Path(compiler.get("assetdir", "")) == tmp_path / "assets"
        assert compiler.get("meshdir") is None
        assert compiler.get("texturedir") is None
    finally:
        staged.unlink(missing_ok=True)


def test_relative_include_paths_are_made_absolute(tmp_path: Path) -> None:
    """An ``<include>`` resolves against the model file, which is now the temp dir."""
    (tmp_path / "assets").mkdir()
    (tmp_path / "assets" / "extra.xml").write_text(
        "<mujocoinclude><option timestep='0.004'/></mujocoinclude>", encoding="utf-8"
    )
    src = _write_device(tmp_path, body="<include file='assets/extra.xml'/>")
    staged = Path(prepare_device_xml(str(src)))
    try:
        include = ET.parse(str(staged)).getroot().find("include")
        assert include is not None
        assert Path(include.get("file", "")) == (tmp_path / "assets" / "extra.xml")
        # The include really loaded: the timestep it carries reaches the model.
        assert mj.MjSpec.from_file(str(staged)).compile().opt.timestep == pytest.approx(0.004)
    finally:
        staged.unlink(missing_ok=True)


@pytest.mark.skipif(os.name == "nt", reason="Windows ACLs, not mode bits, gate directory writes")
@pytest.mark.skipif(hasattr(os, "geteuid") and os.geteuid() == 0, reason="root ignores mode bits")
def test_staging_works_from_a_read_only_device_directory(tmp_path: Path) -> None:
    """The container / HPC case: the package tree cannot be written to."""
    device_dir = tmp_path / "ReadOnlyDevice"
    device_dir.mkdir()
    src = _write_device(device_dir)
    device_dir.chmod(0o500)
    try:
        staged = Path(prepare_device_xml(str(src)))
        try:
            assert mj.MjSpec.from_file(str(staged)).compile().nbody == 2
        finally:
            staged.unlink(missing_ok=True)
    finally:
        device_dir.chmod(0o700)


@needs_myo_sim
def test_a_full_combine_never_writes_into_the_package(monkeypatch, models_dir: Path) -> None:
    """Checked *while* the staged files exist, not after the cleanup runs."""
    from assist_sim import combine as combine_mod
    from assist_sim import load_combined

    device_dir = models_dir / "DephyExoBoot"
    before = sorted(p.name for p in device_dir.iterdir())
    seen: list[str] = []

    def _checked(device_xml: str, strip_meshes: bool = False) -> str:
        staged = preprocess.prepare_device_xml(device_xml, strip_meshes=strip_meshes)
        seen.append(staged)
        assert sorted(p.name for p in device_dir.iterdir()) == before
        return staged

    monkeypatch.setattr(combine_mod, "prepare_device_xml", _checked)
    load_combined("myolegs22", "DephyExoBoot_L1")

    assert len(seen) == 2
    assert sorted(p.name for p in device_dir.iterdir()) == before
    assert not [p for p in seen if Path(p).exists()]
