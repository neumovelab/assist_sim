"""Device-XML preparation for the in-memory combination pipeline.

Historically this module held an ElementTree "Phase 1" that applied every
removal to the human model before it entered the MjSpec phase.  That is gone:
model surgery now runs in-memory on the human ``MjSpec`` via ``spec.delete``
(see :mod:`assist_sim.combine`), which needs ``mujoco>=3.4``, the package floor.

What remains here is the small amount of *device*-side XML massaging that still
happens at the text level (device models are static XML files that round-trip
fine, unlike the composed human models), plus the :class:`KeyframeData`
container the combiner uses to carry keyframes across surgery by joint name.
"""

from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List


@dataclass
class KeyframeData:
    """A keyframe decomposed into per-joint qpos / qvel slices (by name)."""

    time: float
    qpos_by_joint: Dict[str, List[float]] = field(default_factory=dict)
    qvel_by_joint: Dict[str, List[float]] = field(default_factory=dict)


def _write_temp_xml(root: ET.Element, src_path: Path, tag: str) -> str:
    """Serialize *root* to a temp XML in the system temp dir; return its path.

    The staged copy deliberately does *not* live next to *src_path*.  For a
    pip-installed package that directory is inside ``site-packages``, which is
    read-only or root-owned in containers and on shared HPC nodes, and staging
    runs on the critical path of every combine.  Writing there would also drop
    stray files into the package whenever a process dies before the cleanup in
    :meth:`ModelCombiner.combine` runs.  :func:`_absolutize_asset_paths` makes
    every asset reference absolute first, so the copy resolves its meshes from
    any location.
    """
    fd, tmp_path = tempfile.mkstemp(suffix=".xml", prefix=f"{src_path.stem}__{tag}_")
    os.close(fd)
    ET.ElementTree(root).write(tmp_path, encoding="utf-8", xml_declaration=False)
    return tmp_path


def _absolutize_asset_paths(root: ET.Element, base: Path) -> None:
    """Pin every path MuJoCo resolves against the model file to *base*.

    *base* is the folder that holds the original device XML.  The staged copy
    goes to a temp dir, so ``meshdir`` / ``texturedir`` / ``assetdir`` and any
    ``<include>`` path have to be absolute before the copy is written.  An
    authored value stays relative to *base*; an absent ``meshdir`` /
    ``texturedir`` would otherwise fall back to the temp dir, so both get set
    unless ``assetdir`` already supplies the default.
    """
    compiler = root.find("compiler")
    if compiler is None:  # pragma: no cover - the caller guarantees one
        return

    for attr in ("assetdir", "meshdir", "texturedir"):
        authored = compiler.get(attr)
        if authored is not None:
            compiler.set(attr, str((base / authored).resolve()))

    if compiler.get("assetdir") is None:
        for attr in ("meshdir", "texturedir"):
            if compiler.get(attr) is None:
                compiler.set(attr, str(base))

    for inc in root.iter("include"):
        target = inc.get("file")
        if target and not Path(target).is_absolute():
            inc.set("file", str((base / target).resolve()))


def prepare_device_xml(device_xml: str, strip_meshes: bool = False) -> str:
    """Write a temp copy of the device XML for attachment.

    The copy goes to the system temp dir (see :func:`_write_temp_xml`), not into
    the package, and carries absolute asset paths so its meshes still resolve.

    Always strips ``<keyframe>`` (devices must not contribute keys).  When
    ``strip_meshes`` is True, also removes ``<mesh>`` assets -- used for every
    attach after the first so device meshes are not re-added (which would raise
    a duplicate-name error).  Device-side ``<tendon>`` / ``<actuator>`` sections
    are always stripped from the attachment specs: spanning tendons are not
    handled by ``attach_body``'s subtree-scoped migration, so ``ModelCombiner``
    imports them from the source XML once, after all attachments complete.
    """
    src = Path(device_xml).resolve()
    tree = ET.parse(str(src))
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    if compiler.get("angle") is None:
        compiler.set("angle", "radian")
    # The staged copy lives in a temp dir, and this also keeps device meshes
    # local when the human MSK sets a global meshdir.
    _absolutize_asset_paths(root, src.parent)

    for kf in root.findall("keyframe"):
        root.remove(kf)

    if strip_meshes:
        asset = root.find("asset")
        if asset is not None:
            for mesh in list(asset.findall("mesh")):
                asset.remove(mesh)

    for section in ("tendon", "actuator"):
        for elem in list(root.findall(section)):
            root.remove(elem)

    tag = "dev_nomesh" if strip_meshes else "dev_full"
    return _write_temp_xml(root, src, tag)
