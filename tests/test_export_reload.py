"""Composed-MSK export round-trip + caching.

These lock in that ``export_combined_xml`` produces a standalone XML that
MuJoCo can reload from disk -- the thing that was broken for composed models
(myo_sim's ``to_xml`` emits a nested unnamed ``<default>`` -> "empty class
name", and the stripped scene leaves dangling ``scene/*.png`` textures).  Both
the legs-only (``myolegs26``) and torso'd (``myolegs``) paths are exercised;
the torso'd one is the case that needs the nested-default flatten.

Requires ``myo_sim`` (the MSKs are composed at runtime); skipped otherwise.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import mujoco as mj
import pytest

from assist_sim import load_combined

from .conftest import needs_myo_sim

# (msk, device): legs26 = no torso (scene-strip path); myolegs = torso'd
# (nested-default flatten path).
COMBOS = [
    ("myolegs26", "DephyExoBoot_L1"),
    ("myolegs", "HMEDI_L1"),
]


@needs_myo_sim
@pytest.mark.parametrize("msk,device", COMBOS, ids=lambda x: str(x))
def test_export_reloads_from_disk(msk, device, tmp_path):
    out = tmp_path / f"{msk}__{device}.xml"
    model, _ = load_combined(msk, device, export_xml=str(out))
    assert out.exists()

    reloaded = mj.MjModel.from_xml_path(str(out))
    # The exported standalone model matches the in-memory one.
    assert (reloaded.nq, reloaded.nu, reloaded.nbody, reloaded.ngeom) == (
        model.nq,
        model.nu,
        model.nbody,
        model.ngeom,
    )


@needs_myo_sim
@pytest.mark.parametrize("msk,device", COMBOS, ids=lambda x: str(x))
def test_export_swaps_scene_lighting_for_standalone_visual(msk, device, tmp_path):
    """Export drops the myosuite scene styling but stays viewable on its own.

    The myosuite <visual> headlight (heavy ambient) + <global> camera are scene
    styling; left in, they'd fight a downstream scene's lighting when layered.
    ``_strip_scene_visual`` drops them, then ``_ensure_minimal_visual`` guarantees
    a *soft* headlight (ambient 0.4) + a skybox so the model-only export still
    renders sensibly opened standalone.  Both are overridable: a downstream scene's
    own <visual>/skybox wins when layered (last-include-wins), so downstream still
    owns the final lighting.
    """
    out = tmp_path / f"{msk}__{device}.xml"
    load_combined(msk, device, export_xml=str(out))
    root = ET.parse(out).getroot()

    visual = root.find("visual")
    assert visual is not None
    # No scene camera framing survives.
    assert visual.find("global") is None
    # The headlight is the soft standalone default, not the myosuite scene one.
    headlight = visual.find("headlight")
    assert headlight is not None
    assert headlight.get("ambient") == "0.4 0.4 0.4"
    # A skybox is present so standalone viewing is not a black void.
    asset = root.find("asset")
    assert asset is not None
    assert any(t.get("type") == "skybox" for t in asset.findall("texture"))


@needs_myo_sim
@pytest.mark.parametrize("msk,device", COMBOS, ids=lambda x: str(x))
def test_cache_dir_round_trips(msk, device, tmp_path):
    # First call: cache miss -> composes, writes <key>.xml + <key>.meta.json.
    m1, _ = load_combined(msk, device, cache_dir=tmp_path)
    assert len(list(tmp_path.glob("*.xml"))) == 1
    assert len(list(tmp_path.glob("*.meta.json"))) == 1
    mtime = next(tmp_path.glob("*.xml")).stat().st_mtime_ns

    # Second call: cache hit -> reloads the XML without rewriting it.
    m2, _ = load_combined(msk, device, cache_dir=tmp_path)
    assert next(tmp_path.glob("*.xml")).stat().st_mtime_ns == mtime
    assert (m1.nq, m1.nu, m1.nbody, m1.ngeom) == (m2.nq, m2.nu, m2.nbody, m2.ngeom)
