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
