"""Composed upper-body env export round-trip.

Locks in that the composed upper-body envs (wheelchair variants, Auxivo Liftsuit)
serialize to a standalone XML that MuJoCo reloads from disk matching the live build.
These share the composed-model export hazards guarded in ``test_export_reload.py``
(nested unnamed ``<default>`` -> "empty class name"; stripped scene -> dangling
``scene/*.png`` textures), routed through ``export_upper_body_xml`` /
``export_combined_xml``. Requires ``myo_sim``; skipped otherwise.
"""

from __future__ import annotations

import mujoco as mj
import pytest

from assist_sim.upper_body import (
    build_auxivo_liftsuit,
    build_auxivo_liftsuit_spec,
    build_wheelchair,
    build_wheelchair_spec,
    export_upper_body_xml,
)

from .conftest import needs_myo_sim

# (filename stem, spec builder, compiled builder) -- one per composed upper-body env.
CASES = [
    ("wheelchair_both", lambda: build_wheelchair_spec("both"), lambda: build_wheelchair("both")),
    ("wheelchair_right", lambda: build_wheelchair_spec("right"), lambda: build_wheelchair("right")),
    ("wheelchair_left", lambda: build_wheelchair_spec("left"), lambda: build_wheelchair("left")),
    ("wheelchair_muscled", lambda: build_wheelchair_spec("both", "muscled"),
     lambda: build_wheelchair("both", "muscled")),
    ("auxivo_liftsuit", build_auxivo_liftsuit_spec, build_auxivo_liftsuit),
]


@needs_myo_sim
@pytest.mark.parametrize("stem,mkspec,mkbuild", CASES, ids=[c[0] for c in CASES])
def test_upper_body_export_reloads_matching_live(stem, mkspec, mkbuild, tmp_path):
    out = tmp_path / f"{stem}.xml"
    export_upper_body_xml(mkspec(), str(out))
    assert out.exists()

    reloaded = mj.MjModel.from_xml_path(str(out))
    live, _ = mkbuild()
    # The standalone export reproduces the live composed model's structure.
    assert (reloaded.nu, reloaded.ntendon, reloaded.neq, reloaded.nbody) == (
        live.nu,
        live.ntendon,
        live.neq,
        live.nbody,
    )


@needs_myo_sim
@pytest.mark.parametrize("geom", ["upper_exo_geom", "lower_exo_belt_geom", "lower_exo_legs_geom"])
def test_auxivo_exo_geoms_use_radian_euler(geom):
    """The exosuit mesh geoms carry the original's radian eulers (~90-180 deg rotations).

    The fragment XML authors these as radians; without ``<compiler angle="radian"/>``
    MjSpec parses them as degrees, rotating the panels ~1.57 deg (a near-identity quat,
    |w| ~ 1) instead of hugging the trunk -- the "splayed suit" regression. Site positions
    (no euler) still match in that broken state, so this orientation check is the guard.
    """
    m, _ = build_auxivo_liftsuit()
    gid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, geom)
    assert gid >= 0
    # A real (radian) rotation is far from identity (|w| <= ~0.95); the degree misparse
    # rotates by ~1.57 deg, a near-identity quat with |w| ~ 0.9997.
    assert abs(m.geom_quat[gid][0]) < 0.99
