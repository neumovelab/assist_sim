"""Composed upper-body env export round-trip.

Locks in that the composed upper-body envs (wheelchair variants, Auxivo Liftsuit)
serialize to a standalone XML that MuJoCo reloads from disk matching the live build.
These share the composed-model export hazards guarded in ``test_export_reload.py``
(nested unnamed ``<default>`` -> "empty class name"; stripped scene -> dangling
``scene/*.png`` textures), routed through ``export_upper_body_xml`` /
``export_combined_xml``. Requires ``myo_sim``; skipped otherwise.
"""

from __future__ import annotations

import os
from pathlib import Path

import mujoco as mj
import pytest

from assist_sim.upper_body import (
    build_auxivo_liftsuit,
    build_auxivo_liftsuit_spec,
    build_bionic_bimanual,
    build_bionic_bimanual_spec,
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
    ("bionic_bimanual", build_bionic_bimanual_spec, build_bionic_bimanual),
]

# Faithfulness baseline: the fully-inlined standalone of the original MyoChallenge
# "bionic bimanual" env. Lives outside the package; override via env var, skip if absent.
_BIONIC_BASELINE = Path(
    os.environ.get("BIONIC_BASELINE_XML", r"C:\Users\calde\Work\compile_check\bionic_bimanual.xml")
)


def _nameset(model: "mj.MjModel", obj: "mj.mjtObj", n: int) -> set[str]:
    return {x for x in (mj.mj_id2name(model, obj, i) for i in range(n)) if x}


def _strip_r(names: set[str]) -> set[str]:
    """Normalize the current myo_sim right-arm ``_r`` joint suffix for name-set comparison."""
    return {x[:-2] if x.endswith("_r") else x for x in names}


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
def test_bionic_bimanual_matches_baseline():
    """The relocated bionic-bimanual env matches the original's structure + behavior.

    Counts (nu, nq, nkey) and the actuator/joint name sets match the baseline (the human
    arm joints carry the current myo_sim ``_r`` suffix; muscles + prosthesis actuators are
    identical). ``nbody``/``ngeom`` intentionally differ: the current myo_sim right arm cannot
    self-assemble (chest muscle origins moved to ``myotorso``), so a passive anatomical torso
    replaces the original's decorative body shell (Option A). The full name-set + keyframe
    behavior comparison runs only when the external baseline XML is present.
    """
    m, d = build_bionic_bimanual()
    assert (m.nu, m.nq, m.nkey, m.nsensor) == (80, 71, 4, 1)

    acts = _nameset(m, mj.mjtObj.mjOBJ_ACTUATOR, m.nu)
    joints = _nameset(m, mj.mjtObj.mjOBJ_JOINT, m.njnt)
    bodies = _nameset(m, mj.mjtObj.mjOBJ_BODY, m.nbody)
    # Static-scene structure is present regardless of baseline availability.
    assert {"manip_object", "start", "goal"} <= bodies
    assert sum(a.startswith("prosthesis/") for a in acts) == 17  # 4 MPL arm + 13 hand
    assert "manip_object/freejoint" in joints
    # Grounded: base pedestal + rigid standing legs (pelvis), with NO leg DOF/actuators.
    assert {"pedestal", "pelvis"} <= bodies
    leg_tokens = ("hip_", "knee_angle", "ankle_angle", "subtalar_", "mtp_")
    assert not any(tok in j for j in joints for tok in leg_tokens)
    # The original enables multiccd so the box rests stably on a pillar; simulate from the
    # start keyframe and confirm the object does not fall through (regression on the flag).
    assert bool(m.opt.enableflags & mj.mjtEnableBit.mjENBL_MULTICCD)
    mj.mj_resetDataKeyframe(m, d, 0)
    for _ in range(600):
        mj.mj_step(m, d)
    oid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, "manip_object")
    assert d.xpos[oid][2] > 1.0  # rests near the pillar top (~1.09), not fallen through

    if not _BIONIC_BASELINE.exists():
        pytest.skip(f"baseline {_BIONIC_BASELINE} absent; skipped full name-set/keyframe check")

    bm = mj.MjModel.from_xml_path(str(_BIONIC_BASELINE))
    b_acts = _nameset(bm, mj.mjtObj.mjOBJ_ACTUATOR, bm.nu)
    b_joints = _nameset(bm, mj.mjtObj.mjOBJ_JOINT, bm.njnt)
    b_bodies = _nameset(bm, mj.mjtObj.mjOBJ_BODY, bm.nbody)

    assert (m.nu, m.nq, m.nkey) == (bm.nu, bm.nq, bm.nkey)
    assert acts == b_acts  # muscles + prosthesis actuators identical
    assert _strip_r(joints) == b_joints  # arm joints match modulo the myo_sim _r suffix

    # Prosthesis + object/pillar bodies must match exactly (the human bodies legitimately
    # differ: _r suffix + the extra passive-torso backdrop).
    def _device(names: set[str]) -> set[str]:
        return {x for x in names if x.startswith("prosthesis/") or x in ("manip_object", "start", "goal")}

    assert _device(bodies) == _device(b_bodies)

    # Behavior: object / prosthesis palm / distal hand poses match the baseline per keyframe.
    bd = mj.MjData(bm)
    pairs = [("manip_object", "manip_object"), ("prosthesis/palm", "prosthesis/palm"),
             ("distph2", "distph2_r"), ("distph5", "distph5_r")]
    for k in range(m.nkey):
        mj.mj_resetDataKeyframe(bm, bd, k)
        mj.mj_forward(bm, bd)
        mj.mj_resetDataKeyframe(m, d, k)
        mj.mj_forward(m, d)
        for bn, rn in pairs:
            bi = mj.mj_name2id(bm, mj.mjtObj.mjOBJ_BODY, bn)
            ri = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, rn)
            assert ((bd.xpos[bi] - d.xpos[ri]) ** 2).sum() ** 0.5 < 2e-3  # within 2 mm


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
