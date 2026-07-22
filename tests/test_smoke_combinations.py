"""Frozen regression net: every supported MSK x device combination.

Asserts the compiled ``(nq, nu, nbody, nmesh)`` for each combination matches a
frozen expected tuple.  The MSK is composed at runtime by ``myo_sim`` and the
whole pipeline runs in-memory (``spec.delete`` surgery, no XML round-trip), so
these require ``mujoco>=3.3.4``; the ``needs_myo_sim`` gate skips them when
myo_sim isn't installed.

``myolegs26`` (26-muscle) and ``myolegs`` (80-muscle) are both buildable; each is a
passive anatomical torso scaffold over its leg chain, so every combo carries the
torso bodies.  ``myolegs22`` has no source yet (a planned 26->22 reduction) and is
covered by :func:`test_gated_msk_raises`.  Signatures were captured on
``mujoco==3.3.4`` (the pinned floor).
"""

from __future__ import annotations

import mujoco as mj
import pytest

from assist_sim import load_combined
from assist_sim.registry import _msk_available

from .conftest import needs_myo_sim

# (msk_key, device_key) -> (nq, nu, nbody, nmesh).  assist_sim emits model-only
# models: the bundled myosuite scene (floor, backdrop, pedestal, logo) is
# stripped, and prosthetic surgery runs via spec.delete (which cascades the
# subtree + the muscles/tendons/sensors that referenced removed bodies).
EXPECTED = {
    ("myolegs26", "DephyExoBoot_L1"): (47, 28, 52, 50),
    ("myolegs26", "OpenSourceLeg_A_L1"): (46, 22, 39, 41),
    ("myolegs26", "OpenSourceLeg_KA_L1"): (38, 19, 38, 43),
    ("myolegs26", "Anatomics_L1"): (47, 26, 49, 48),
    ("myolegs26", "KFoot_L1"): (47, 21, 40, 43),
    ("myolegs26", "UTAnkleExo_L2"): (65, 28, 44, 43),
    ("myolegs26", "Hippo_L1"): (47, 28, 46, 45),
    ("myolegs26", "HMEDI_L1"): (47, 28, 48, 47),
    ("myolegs", "DephyExoBoot_L1"): (35, 82, 44, 51),
    ("myolegs", "OpenSourceLeg_A_L1"): (33, 69, 31, 42),
    ("myolegs", "OpenSourceLeg_KA_L1"): (29, 56, 33, 44),
    ("myolegs", "Anatomics_L1"): (35, 80, 41, 49),
    ("myolegs", "KFoot_L1"): (34, 68, 32, 44),
    ("myolegs", "UTAnkleExo_L2"): (53, 82, 36, 44),
    ("myolegs", "Hippo_L1"): (35, 82, 38, 46),
    ("myolegs", "HMEDI_L1"): (35, 82, 40, 48),
}

# MSKs that are registered but not yet buildable, and the error resolve raises.
GATED = {
    "myolegs22": ValueError,  # no source yet (planned 26->22 mjspec reduction)
}


@needs_myo_sim
@pytest.mark.parametrize("keys,expected", list(EXPECTED.items()), ids=lambda x: str(x))
def test_combination_signature(keys, expected):
    msk_key, device_key = keys
    model, _ = load_combined(msk_key, device_key)
    actual = (model.nq, model.nu, model.nbody, model.nmesh)
    assert actual == expected


@needs_myo_sim
@pytest.mark.parametrize("keys", list(EXPECTED), ids=lambda x: str(x))
def test_combination_is_simulatable(keys):
    """A compiled combination steps without error (no rollout, just stability)."""
    msk_key, device_key = keys
    model, data = load_combined(msk_key, device_key)
    mj.mj_forward(model, data)
    for _ in range(5):
        mj.mj_step(model, data)


@needs_myo_sim
@pytest.mark.parametrize("msk_key,exc", list(GATED.items()))
def test_gated_msk_raises(msk_key, exc):
    """MSKs without a source raise a clear error (no silent fallback)."""
    with pytest.raises(exc):
        load_combined(msk_key, "DephyExoBoot_L1")


@needs_myo_sim
@pytest.mark.skipif(not _msk_available("myolegs"), reason="requires the torso'd myolegs (mujoco>=3.3.4)")
def test_hmedi_cable_tendons_and_actuators_imported():
    """HMEDI's device-XML <tendon>/<actuator> sections (cable_r/l + Exo_R/L)
    must reach the combined model with the device prefix.  HMEDI needs a torso,
    so it runs against myolegs."""
    model, _ = load_combined("myolegs", "HMEDI_L1")
    actuators = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)}
    tendons = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_TENDON, i) for i in range(model.ntendon)}
    assert "HMEDI_L1_Exo_R" in actuators
    assert "HMEDI_L1_Exo_L" in actuators
    assert "HMEDI_L1_cable_r" in tendons
    assert "HMEDI_L1_cable_l" in tendons


@needs_myo_sim
def test_utankleexo_connect_equalities_and_free_roots():
    """The UT ankle exo integrates via the constraint-clamped attach path:
    free-rooted device bodies (``parent_body: world``) tied to the leg with
    ``<connect>`` equalities -- not the rigid re-parenting the other devices use.
    """
    model, _ = load_combined("myolegs26", "UTAnkleExo_L2")
    prefix = "UTAnkleExo_L2_"

    # Both exo roots keep their freejoint and hang directly under worldbody.
    for root in ("part3_dx", "part3_sx"):
        bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, prefix + root)
        assert bid > 0
        assert model.body_parentid[bid] == 0  # worldbody
    free_joints = {
        mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, j) for j in range(model.njnt) if model.jnt_type[j] == mj.mjtJoint.mjJNT_FREE
    }
    assert prefix + "utexo_root_dx" in free_joints
    assert prefix + "utexo_root_sx" in free_joints

    # Six connect constraints clamp exo bodies to calcn / talus / tibia per side.
    exo_connects = 0
    for e in range(model.neq):
        if model.eq_type[e] != mj.mjtEq.mjEQ_CONNECT:
            continue  # skip myolegs' own joint-coupling equalities
        n1 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, model.eq_obj1id[e]) or ""
        n2 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, model.eq_obj2id[e]) or ""
        if prefix in n1 or prefix in n2:
            exo_connects += 1
    assert exo_connects == 6

    # The cable actuators are imported with the device prefix.
    actuators = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)}
    assert prefix + "part2part3act_dx" in actuators
    assert prefix + "part2part3act_sx" in actuators


def test_hmedi_torso_per_msk_attachment(models_dir):
    """Both torso'd leg models (myolegs, myolegs26) attach hmedi_torso to pelvis
    (not torso) with a compensating pos offset -- myolegs26 reuses the myolegs
    attachment verbatim.  Pure config-resolution test -- doesn't require myo_sim."""
    from assist_sim.config import DeviceConfig

    config = DeviceConfig.from_yaml(str(models_dir / "HMEDI" / "L1config.yaml"))
    default_atts = {a.device_body: a for a in config.resolve_attachments()}
    assert default_atts["hmedi_torso"].parent_body == "torso"
    assert default_atts["hmedi_torso"].pos is None
    for msk in ("myolegs", "myolegs26"):
        atts = {a.device_body: a for a in config.resolve_attachments(msk)}
        assert atts["hmedi_torso"].parent_body == "pelvis", msk
        assert atts["hmedi_torso"].pos == [-0.105, 0.08, 0], msk
        assert atts["hmedi_torso"].quat is None, msk
