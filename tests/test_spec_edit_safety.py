"""Compiling a working ``MjSpec`` and then editing it corrupts later add/delete pairs.

Two places in the package probe a ``spec.copy()`` rather than the spec they are about to
mutate: ``reduce_legs.reduce_myolegs26_to_22`` (to find orphaned sites) and
``combine.ModelCombiner._decompose_keyframes`` (to read keyframe arrays). Those copies are
load-bearing, and cheap to "simplify" away by someone who does not know why they are there.

Measured on mujoco 3.4 and 3.11: after a compile, the sequence "add three joints, then
delete four others" loses one of the added joints and leaves one delete target in place,
because a delete resolves to the wrong element. The joint count still comes out right, so
the result compiles and looks correct while being a different model.
"""

from __future__ import annotations

import mujoco as mj
import pytest

from assist_sim.combine import ModelCombiner
from assist_sim.registry import _resolve_msk

from .conftest import needs_myo_sim

# Frontal-plane hip DOFs that the 26-muscle model carries and the planar reduction deletes.
FRONTAL_HIPS = ("hip_adduction_r", "hip_rotation_r", "hip_adduction_l", "hip_rotation_l")
ADDED = ("probe_tx", "probe_ty", "probe_tilt")


def _add_then_delete(spec) -> tuple:
    """Replace the free root with three named joints, then drop the frontal hip DOFs.

    Returns ``(added_but_missing, deleted_but_surviving)`` -- both empty when the spec
    behaved.
    """
    full_body = spec.body("Full Body")
    spec.delete(spec.joint("root"))
    for name, jtype, axis in (
        (ADDED[0], mj.mjtJoint.mjJNT_SLIDE, [1.0, 0.0, 0.0]),
        (ADDED[1], mj.mjtJoint.mjJNT_SLIDE, [0.0, 0.0, 1.0]),
        (ADDED[2], mj.mjtJoint.mjJNT_HINGE, [0.0, -1.0, 0.0]),
    ):
        full_body.add_joint(name=name, type=jtype, axis=axis, limited=mj.mjtLimited.mjLIMITED_FALSE)
    for name in FRONTAL_HIPS:
        spec.delete(spec.joint(name))

    model = spec.compile()
    joints = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)}
    return tuple(a for a in ADDED if a not in joints), tuple(j for j in FRONTAL_HIPS if j in joints)


@needs_myo_sim
def test_add_then_delete_is_clean_on_a_fresh_spec():
    """The control: the same edits on a never-compiled spec behave."""
    missing, surviving = _add_then_delete(_resolve_msk("myolegs26"))
    assert missing == () and surviving == ()


@needs_myo_sim
def test_decompose_keyframes_does_not_compile_the_working_spec():
    """After ``_decompose_keyframes``, the spec must still edit correctly.

    This is the regression guard for the copy-probe: swap
    ``human_spec.copy().compile()`` back to ``human_spec.compile()`` and this fails.
    ``myolegs26`` ships no keyframes, so one is added to make the decompose path actually
    compile something.
    """
    spec = _resolve_msk("myolegs26")
    key = spec.add_key()
    key.name = "probe"
    assert spec.keys, "decompose returns early on a keyframe-less spec, so this would not cover it"

    ModelCombiner._decompose_keyframes(spec)

    missing, surviving = _add_then_delete(spec)
    assert missing == (), f"joints added after the decompose probe went missing: {missing}"
    assert surviving == (), f"joints deleted after the decompose probe survived: {surviving}"


@needs_myo_sim
def test_reduction_is_correct_after_a_decompose_probe():
    """The planar reduction still produces its own joints when run on a probed spec.

    ``pelvis_ty`` is the joint the corruption dropped, and a frontal hip DOF is what it
    left behind, so assert on both rather than on the joint count -- the count stays right
    either way, which is what made this silent.
    """
    from assist_sim.reduce_legs import reduce_myolegs26_to_22

    spec = _resolve_msk("myolegs26")
    key = spec.add_key()
    key.name = "probe"
    ModelCombiner._decompose_keyframes(spec)

    model = reduce_myolegs26_to_22(spec).compile()
    joints = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)}
    assert {"pelvis_tx", "pelvis_ty", "pelvis_tilt"} <= joints, "planar root incomplete"
    assert not (set(FRONTAL_HIPS) & joints), "frontal-plane hip DOF survived the reduction"


@needs_myo_sim
@pytest.mark.parametrize("msk,device", [("myolegs22", "OpenSourceLeg_KA_L1"), ("myolegs22", "OpenSourceLeg_A_L1")])
def test_reanchor_combination_with_base_keyframes(msk, device):
    """The combination that exercises both halves: base keyframes *and* a re-anchor.

    ``myolegs22`` ships keyframes, so ``_decompose_keyframes`` compiles, and the OSL configs
    then re-anchor tendons, which is an add-then-delete. The re-anchored muscles must still
    exist with their tendons intact.
    """
    from assist_sim import load_combined

    model, _ = load_combined(msk, device)
    actuators = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)}
    tendons = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_TENDON, i) for i in range(model.ntendon)}
    assert actuators, "no actuators survived"
    # Every muscle actuator must still have the tendon it transmits through.
    for i in range(model.nu):
        if model.actuator_trntype[i] == mj.mjtTrn.mjTRN_TENDON:
            name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_TENDON, int(model.actuator_trnid[i][0]))
            assert name in tendons, f"actuator {i} points at a missing tendon"
