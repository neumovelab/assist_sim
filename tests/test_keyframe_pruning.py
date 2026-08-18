"""Canonical keyframe injection, the knee-sign convention, and the RL / CO split.

``myolegs22`` ships the five ``stand`` / ``walk_left`` / ``walk_right`` / ``squat`` /
``lunge`` poses; the 3D-lineage ``myolegs26`` and the 80-muscle ``myolegs`` ship none.
For those two, ``ModelCombiner._rebuild_keyframes`` injects the canonical per-joint table
(:mod:`assist_sim.canonical_keyframes`), because a downstream consumer that seats or
poses from a named keyframe fails outright against ``nkey=0`` -- myoassist asserts
``nkey > 0`` when it reads the initial pose.

This module previously asserted the opposite (``nkey == 0``, "no fabricated keyframe").
That contract was replaced when the canonical fallback landed.

Note the RL / CO split: a device's ``keyframe_overrides`` on ``pelvis_ty`` land only when
that joint exists.  On the default freejoint-rooted build (RL) it does not, so the
override is skipped; under ``planar_root=True`` (CO) the named pelvis DOFs exist and both
the override and the walk velocity apply.
"""

from __future__ import annotations

import mujoco as mj
import pytest

from assist_sim import load_combined
from assist_sim.canonical_keyframes import _CANONICAL_LEG_POSES

from .conftest import needs_myo_sim

CANONICAL = ("stand", "walk_left", "walk_right", "squat", "lunge")

# Joints the canonical table actually writes -- the ones injection is responsible for.
CANONICAL_JOINTS = sorted({j for pose in _CANONICAL_LEG_POSES.values() for j in pose})

# A device that overrides none of CANONICAL_JOINTS, so the limit check below measures the
# canonical table itself against the host MSK rather than against a device's narrowed range
# or re-posed joint.  The devices that do both (DephyExoBoot's rigid toe box, and the four
# that re-pose the lunge knee) are covered separately at the bottom of this module.
CLEAN_DEVICE = "KFoot_L1"

# DephyExoBoot_L1's keyframe_overrides: keyframe name -> authored pelvis_ty (metres).
DEPHY_PELVIS_TY = {
    "stand": 0.96,
    "walk_left": 0.93285,
    "walk_right": 0.93285,
    "squat": 0.77,
    "lunge": 0.72,
}

WALK_FORWARD_VEL = 1.5


def _key_names(model) -> list:
    return [mj.mj_id2name(model, mj.mjtObj.mjOBJ_KEY, i) for i in range(model.nkey)]


def _qpos_of(model, key_index: int, joint: str):
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint)
    if jid < 0:
        return None
    return float(model.key_qpos[key_index][model.jnt_qposadr[jid]])


def _out_of_range(model, joints):
    """Return ``[(keyframe, joint, qpos, lo, hi)]`` for every limited joint violated."""
    bad = []
    for k, kf_name in enumerate(_key_names(model)):
        for joint in joints:
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint)
            if jid < 0 or not model.jnt_limited[jid]:
                continue
            lo, hi = (float(v) for v in model.jnt_range[jid])
            q = float(model.key_qpos[k][model.jnt_qposadr[jid]])
            if not (lo - 1e-6 <= q <= hi + 1e-6):
                bad.append((kf_name, joint, q, lo, hi))
    return bad


@needs_myo_sim
def test_canonical_keyframes_injected_when_base_ships_none():
    """A keyframe-less composed MSK still gets the five canonical poses, by name."""
    for msk in ("myolegs26", "myolegs"):
        model, _ = load_combined(msk, CLEAN_DEVICE)
        assert _key_names(model) == list(CANONICAL), f"{msk}: canonical poses missing or renamed"


@needs_myo_sim
def test_canonical_poses_respect_the_joint_limits():
    """Every injected pose must sit inside the host model's own joint ranges.

    The canonical angles are authored against the myoLeg knee, which flexes negative.
    The 80-muscle ``myolegs`` (gait2392 lineage) flexes positive, so feeding it the raw
    table hyperextended the knee -- a walk pose landed below the range and a squat folded
    the model over.  A clean compile does not catch that: MuJoCo happily compiles a
    keyframe outside a joint limit.  ``myofullbody`` shares the positive-flexion knee, so it
    is covered too -- an earlier sweep omitted it and under-reported the spread.
    """
    for msk in ("myolegs22", "myolegs26", "myolegs", "myofullbody"):
        model, _ = load_combined(msk, CLEAN_DEVICE)
        bad = _out_of_range(model, CANONICAL_JOINTS)
        assert not bad, f"{msk}: " + "; ".join(f"{k}/{j} {q:+.4f} not in [{lo:+.4f}, {hi:+.4f}]" for k, j, q, lo, hi in bad)


@needs_myo_sim
def test_canonical_knee_sign_follows_the_model_convention():
    """The squat knee flexes in whichever direction the host model's range allows."""
    for msk, want in (("myolegs26", -1.309), ("myolegs", +1.309)):
        model, _ = load_combined(msk, CLEAN_DEVICE)
        squat = _key_names(model).index("squat")
        got = _qpos_of(model, squat, "knee_angle_r")
        assert got is not None, f"{msk}: no knee_angle_r"
        assert abs(got - want) < 1e-3, f"{msk}: squat knee {got:+.4f}, expected {want:+.4f}"


@needs_myo_sim
def test_canonical_knee_sign_survives_amputation():
    """The sign probe must not depend on the amputated side's knee still existing.

    A transfemoral amputation deletes ``knee_angle_r``.  Probing only that joint found it
    absent, skipped the flip, and drove the *intact* left knee of the 80-muscle model to
    -1.309 against a ``[0, +2.09]`` range.
    """
    for msk in ("myolegs26", "myolegs"):
        model, _ = load_combined(msk, "OpenSourceLeg_KA_L1")
        assert mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "knee_angle_r") < 0, (
            f"{msk}: knee_angle_r survived, so this no longer covers the amputated case"
        )
        bad = _out_of_range(model, ["knee_angle_l"])
        assert not bad, f"{msk}: intact knee out of range -- " + "; ".join(
            f"{k} {q:+.4f} not in [{lo:+.4f}, {hi:+.4f}]" for k, _, q, lo, hi in bad
        )


@needs_myo_sim
def test_co_planar_root_applies_overrides_and_walk_velocity():
    """Under ``planar_root=True`` (CO), the named pelvis DOFs exist, so overrides land."""
    for msk in ("myolegs26", "myolegs"):
        model, _ = load_combined(msk, "DephyExoBoot_L1", planar_root=True)
        names = _key_names(model)
        assert names == list(CANONICAL), f"{msk}: canonical poses lost under planar_root"
        for kf_name, want in DEPHY_PELVIS_TY.items():
            got = _qpos_of(model, names.index(kf_name), "pelvis_ty")
            assert got is not None and abs(got - want) < 1e-5, f"{msk}/{kf_name}: pelvis_ty {got}, expected {want}"
        tx = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pelvis_tx")
        assert tx >= 0, f"{msk}: planar_root did not add pelvis_tx"
        for walk in ("walk_left", "walk_right"):
            vel = float(model.key_qvel[names.index(walk)][model.jnt_dofadr[tx]])
            assert abs(vel - WALK_FORWARD_VEL) < 1e-6, f"{msk}/{walk}: pelvis_tx qvel {vel}, expected {WALK_FORWARD_VEL}"


@needs_myo_sim
def test_rl_path_skips_overrides_for_absent_joints():
    """On the default freejoint build (RL) there is no ``pelvis_ty``, so the override is
    a silent no-op and the walk velocity has nowhere to land.

    Height is re-seated downstream by myoassist's compose step instead, so the skip is
    intended -- but it *is* silent, which is why a typo in ``keyframe_overrides`` surfaces
    nowhere.
    """
    for msk in ("myolegs26", "myolegs"):
        model, _ = load_combined(msk, "DephyExoBoot_L1")
        assert mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pelvis_ty") < 0, f"{msk} unexpectedly has pelvis_ty"
        assert _key_names(model) == list(CANONICAL), f"{msk}: canonical poses lost"


@needs_myo_sim
def test_device_joint_override_does_not_exclude_its_own_poses():
    """A device that narrows a joint range must not exclude the poses it ships with.

    DephyExoBoot restricts mtp to [-0.0145, 0.2] because the boot's toe box is rigid, but
    shipped no pose overrides, so three of its five poses sat outside its own range.  The
    poses are now clamped to the boot's travel, matching what Humotech and STRIDE already do.
    """
    for msk in ("myolegs22", "myolegs26", "myolegs", "myofullbody"):
        model, _ = load_combined(msk, "DephyExoBoot_L1")
        bad = _out_of_range(model, CANONICAL_JOINTS)
        assert not bad, f"{msk}: " + "; ".join(f"{k}/{j} {q:+.4f} not in [{lo:+.4f}, {hi:+.4f}]" for k, j, q, lo, hi in bad)


@needs_myo_sim
@pytest.mark.parametrize("msk", ["myolegs", "myofullbody"])
def test_device_keyframe_override_respects_the_knee_sign(msk):
    """An authored knee override must respect the host model's flexion convention.

    ``keyframe_overrides`` are applied verbatim after the canonical pose, with no sign flip,
    so the myoLeg-negative lunge knee these four devices author used to hyperextend the
    positive-flexion knee of ``myolegs`` and ``myofullbody``.  Each now carries a per-MSK
    block for those two lineages, merged onto its default.
    """
    for device in ("Tutorial_L1", "Humotech_L1", "OpenExo_L1", "STRIDE_L2"):
        model, _ = load_combined(msk, device)
        bad = _out_of_range(model, ["knee_angle_l", "knee_angle_r"])
        assert not bad, f"{device}: " + "; ".join(f"{k}/{j} {q:+.4f} not in [{lo:+.4f}, {hi:+.4f}]" for k, j, q, lo, hi in bad)
