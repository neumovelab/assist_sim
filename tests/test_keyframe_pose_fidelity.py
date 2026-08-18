"""The canonical poses must match the MyoAssist 0.1 reference, and stay symmetric.

The rigid 26->22 conversion stage that ``reduce_legs._KEYFRAMES`` was transcribed from
had shifted the right leg's distal pair by one slot: ``ankle_angle_r`` held 0 and
``mtp_angle_r`` held the ankle's value. ``canonical_keyframes`` was transcribed from the
same stage, so the defect reached all four MSK keys once the injection shipped: a squat
stood the right foot on 0.349 rad (20 degrees) of toe extension with a neutral ankle,
while the left leg had the ankle at 0.349 and the toe at 0.125.

The reference model (``myoLeg22_2D_BASELINE.xml``, recovered from the myoassist history)
is symmetric in ``stand`` and ``squat``, mirrors ``walk_left`` / ``walk_right``, and is
asymmetric only in ``lunge``. These tests pin the values and the three invariants, any
one of which would have caught the shift.

Not covered here: ``knee_angle_translation*`` on the gait2392 lineage (``myolegs``,
``myofullbody``). Those are coupled DOFs whose ranges exclude 0, and the model's own
``qpos0`` is 0, so an injected keyframe is no worse than a fresh load. That is a separate,
pre-existing issue.
"""

from __future__ import annotations

import mujoco as mj
import pytest

from assist_sim import load_combined, load_msk

from .conftest import needs_myo_sim

MSK_KEYS = ("myolegs22", "myolegs26", "myolegs", "myofullbody")

# pose -> (ankle_angle_r, mtp_angle_r, ankle_angle_l, mtp_angle_l), from the reference.
REFERENCE = {
    "stand": (-0.0143, 0.0, -0.0143, 0.0),
    "walk_left": (0.0, 0.0, -0.0737, 0.0),
    "walk_right": (-0.0737, 0.0, 0.0, 0.0),
    "squat": (0.349, 0.125, 0.349, 0.125),
    "lunge": (0.349, 0.2, 0.174, 0.06),
}
DISTAL = ("ankle_angle_r", "mtp_angle_r", "ankle_angle_l", "mtp_angle_l")
# Sagittal pairs that a symmetric pose must match on. The knee sign flips per lineage,
# but it flips for both legs, so every pair below survives the flip.
SAGITTAL = ("hip_flexion", "knee_angle", "ankle_angle", "mtp_angle")


def _poses(model):
    """{keyframe name: {joint name: qpos value}} for the hinge joints we author."""
    names = [mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, j) for j in range(model.njnt)]
    out = {}
    for i in range(model.nkey):
        kf = mj.mj_id2name(model, mj.mjtObj.mjOBJ_KEY, i)
        out[kf] = {n: model.key_qpos[i][model.jnt_qposadr[j]] for j, n in enumerate(names)}
    return out


@needs_myo_sim
def test_reduced_myolegs22_matches_the_reference_distal_angles() -> None:
    """``reduce_legs._KEYFRAMES``, before any device or injection is involved."""
    poses = _poses(load_msk("myolegs22")[0])
    assert set(poses) == set(REFERENCE)
    for kf, expected in REFERENCE.items():
        for joint, want in zip(DISTAL, expected):
            assert poses[kf][joint] == pytest.approx(want, abs=1e-4), f"{kf}.{joint}"


@needs_myo_sim
@pytest.mark.parametrize("msk_key", MSK_KEYS)
def test_combined_keyframes_match_the_reference_distal_angles(msk_key: str) -> None:
    """The injected table has to agree with the reduction it was copied from."""
    poses = _poses(load_combined(msk_key, "Tutorial_L1")[0])
    for kf, expected in REFERENCE.items():
        for joint, want in zip(DISTAL, expected):
            if joint not in poses[kf]:
                continue
            assert poses[kf][joint] == pytest.approx(want, abs=1e-4), f"{msk_key} {kf}.{joint}"


@needs_myo_sim
@pytest.mark.parametrize("msk_key", MSK_KEYS)
@pytest.mark.parametrize("kf", ["stand", "squat"])
def test_symmetric_poses_are_left_right_symmetric(msk_key: str, kf: str) -> None:
    """A stand and a squat are symmetric poses; the shift showed up here first."""
    pose = _poses(load_combined(msk_key, "Tutorial_L1")[0])[kf]
    for base in SAGITTAL:
        r, left = f"{base}_r", f"{base}_l"
        if r not in pose or left not in pose:
            continue
        assert pose[r] == pytest.approx(pose[left], abs=1e-6), f"{msk_key} {kf} {base}"


@needs_myo_sim
@pytest.mark.parametrize("msk_key", MSK_KEYS)
def test_the_two_walk_poses_mirror_each_other(msk_key: str) -> None:
    """Same gait phase, opposite legs: each leg of one is the other leg of the other."""
    poses = _poses(load_combined(msk_key, "Tutorial_L1")[0])
    left_pose, right_pose = poses["walk_left"], poses["walk_right"]
    for base in SAGITTAL:
        r, left = f"{base}_r", f"{base}_l"
        if r not in left_pose or left not in left_pose:
            continue
        assert left_pose[r] == pytest.approx(right_pose[left], abs=1e-6), f"{msk_key} {base} r/l"
        assert left_pose[left] == pytest.approx(right_pose[r], abs=1e-6), f"{msk_key} {base} l/r"


@needs_myo_sim
@pytest.mark.parametrize("msk_key", MSK_KEYS)
def test_authored_hinge_angles_stay_inside_their_joint_ranges(msk_key: str) -> None:
    """A corrected pose must not trade one defect for an out-of-range angle."""
    model = load_combined(msk_key, "Tutorial_L1")[0]
    names = [mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, j) for j in range(model.njnt)]
    authored = {f"{base}_{side}" for base in SAGITTAL for side in ("r", "l")}
    for i in range(model.nkey):
        kf = mj.mj_id2name(model, mj.mjtObj.mjOBJ_KEY, i)
        for j, name in enumerate(names):
            if name not in authored or not model.jnt_limited[j]:
                continue
            value = model.key_qpos[i][model.jnt_qposadr[j]]
            lo, hi = model.jnt_range[j]
            assert lo - 1e-9 <= value <= hi + 1e-9, f"{msk_key} {kf} {name}={value} not in [{lo}, {hi}]"
