"""Canonical musculoskeletal keyframes (fallback when a base MSK ships none).

Some myo_sim MSK models carry the standard ``stand`` / ``walk_left`` /
``walk_right`` / ``squat`` / ``lunge`` keyframes (``myolegs22``), and some do not
(the 3D-lineage ``myolegs26`` and the 80-muscle ``myolegs``).  The combine
pipeline restores keyframes by joint *name* and defaults everything else to the
compiled ``qpos0``, so one per-joint pose table works across all leg lineages:

* ``myolegs26`` takes the ``myolegs22`` hinge angles directly (same knee sign).
* ``myolegs`` (80-muscle, gait2392 lineage) shares the angles too, but its knee
  flexes with the **opposite sign** (positive flexion, joint range ``[0, +pi]``)
  where the myoLeg knee is negative.  Feeding the raw myoLeg angles hyperextends
  it (a walk pose lands the knee below its range, a squat folds the model over),
  so ``canonical_leg_keyframes`` takes a ``knee_sign`` and the caller flips it for
  a positive-convention knee.  Its extra DOFs (``subtalar_angle_*`` and the
  freejoint root) stay at ``qpos0``.
* Planar-root joints (``pelvis_tx`` / ``pelvis_ty`` / ``pelvis_tilt``) apply on a
  planar-root model and are skipped on a freejoint-root model (they do not exist
  there); the standing height is re-seated downstream by the myoassist compose
  step, so a neutral upright root is a fine starting point.

The two walking poses carry the ``myolegs22`` initial velocity -- a forward
``pelvis_tx`` of 1.5 m/s -- so the reflex controller starts mid-gait instead of
from a standstill (the static poses stay at zero velocity).

Values are the ``myolegs22`` keyframe angles (radians; ``pelvis_ty`` in metres).
Device ``keyframe_overrides`` still apply on top per model.
"""

from typing import Dict

from .preprocess import KeyframeData

# Knee joints whose sign is flipped for a positive-flexion (gait2392) knee.
KNEE_JOINTS = ("knee_angle_r", "knee_angle_l")
# myolegs22 walking initial velocity: forward pelvis translation (m/s).
_WALK_FWD_VEL = 1.5
_CANONICAL_LEG_VEL: Dict[str, Dict[str, float]] = {
    "walk_left": {"pelvis_tx": _WALK_FWD_VEL},
    "walk_right": {"pelvis_tx": _WALK_FWD_VEL},
}

# pose name -> {joint name: value}.  pelvis_tx is 0 in every pose, so it is
# omitted (a missing joint defaults to qpos0 anyway).
_CANONICAL_LEG_POSES: Dict[str, Dict[str, float]] = {
    "stand": {
        "pelvis_ty": 0.91,
        "pelvis_tilt": 0.0,
        "hip_flexion_r": 0.0,
        "knee_angle_r": 0.0,
        "ankle_angle_r": 0.0,
        "mtp_angle_r": -0.0143,
        "hip_flexion_l": 0.0,
        "knee_angle_l": 0.0,
        "ankle_angle_l": -0.0143,
        "mtp_angle_l": 0.0,
    },
    "walk_left": {
        "pelvis_ty": 0.88,
        "pelvis_tilt": -0.262,
        "hip_flexion_r": -0.174,
        "knee_angle_r": -0.436,
        "ankle_angle_r": 0.0,
        "mtp_angle_r": 0.0,
        "hip_flexion_l": 0.436,
        "knee_angle_l": -0.0873,
        "ankle_angle_l": -0.0737,
        "mtp_angle_l": 0.0,
    },
    "walk_right": {
        "pelvis_ty": 0.88,
        "pelvis_tilt": -0.262,
        "hip_flexion_r": 0.436,
        "knee_angle_r": -0.0873,
        "ankle_angle_r": 0.0,
        "mtp_angle_r": -0.0737,
        "hip_flexion_l": -0.174,
        "knee_angle_l": -0.436,
        "ankle_angle_l": 0.0,
        "mtp_angle_l": 0.0,
    },
    "squat": {
        "pelvis_ty": 0.735,
        "pelvis_tilt": -0.611,
        "hip_flexion_r": 1.309,
        "knee_angle_r": -1.309,
        "ankle_angle_r": 0.0,
        "mtp_angle_r": 0.349,
        "hip_flexion_l": 1.309,
        "knee_angle_l": -1.309,
        "ankle_angle_l": 0.349,
        "mtp_angle_l": 0.125,
    },
    "lunge": {
        "pelvis_ty": 0.67,
        "pelvis_tilt": -0.558,
        "hip_flexion_r": 0.698,
        "knee_angle_r": -1.56,
        "ankle_angle_r": 0.0,
        "mtp_angle_r": 0.349,
        "hip_flexion_l": 1.57,
        "knee_angle_l": -1.222,
        "ankle_angle_l": 0.174,
        "mtp_angle_l": 0.06,
    },
}

# A leg MSK is identified by this joint; the canonical set is only injected when
# it is present, so it never lands on a non-leg model (e.g. an arm-only MSK).
LEG_SENTINEL_JOINT = "hip_flexion_r"


def canonical_leg_keyframes(knee_sign: float = 1.0) -> Dict[str, KeyframeData]:
    """Standard leg keyframes as ``{name: KeyframeData}``.

    ``knee_sign`` selects the host knee's flexion convention; it is a **flag, not a
    multiplier**.  Any negative value flips the knee angles, any non-negative value leaves
    them alone -- so pass ``+1`` for the myoLeg (negative-flexion) knee and ``-1`` for a
    positive-flexion (gait2392) knee, and do not expect an intermediate value to scale
    anything.  Without the flip, the shared poses hyperextend a positive-flexion knee.

    Walking poses carry the forward ``pelvis_tx`` initial velocity; static poses stay at
    zero.  Note ``pelvis_tx`` only exists on a planar-root model, so on a freejoint-rooted
    MSK (the default RL build) that velocity has nowhere to land and the walk poses start
    from rest; it applies under ``planar_root=True``, the CO build.
    """

    def _qpos(joints: Dict[str, float]) -> Dict[str, list]:
        return {j: [(-v if j in KNEE_JOINTS and knee_sign < 0 else v)] for j, v in joints.items()}

    return {
        name: KeyframeData(
            time=0.0,
            qpos_by_joint=_qpos(joints),
            qvel_by_joint={j: [v] for j, v in _CANONICAL_LEG_VEL.get(name, {}).items()},
        )
        for name, joints in _CANONICAL_LEG_POSES.items()
    }
