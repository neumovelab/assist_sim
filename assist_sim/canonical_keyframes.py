"""Canonical musculoskeletal keyframes (fallback when a base MSK ships none).

Some myo_sim MSK models carry the standard ``stand`` / ``walk_left`` /
``walk_right`` / ``squat`` / ``lunge`` keyframes (``myolegs22``), and some do not
(the 3D-lineage ``myolegs26`` and the 80-muscle ``myolegs``).  The combine
pipeline restores keyframes by joint *name* and defaults everything else to the
compiled ``qpos0``, so one per-joint pose table works across all leg lineages:

* ``myolegs26`` takes the ``myolegs22`` hinge angles directly.
* ``myolegs`` (80-muscle) is *derived* from the same angles; its extra DOFs
  (``subtalar_angle_*`` and the freejoint root) stay at ``qpos0``.
* Planar-root joints (``pelvis_tx`` / ``pelvis_ty`` / ``pelvis_tilt``) apply on a
  planar-root model and are skipped on a freejoint-root model (they do not exist
  there); the standing height is re-seated downstream by the myoassist compose
  step, so a neutral upright root is a fine starting point.

Values are the ``myolegs22`` keyframe angles (radians; ``pelvis_ty`` in metres).
Device ``keyframe_overrides`` still apply on top per model.
"""

from typing import Dict

from .preprocess import KeyframeData

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


def canonical_leg_keyframes() -> Dict[str, KeyframeData]:
    """Standard leg keyframes as ``{name: KeyframeData}`` (qvel left at zero)."""
    return {
        name: KeyframeData(time=0.0, qpos_by_joint={j: [v] for j, v in joints.items()})
        for name, joints in _CANONICAL_LEG_POSES.items()
    }
