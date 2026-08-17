"""Reconcile a freejoint 3D-lineage leg MSK to the ``myolegs22`` root frame.

The 2D ``myolegs22`` model roots the skeleton on three *named* planar joints
(``pelvis_tx`` slide-x, ``pelvis_ty`` slide-z/up, ``pelvis_tilt`` hinge about
``-y``) with the pelvis body mounted at quat ``[0.707, 0.707, 0, 0]``. The
3D-lineage models (``myolegs26``, the 80-muscle ``myolegs``) instead float on a
``freejoint`` and mount the pelvis **yawed 90 deg about vertical**
(quat ``[0.5, 0.5, -0.5, -0.5]``). That difference is invisible to joint-angle
readouts but not to anything that reads the pelvis *orientation* in world (the
reflex controller's trunk/`alpha` sensing) or that seats/poses from a world
frame -- so poses, `pelvis_tilt`, forward velocity, and named keyframes authored
for ``myolegs22`` do not transfer.

:func:`to_planar_root` fixes both in one pass, for the controller-optimization
(CO) build only:

1. **Re-orient** the pelvis to the ``myolegs22`` quat. Because the whole leg
   subtree hangs rigidly off the pelvis, this is a pure yaw about vertical -- it
   leaves gravity and the upright stance untouched, only the horizontal facing
   changes.
2. **Swap** the ``freejoint`` for the six named pelvis DOF joints
   (``tx, ty, tz, tilt, list, rotation``) with the ``myolegs22`` axes, so the
   model becomes a structural + frame drop-in for ``myolegs22``.

It is a no-op on a model that already has a planar named root (no freejoint) or
that is not a leg model, so applying it to ``myolegs22`` is safe. It is gated by
a CO-only flag on the combine/compose path, so the RL build (which wants the
floating freejoint base) never sees it.
"""

import mujoco as mj

# The ``myolegs22`` root convention (z-up world).
_PELVIS_QUAT = [0.7071068, 0.7071068, 0.0, 0.0]
# Named root DOFs in qpos order, with the ``myolegs22`` axes.
_ROOT_DOF = (
    ("pelvis_tx", mj.mjtJoint.mjJNT_SLIDE, [1.0, 0.0, 0.0]),
    ("pelvis_ty", mj.mjtJoint.mjJNT_SLIDE, [0.0, 0.0, 1.0]),
    ("pelvis_tz", mj.mjtJoint.mjJNT_SLIDE, [0.0, 1.0, 0.0]),
    ("pelvis_tilt", mj.mjtJoint.mjJNT_HINGE, [0.0, -1.0, 0.0]),
    ("pelvis_list", mj.mjtJoint.mjJNT_HINGE, [1.0, 0.0, 0.0]),
    ("pelvis_rotation", mj.mjtJoint.mjJNT_HINGE, [0.0, 0.0, 1.0]),
)
# Only transform leg models (guards against mangling an unrelated freejoint MSK).
_LEG_SENTINEL_JOINT = "hip_flexion_r"

# jointlimitfrc sensors (name -> joint) the reflex controller reads for the
# joint-limit cost.  The myoLeg models (22/26) ship these; the 80-muscle
# ``myolegs`` does not, so they are added when absent.
_LIMIT_SENSORS = {
    "r_knee_sensor": "knee_angle_r",
    "l_knee_sensor": "knee_angle_l",
    "r_hip_sensor": "hip_flexion_r",
    "l_hip_sensor": "hip_flexion_l",
    "r_ankle_sensor": "ankle_angle_r",
    "l_ankle_sensor": "ankle_angle_l",
}


def _ensure_limit_sensors(spec: mj.MjSpec) -> None:
    """Add any missing knee/hip/ankle jointlimitfrc sensors (no-op if present)."""
    have = {s.name for s in spec.sensors}
    joints = {j.name for j in spec.joints}
    for name, joint in _LIMIT_SENSORS.items():
        if name in have or joint not in joints:
            continue
        sensor = spec.add_sensor()
        sensor.name = name
        sensor.type = mj.mjtSensor.mjSENS_JOINTLIMITFRC
        sensor.objtype = mj.mjtObj.mjOBJ_JOINT
        sensor.objname = joint


def to_planar_root(spec: mj.MjSpec) -> bool:
    """Align a freejoint leg MSK ``spec`` to the ``myolegs22`` root frame in place.

    Returns ``True`` when the transform was applied, ``False`` (no-op) when the
    spec has no freejoint (already a planar named root), no ``pelvis`` body, or is
    not a leg model.
    """
    root_body = free_joint = None
    for body in spec.bodies:
        for joint in body.joints:
            if joint.type == mj.mjtJoint.mjJNT_FREE:
                root_body, free_joint = body, joint
                break
        if free_joint is not None:
            break
    if free_joint is None:
        return False
    if not any(j.name == _LEG_SENTINEL_JOINT for j in spec.joints):
        return False
    pelvis = next((b for b in spec.bodies if b.name == "pelvis"), None)
    if pelvis is None:
        return False

    # 1) re-orient the pelvis frame to the myolegs22 convention (a rigid yaw).
    pelvis.quat = list(_PELVIS_QUAT)

    # 2) replace the freejoint with the six named pelvis DOF joints.
    spec.delete(free_joint)
    for name, jtype, axis in _ROOT_DOF:
        joint = root_body.add_joint()
        joint.name = name
        joint.type = jtype
        joint.axis = list(axis)
        joint.limited = mj.mjtLimited.mjLIMITED_FALSE

    # 3) add the knee/hip/ankle joint-limit sensors the reflex controller reads
    #    but the 80-muscle myolegs does not ship (myolegs26 already has them).
    _ensure_limit_sensors(spec)
    return True
