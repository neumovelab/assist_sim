"""Composed upper-body environments (wheelchair, ...).

These are single composed models (a myo_sim human + device hardware attached via
``MjSpec``), not the modular MSK x device compositions the lower-limb devices use.
Anatomical meshes come from the myo_sim import; assist_sim houses only device
hardware meshes under ``models/<Name>/mesh``. See ``models/Wheelchair/CONVERSION.md``
for how the wheelchair maps to the original collaborator environment.
"""

from __future__ import annotations

from importlib.resources import files as _files

import mujoco
import numpy as np
from myo_sim.build.compose import (
    LEFT_ARM_ATTACH_SITE,
    MODEL_REGISTRY,
    RIGHT_ARM_ATTACH_SITE,
    find_site,
    load_legs_spec,
    load_mirrored_left_arm_spec,
    load_passive_torso_spec,
    load_right_arm_spec,
    load_torso_spec,
)

_WHEELCHAIR_XML = str(_files("assist_sim").joinpath("models", "Wheelchair", "wheelchair.xml"))

# Chair placement in the world seat frame (tuned so the pushing hand matches the original).
_CHAIR_SEAT_OFFSET = (0.213, 0.357, 0.48)

# Seated leg pose [rad], baked into the leg geometry and mirrored onto both sides.
_LEG_JOINT_TOKENS = ("hip_", "knee_angle", "ankle_angle", "subtalar_", "mtp_")
_SEATED_LEGS = {
    "hip_flexion": 1.7668,
    "hip_adduction": -0.16757,
    "hip_rotation": -0.04189,
    "knee_angle": 1.6,
    "knee_angle_rotation3": 0.2628,
    "ankle_angle": 0.27315,
    "subtalar_angle": 0.0,
    "mtp_angle": 0.0,
}

# Arm keyframe poses, transcribed from the original env (side-neutral joint names,
# mapped onto this build's _r/_l joints and mirrored onto the active arm(s)).
_ARM_START = {
    "sternoclavicular_r2": -0.0,
    "sternoclavicular_r3": 0.2973,
    "unrotscap_r3": -0.0095,
    "unrotscap_r2": 0.0,
    "acromioclavicular_r2": -0.152,
    "acromioclavicular_r3": 0.2824,
    "acromioclavicular_r1": 0.0,
    "unrothum_r1": -0.0,
    "unrothum_r3": -0.3254,
    "unrothum_r2": 0.0,
    "elv_angle": -0.7352,
    "shoulder_elv": 0.3142,
    "shoulder1_r2": -0.37,
    "shoulder_rot": 1.5442,
    "elbow_flexion": 0.9416,
    "pro_sup": 0.2199,
    "deviation": -0.1745,
    "flexion": -0.5419,
    "cmc_abduction": 0.78,
    "cmc_flexion": -0.6246,
    "mp_flexion": 0.1937,
    "ip_flexion": 0.0087,
    "mcp2_flexion": 1.1704,
    "mcp2_abduction": 0.2356,
    "pm2_flexion": 0.8955,
    "md2_flexion": 1.3432,
    "mcp3_flexion": 1.461,
    "mcp3_abduction": 0.0105,
    "pm3_flexion": 0.7384,
    "md3_flexion": 1.1625,
    "mcp4_flexion": 1.3354,
    "mcp4_abduction": -0.1545,
    "pm4_flexion": 0.1414,
    "md4_flexion": 1.571,
    "mcp5_flexion": 1.2411,
    "mcp5_abduction": -0.2618,
    "pm5_flexion": 0.4477,
    "md5_flexion": 0.597,
}
_ARM_PUSH = {
    "sternoclavicular_r2": -0.465,
    "sternoclavicular_r3": 0.1272,
    "unrotscap_r3": -0.0095,
    "unrotscap_r2": 0.0,
    "acromioclavicular_r2": -0.152,
    "acromioclavicular_r3": 0.5833,
    "acromioclavicular_r1": 0.0,
    "unrothum_r1": -0.552,
    "unrothum_r3": -0.3131,
    "unrothum_r2": 0.0,
    "elv_angle": -0.2639,
    "shoulder_elv": 0.4085,
    "shoulder1_r2": -0.37,
    "shoulder_rot": 2.094,
    "elbow_flexion": 1.0778,
    "pro_sup": 0.11,
    "deviation": 0.4363,
    "flexion": -0.7854,
    "cmc_abduction": 0.78,
    "cmc_flexion": -0.6246,
    "mp_flexion": 0.1937,
    "ip_flexion": 0.0087,
    "mcp2_flexion": 1.1704,
    "mcp2_abduction": 0.2356,
    "pm2_flexion": 0.8955,
    "md2_flexion": 1.3432,
    "mcp3_flexion": 1.461,
    "mcp3_abduction": 0.0105,
    "pm3_flexion": 0.7384,
    "md3_flexion": 1.1625,
    "mcp4_flexion": 1.3354,
    "mcp4_abduction": -0.1545,
    "pm4_flexion": 0.1414,
    "md4_flexion": 1.571,
    "mcp5_flexion": 1.2411,
    "mcp5_abduction": -0.2618,
    "pm5_flexion": 0.4477,
    "md5_flexion": 0.597,
}

_ARM_SIDES = {"both": ("r", "l"), "right": ("r",), "left": ("l",)}


def _is_leg_joint(name: str | None) -> bool:
    return bool(name) and any(tok in name for tok in _LEG_JOINT_TOKENS)


def _arm_joint(orig_name: str, side: str, jointset: set) -> str | None:
    """Map a side-neutral arm joint name onto this build's ``_r``/``_l`` joint set."""
    if side == "r":
        if orig_name in jointset:
            return orig_name
        return orig_name + "_r" if orig_name + "_r" in jointset else None
    return orig_name + "_l" if orig_name + "_l" in jointset else None


def _build_human(arms: str, torso: str = "passive") -> "mujoco.MjSpec":
    """Torso + the selected muscled arm(s). ``torso="passive"`` is a locked muscle-less
    scaffold (default); ``"muscled"`` keeps the active ``myotorso`` (spine joints + muscles)."""
    reg = MODEL_REGISTRY["myoarms"]
    human = load_passive_torso_spec(reg) if torso == "passive" else load_torso_spec(reg)
    for geom in list(human.worldbody.geoms):  # drop myo_sim scene decor
        human.delete(geom)
    for light in list(human.worldbody.lights):
        human.delete(light)
    if arms in ("both", "right"):
        human.attach(load_right_arm_spec(), prefix="", suffix="", site=find_site(human, RIGHT_ARM_ATTACH_SITE))
    if arms in ("both", "left"):
        human.attach(
            load_mirrored_left_arm_spec(reg.mirror_rules), prefix="", suffix="", site=find_site(human, LEFT_ARM_ATTACH_SITE)
        )
    # The torso ships contact pairs for both arms; drop any referencing a missing geom.
    geom_names = {g.name for g in human.geoms if g.name}
    for pair in list(human.pairs):
        if pair.geomname1 not in geom_names or pair.geomname2 not in geom_names:
            human.delete(pair)
    return human


def _freeze_legs_seated(human: "mujoco.MjSpec") -> None:
    """Attach both legs muscle-less, bake ``_SEATED_LEGS`` into the leg geometry, and
    delete every leg joint -- rigid seated legs with no leg DOF (as in the original)."""
    legs = load_legs_spec()
    for actuator in list(legs.actuators):
        legs.delete(actuator)
    for tendon in list(legs.tendons):
        legs.delete(tendon)
    frame = human.body("Full Body").add_frame()
    frame.name = "legs_attach"
    human.attach(legs, prefix="", suffix="", frame=frame)

    wm = human.compile()  # solve the seated configuration on a throwaway compile
    wd = mujoco.MjData(wm)
    mujoco.mj_resetData(wm, wd)
    for j in range(wm.njnt):
        base = (mujoco.mj_id2name(wm, mujoco.mjtObj.mjOBJ_JOINT, j) or "").rsplit("_", 1)[0]
        if base in _SEATED_LEGS:
            wd.qpos[wm.jnt_qposadr[j]] = _SEATED_LEGS[base]
    mujoco.mj_forward(wm, wd)

    baked = {}  # each leg-jointed body's seated transform relative to its parent
    for j in range(wm.njnt):
        if not _is_leg_joint(mujoco.mj_id2name(wm, mujoco.mjtObj.mjOBJ_JOINT, j)):
            continue
        bid = int(wm.jnt_bodyid[j])
        pid = int(wm.body_parentid[bid])
        pconj = np.zeros(4)
        mujoco.mju_negQuat(pconj, wd.xquat[pid])
        rel_pos = np.zeros(3)
        mujoco.mju_rotVecQuat(rel_pos, wd.xpos[bid] - wd.xpos[pid], pconj)
        rel_quat = np.zeros(4)
        mujoco.mju_mulQuat(rel_quat, pconj, wd.xquat[bid])
        baked[mujoco.mj_id2name(wm, mujoco.mjtObj.mjOBJ_BODY, bid)] = (rel_pos.copy(), rel_quat.copy())

    for bname, (rel_pos, rel_quat) in baked.items():
        body = human.body(bname)
        body.alt.type = mujoco.mjtOrientation.mjORIENTATION_QUAT
        body.pos = rel_pos
        body.quat = rel_quat
    for joint in list(human.joints):
        if _is_leg_joint(joint.name):
            human.delete(joint)
    for eq in list(human.equalities):
        if eq.type == mujoco.mjtEq.mjEQ_JOINT and _is_leg_joint(eq.name1 or ""):
            human.delete(eq)


def _attach_chair(human: "mujoco.MjSpec") -> None:
    """Fix the chair rigidly to the torso (``Full Body``) and freejoint the rig so the
    seated human + chair is one free body rolling on its wheels."""
    fb = human.body("Full Body")
    tmp = human.compile()  # Full Body rest pose, to keep the chair's tuned placement
    td = mujoco.MjData(tmp)
    mujoco.mj_resetData(tmp, td)
    mujoco.mj_forward(tmp, td)
    fbid = mujoco.mj_name2id(tmp, mujoco.mjtObj.mjOBJ_BODY, "Full Body")
    fb_conj = np.zeros(4)
    mujoco.mju_negQuat(fb_conj, td.xquat[fbid])
    rel_pos = np.zeros(3)
    mujoco.mju_rotVecQuat(rel_pos, np.asarray(_CHAIR_SEAT_OFFSET) - td.xpos[fbid], fb_conj)

    chair = mujoco.MjSpec.from_file(_WHEELCHAIR_XML)
    for joint in list(chair.joints):  # rigid seat: drop the chair's own freejoint
        if joint.type == mujoco.mjtJoint.mjJNT_FREE:
            chair.delete(joint)
    frame = fb.add_frame()
    frame.pos = rel_pos
    frame.quat = fb_conj
    human.attach(chair, prefix="wc_", frame=frame)
    fb.add_freejoint()


def _add_keyframes(human: "mujoco.MjSpec", arms: str) -> None:
    """Add ``start_return`` + ``pushing`` keyframes; arm pose on the active arm(s)."""
    model = human.compile()
    jointset = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)}

    def qpos_for(arm_pose: dict) -> list:
        q = model.qpos0.copy()  # legs are baked geometry; this only sets arm joints
        for side in _ARM_SIDES[arms]:
            for name, value in arm_pose.items():
                jname = _arm_joint(name, side, jointset)
                if jname is not None:
                    q[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)]] = value
        return q.tolist()

    for name, pose in (("start_return", _ARM_START), ("pushing", _ARM_PUSH)):
        key = human.add_key()
        key.name = name
        key.qpos = qpos_for(pose)


def _add_ground(human: "mujoco.MjSpec") -> None:
    """Add a ground plane at the resting wheel height + a light, and set the 1 ms
    timestep. Terrain composition replaces this ground later."""
    human.option.timestep = 0.001
    tmp = human.compile()
    td = mujoco.MjData(tmp)
    mujoco.mj_resetData(tmp, td)
    mujoco.mj_forward(tmp, td)
    floor_z = float(td.geom_xpos[:, 2].min()) if tmp.ngeom else 0.0

    light = human.worldbody.add_light()
    light.pos = [0, -2, 4]
    light.dir = [0, 0.4, -1]
    ground = human.worldbody.add_geom()
    ground.name = "floor"
    ground.type = mujoco.mjtGeom.mjGEOM_PLANE
    ground.size = [0, 0, 0.05]
    ground.pos = [0, 0, floor_z]
    ground.contype = 1
    ground.conaffinity = 1
    ground.rgba = [0.6, 0.6, 0.6, 1]


def build_wheelchair_spec(arms: str = "both", torso: str = "passive") -> "mujoco.MjSpec":
    """Compose the seated wheelchair env; return the uncompiled ``MjSpec``.

    ``arms`` = ``"both"`` (mirrored bimanual), ``"right"``, or ``"left"``.
    ``torso`` = ``"passive"`` (locked muscle-less scaffold, default) or ``"muscled"``
    (active ``myotorso`` with spine joints + trunk muscles). Rigid baked seated legs +
    the selected muscled arm(s), fixed to the chair, on a ground; ships ``start_return``
    + ``pushing`` keyframes. See ``models/Wheelchair/CONVERSION.md``.
    """
    if arms not in _ARM_SIDES:
        raise ValueError(f"arms must be one of {sorted(_ARM_SIDES)}, got {arms!r}")
    if torso not in ("passive", "muscled"):
        raise ValueError(f"torso must be 'passive' or 'muscled', got {torso!r}")
    human = _build_human(arms, torso)
    _freeze_legs_seated(human)
    _attach_chair(human)
    _add_keyframes(human, arms)
    _add_ground(human)
    return human


def build_wheelchair(arms: str = "both", torso: str = "passive"):
    """Compile the wheelchair env. Returns ``(MjModel, MjData)``."""
    model = build_wheelchair_spec(arms, torso).compile()
    return model, mujoco.MjData(model)
