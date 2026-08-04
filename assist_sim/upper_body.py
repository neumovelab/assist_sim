"""Composed upper-body environments (wheelchair, ...).

Unlike the lower-limb devices (modular MSK x device compositions resolved through
``registry`` / ``load_combined``), the upper-body environments are single composed
models: a myo_sim human + device hardware, attached via ``MjSpec.attach``.

Mesh sourcing follows the 1.0 rule -- anatomical/MSK meshes come from the myo_sim
import; assist_sim houses only device-specific hardware meshes under
``models/<Name>/mesh``. These envs are intentionally not made as modular as the
lower-limb devices; a single composed model + its keyframes is the contract.
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
)

_WHEELCHAIR_XML = str(_files("assist_sim").joinpath("models", "Wheelchair", "wheelchair.xml"))

# Chair placement (world seat frame). Tuned so the pushing-keyframe hand matches the
# original's hand-in-chair position; see _attach_chair.
_CHAIR_SEAT_OFFSET = (0.213, 0.357, 0.48)

# Seated pose baked into the (muscle-less) legs [rad], per side. The original env
# shipped the legs rigid (no leg joints); we reproduce that by baking this pose into
# the leg body geometry and deleting the leg joints (see _freeze_legs_seated).
_LEG_JOINT_TOKENS = ("hip_", "knee_angle", "walker_knee", "ankle_angle", "subtalar_", "mtp_", "patella")
# Seated leg pose hand-tuned in the viewer to rest the soles on the footplates (left
# leg posed; identical values mirror onto the right). Applied per side and baked.
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

# Arm poses transcribed verbatim from the ORIGINAL wheelchair env's keyframes
# (compiled from myosuite fork commit `b0e020f`, right-arm model). Joint names are
# the original's (side-neutral); mapped onto this build's `_r`/`_l` joints, and
# mirrored onto whichever arm(s) are active. Do not hand-edit -- these are the
# collaborator's authored propulsion poses.
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


def _arm_joint(orig_name: str, side: str, jointset: set) -> str | None:
    """Map an original (side-neutral) arm joint name onto this build's joint set."""
    if side == "r":
        if orig_name in jointset:
            return orig_name
        return orig_name + "_r" if orig_name + "_r" in jointset else None
    return orig_name + "_l" if orig_name + "_l" in jointset else None


def _build_human(arms: str) -> "mujoco.MjSpec":
    """Passive (locked, muscle-less) torso + the selected muscled arm(s)."""
    reg = MODEL_REGISTRY["myoarms"]
    human = load_passive_torso_spec(reg)
    for geom in list(human.worldbody.geoms):  # drop myo_sim scene decor
        human.delete(geom)
    for light in list(human.worldbody.lights):
        human.delete(light)
    if arms in ("both", "right"):
        human.attach(load_right_arm_spec(), prefix="", suffix="", site=find_site(human, RIGHT_ARM_ATTACH_SITE))
    if arms in ("both", "left"):
        human.attach(
            load_mirrored_left_arm_spec(reg.mirror_rules),
            prefix="",
            suffix="",
            site=find_site(human, LEFT_ARM_ATTACH_SITE),
        )
    # The torso ships contact pairs for BOTH arms; drop any referencing a geom that
    # is missing when only one arm is attached.
    geom_names = {g.name for g in human.geoms if g.name}
    for pair in list(human.pairs):
        if pair.geomname1 not in geom_names or pair.geomname2 not in geom_names:
            human.delete(pair)
    # NB: arm joints keep ref=0 -- the arm pose comes from the keyframes (net joint
    # rotation = keyframe qpos). Setting ref would relabel zero and cancel the pose.
    return human


def _is_leg_joint(name: str | None) -> bool:
    return bool(name) and any(tok in name for tok in _LEG_JOINT_TOKENS)


def _freeze_legs_seated(human: "mujoco.MjSpec") -> None:
    """Attach both legs muscle-less, then bake the seated pose into the leg body
    geometry and delete every leg joint -- rigid seated legs with no leg DOF, as in
    the original env (which shipped zero leg joints). This is the only way to load
    in a posed configuration: `ref` merely relabels a joint's zero, so a hinge can
    only be posed by baking the rotation into the body frame.
    """
    legs = load_legs_spec()
    for actuator in list(legs.actuators):  # strip leg muscles
        legs.delete(actuator)
    for tendon in list(legs.tendons):
        legs.delete(tendon)
    frame = human.body("Full Body").add_frame()
    frame.name = "legs_attach"
    human.attach(legs, prefix="", suffix="", frame=frame)

    # Solve the seated configuration on a throwaway compile.
    wm = human.compile()
    wd = mujoco.MjData(wm)
    mujoco.mj_resetData(wm, wd)
    for j in range(wm.njnt):
        name = mujoco.mj_id2name(wm, mujoco.mjtObj.mjOBJ_JOINT, j) or ""
        base = name.rsplit("_", 1)[0]  # strip _r/_l -> matches _SEATED_LEGS keys
        if base in _SEATED_LEGS:
            wd.qpos[wm.jnt_qposadr[j]] = _SEATED_LEGS[base]
    mujoco.mj_forward(wm, wd)  # exact posed config (no polycoef re-solve)

    # Record each leg-jointed body's seated transform relative to its parent.
    baked = {}
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

    # Apply the bake to the spec, then delete every leg joint + coupled-knee equality.
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
    """Rigidly fix the chair to the torso (into ``Full Body``) so the human is truly
    seated -- pelvis welded to the chair -- then freejoint ``Full Body`` so the whole
    human+chair rig is one free body that rolls on its wheels (as in the original,
    which freejoints the wheelchair and nests the human rigidly inside it).

    The chair is attached at ``_CHAIR_SEAT_OFFSET`` expressed in the WORLD frame; we
    convert it to a frame on ``Full Body`` (whose rest pose is offset/rotated) so the
    chair keeps its tuned world placement while becoming rigid to the torso.
    """
    fb = human.body("Full Body")
    # Rest-pose world transform of Full Body (to keep the chair's tuned placement).
    tmp = human.compile()
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
    frame.quat = fb_conj  # world seat frame is identity-oriented -> cancel FB's rotation
    human.attach(chair, prefix="wc_", frame=frame)
    fb.add_freejoint()


def _add_keyframes(human: "mujoco.MjSpec", arms: str) -> None:
    """Add `start_return` + `pushing` keyframes; arm pose on the active arm(s)."""
    model = human.compile()  # throwaway compile to resolve qpos addresses
    jointset = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)}

    def qpos_for(arm_pose: dict) -> list:
        q = model.qpos0.copy()  # legs are baked geometry (no leg DOF); this sets arm
        for side in _ARM_SIDES[arms]:
            for name, value in arm_pose.items():
                jname = _arm_joint(name, side, jointset)
                if jname is None:
                    continue
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                q[model.jnt_qposadr[jid]] = value
        return q.tolist()

    for name, pose in (("start_return", _ARM_START), ("pushing", _ARM_PUSH)):
        key = human.add_key()
        key.name = name
        key.qpos = qpos_for(pose)


def _add_ground(human: "mujoco.MjSpec") -> None:
    """Add a ground plane (at the resting wheel height) + a light so the rig rolls,
    and match the original's 1 ms timestep. Terrain composition replaces this later.
    """
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


def build_wheelchair_spec(arms: str = "both") -> "mujoco.MjSpec":
    """Compose the seated wheelchair env and return the (uncompiled) ``MjSpec``.

    ``arms`` selects the muscled arm(s): ``"both"`` (mirrored bimanual), ``"right"``,
    or ``"left"``. Torso is a locked muscle-less scaffold (from ``myoarms``); legs are
    a muscle-less scaffold baked rigid into a seated pose (no leg joints, like the
    original); only the selected arm(s) carry muscles. Ships ``start_return`` +
    ``pushing`` keyframes whose arm poses are transcribed from the original env and
    mirrored onto the active arm(s).

    The human is the base spec on purpose: attaching it as a child drops the
    cross-body propulsion muscles, so the rigid chair is attached into it instead --
    physically identical to a chair-rooted freejoint.
    """
    if arms not in _ARM_SIDES:
        raise ValueError(f"arms must be one of {sorted(_ARM_SIDES)}, got {arms!r}")
    human = _build_human(arms)
    _freeze_legs_seated(human)
    _attach_chair(human)
    _add_keyframes(human, arms)
    _add_ground(human)
    return human


def build_wheelchair(arms: str = "both"):
    """Compile the wheelchair env. Returns ``(MjModel, MjData)``."""
    model = build_wheelchair_spec(arms).compile()
    return model, mujoco.MjData(model)
