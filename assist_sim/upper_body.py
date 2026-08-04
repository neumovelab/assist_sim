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

# Chair placement relative to the human worldbody (identity keeps it upright).
_CHAIR_SEAT_OFFSET = (0.22, 0.36, 0.55)

# Seated lock pose for the (muscle-less) legs [rad], applied + pinned per side.
# The original env baked the legs rigid (no leg joints); we pin each leg DOF to a
# tight range so it is rigid in practice while still sourced from myo_sim.
_PIN_EPS = 1e-3
_SEATED_LEGS = {
    "hip_flexion": 1.5,
    "hip_adduction": 0.05,
    "hip_rotation": 0.0,
    "knee_angle": 1.6,
    "ankle_angle": 0.0,
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
    # qpos0 = start_return arm pose (default load = seated recovery, not standing).
    joints = {j.name: j for j in human.joints}
    for side in _ARM_SIDES[arms]:
        for name, value in _ARM_START.items():
            jname = _arm_joint(name, side, joints.keys())
            if jname is not None:
                joints[jname].ref = value
    return human


def _add_locked_legs(human: "mujoco.MjSpec") -> None:
    """Attach both legs as a muscle-less scaffold, pinned rigid in the seated pose."""
    legs = load_legs_spec()
    for actuator in list(legs.actuators):  # strip leg muscles
        legs.delete(actuator)
    for tendon in list(legs.tendons):
        legs.delete(tendon)
    frame = human.body("Full Body").add_frame()
    frame.name = "legs_attach"
    human.attach(legs, prefix="", suffix="", frame=frame)

    joints = {j.name: j for j in human.joints}
    for side in ("r", "l"):
        for base, value in _SEATED_LEGS.items():
            jname = f"{base}_{side}"
            if jname not in joints:
                continue
            joint = joints[jname]
            joint.ref = value  # qpos0 = seated
            joint.range = [value - _PIN_EPS, value + _PIN_EPS]  # pin ~rigid
            joint.limited = 1


def _attach_chair(human: "mujoco.MjSpec") -> None:
    """Rigidly seat the chair into the human; freejoint the rig so it rolls."""
    human.body("Full Body").add_freejoint()
    chair = mujoco.MjSpec.from_file(_WHEELCHAIR_XML)
    for joint in list(chair.joints):  # rigid seat: drop the chair's own freejoint
        if joint.type == mujoco.mjtJoint.mjJNT_FREE:
            chair.delete(joint)
    frame = human.worldbody.add_frame()
    frame.pos = _CHAIR_SEAT_OFFSET
    human.attach(chair, prefix="wc_", frame=frame)


def _add_keyframes(human: "mujoco.MjSpec", arms: str) -> None:
    """Add `start_return` + `pushing` keyframes; arm pose on the active arm(s)."""
    model = human.compile()  # throwaway compile to resolve qpos addresses
    jointset = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)}

    def qpos_for(arm_pose: dict) -> list:
        q = model.qpos0.copy()  # qpos0 already carries the seated legs (leg refs)
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


def build_wheelchair_spec(arms: str = "both") -> "mujoco.MjSpec":
    """Compose the seated wheelchair env and return the (uncompiled) ``MjSpec``.

    ``arms`` selects the muscled arm(s): ``"both"`` (mirrored bimanual), ``"right"``,
    or ``"left"``. Torso is a locked muscle-less scaffold (from ``myoarms``); legs are
    a muscle-less scaffold pinned rigid in a seated pose; only the selected arm(s)
    carry muscles. Ships ``start_return`` + ``pushing`` keyframes whose arm poses are
    transcribed from the original env and mirrored onto the active arm(s).

    The human is the base spec on purpose: attaching it as a child drops the
    cross-body propulsion muscles, so the rigid chair is attached into it instead --
    physically identical to a chair-rooted freejoint.
    """
    if arms not in _ARM_SIDES:
        raise ValueError(f"arms must be one of {sorted(_ARM_SIDES)}, got {arms!r}")
    human = _build_human(arms)
    _add_locked_legs(human)
    _attach_chair(human)
    _add_keyframes(human, arms)
    return human


def build_wheelchair(arms: str = "both"):
    """Compile the wheelchair env. Returns ``(MjModel, MjData)``."""
    model = build_wheelchair_spec(arms).compile()
    return model, mujoco.MjData(model)
