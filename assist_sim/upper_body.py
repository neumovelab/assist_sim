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

# Chair placement relative to the human worldbody (identity orientation keeps the
# chair upright; seats the pelvis on the pan). Tunable; see models/Wheelchair/TODO.md.
_CHAIR_SEAT_OFFSET = (0.22, 0.36, 0.55)

# Seated lock pose for the (muscle-less) legs, applied per side [rad].
_SEATED_LEGS = {
    "hip_flexion": 1.5,
    "hip_adduction": 0.0,
    "hip_rotation": 0.0,
    "knee_angle": 1.6,
    "ankle_angle": 0.0,
    "subtalar_angle": 0.0,
    "mtp_angle": 0.0,
}

# Arm poses (right-side joint values; mirrored onto whichever arm(s) are active).
# start_return == resting/recovery, pushing == hands driving the pushrim. Seeded
# from the original env; fine-tune against the rim (models/Wheelchair/TODO.md).
_ARM_START = {"shoulder_elv": 0.35, "elbow_flexion": 0.6, "elv_angle": 1.2, "pro_sup": 1.0}
_ARM_PUSH = {"shoulder_elv": 0.9, "elbow_flexion": 1.3, "elv_angle": 1.3, "pro_sup": 1.2}

_ARM_SIDES = {"both": ("r", "l"), "right": ("r",), "left": ("l",)}


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
    # The torso ships contact pairs for BOTH arms; drop any that reference a geom
    # missing when only one arm is attached.
    geom_names = {g.name for g in human.geoms if g.name}
    for pair in list(human.pairs):
        if pair.geomname1 not in geom_names or pair.geomname2 not in geom_names:
            human.delete(pair)
    # qpos0 = start_return arm pose (so the default load pose is a seated recovery).
    joints = {j.name: j for j in human.joints}
    for side in _ARM_SIDES[arms]:
        for base, value in _ARM_START.items():
            jname = f"{base}_{side}"
            if jname in joints:
                joints[jname].ref = value
    return human


def _add_locked_legs(human: "mujoco.MjSpec") -> None:
    """Attach both legs as a muscle-less scaffold, frozen in the seated pose."""
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
            joints[jname].ref = value  # qpos0 = seated (so it loads seated, not standing)
            eq = human.add_equality()  # lock each primary leg DOF to the seated angle
            eq.type = mujoco.mjtEq.mjEQ_JOINT
            eq.name = f"lock_{jname}"
            eq.name1 = jname
            eq.name2 = ""
            eq.data[0] = value


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
    """Add `start_return` + `pushing` keyframes, arm pose mirrored to active arm(s)."""
    model = human.compile()  # throwaway compile to resolve qpos addresses

    def qpos_for(arm_pose: dict) -> list:
        q = model.qpos0.copy()
        pairs = []  # seed the seated leg angles too, so qpos0 matches the lock
        for side in ("r", "l"):
            for base, value in _SEATED_LEGS.items():
                pairs.append((f"{base}_{side}", value))
        for side in _ARM_SIDES[arms]:
            for base, value in arm_pose.items():
                pairs.append((f"{base}_{side}", value))
        for name, value in pairs:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                q[model.jnt_qposadr[jid]] = value
        return q.tolist()

    for name, pose in (("start_return", _ARM_START), ("pushing", _ARM_PUSH)):
        key = human.add_key()
        key.name = name
        key.qpos = qpos_for(pose)


def build_wheelchair_spec(arms: str = "both") -> "mujoco.MjSpec":
    """Compose the seated wheelchair env and return the (uncompiled) ``MjSpec``.

    ``arms`` selects which muscled arm(s) drive the chair: ``"both"`` (mirrored
    bimanual), ``"right"``, or ``"left"``. The torso is a locked muscle-less
    scaffold (from ``myoarms``); the legs are a locked muscle-less seated scaffold;
    only the selected arm(s) carry muscles. Ships ``start_return`` + ``pushing``
    keyframes with the push pose mirrored onto the active arm(s).

    The human is the base spec on purpose: attaching it as a child drops the
    cross-body propulsion muscles (see git history / TODO), so the rigid chair is
    attached into it instead -- physically identical to a chair-rooted freejoint.
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
