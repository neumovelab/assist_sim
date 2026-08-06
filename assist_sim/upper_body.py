"""Composed upper-body environments (wheelchair, Auxivo Liftsuit, MPL).

These are single composed models (a myo_sim human + device hardware attached via
``MjSpec``), not the modular MSK x device compositions the lower-limb devices use.
Anatomical meshes come from the myo_sim import; assist_sim houses only device
hardware meshes under ``models/<Name>/mesh``. See each ``models/<Name>/CONVERSION.md``
for how an env maps to the original collaborator environment.

Each composed env has a ``build_*_spec()`` (uncompiled ``MjSpec``) and a ``build_*()``
(compiled ``(MjModel, MjData)``). To serialize a composed env to a standalone,
reloadable XML, use :func:`export_upper_body_xml` -- never a raw ``spec.to_xml()``,
which emits anonymous ``<default>`` blocks and dangling asset paths that fail to reload.
"""

from __future__ import annotations

import json
import re
from importlib.resources import files as _files
from pathlib import Path

import mujoco
import numpy as np

# ``myo_sim`` is imported lazily inside the composed builders below (never at module top),
# so ``import assist_sim.upper_body`` and ``build_mpl()`` (which needs no myo_sim human) work
# without myo_sim installed -- matching the library convention (``loading.py`` does the same).

_WHEELCHAIR_XML = str(_files("assist_sim").joinpath("models", "Wheelchair", "wheelchair.xml"))
_MPL_XML = str(_files("assist_sim").joinpath("models", "MPL", "scenes", "sally.xml"))
_AUXIVO_XML = str(_files("assist_sim").joinpath("models", "AuxivoLiftsuit", "auxivo_liftsuit.xml"))
_AUXIVO_MESH = str(_files("assist_sim").joinpath("models", "AuxivoLiftsuit", "mesh"))

_BIONIC_SCENE_XML = str(_files("assist_sim").joinpath("models", "MPL", "scenes", "bionic_bimanual.xml"))
_BIONIC_KEYFRAMES_JSON = str(_files("assist_sim").joinpath("models", "MPL", "scenes", "bionic_bimanual_keyframes.json"))
_MPL_MESH_DIR = str(_files("assist_sim").joinpath("models", "MPL", "meshes"))
_YCB_DIR = str(_files("assist_sim").joinpath("models", "YCB"))

# The original MyoChallenge env fixes the biological right arm via `full_body` at
# (-0.025, 0.1, 1.40); the current myo_sim right arm is registered to the same world
# frame by aligning this build's `humerus_r` to the original's rest-pose humerus world
# pose, so the arm meets the fixed-world prosthesis / object / pillars exactly.
_BIONIC_HUMERUS_POS = (-0.16319397046550244, 0.09828382285253014, 1.392981079773622)
_BIONIC_HUMERUS_QUAT = (0.5003981633553667, 0.49999984146591736, -0.49999984146591736, -0.49960183664463337)

# Base pedestal (matches the original myosuite scene pedestal: radius 1.05, half-height
# 0.205). The feet + start/goal pillars rest on the top cap surface, which sits this far
# above the pedestal body center (half-height + the two 6 mm cap disks).
_PED_TOP_FROM_CENTER = 0.205 + 2 * 0.006

# The exosuit hardware was authored against the original env's `torso` world pose; the
# suit is placed by the rigid map taking this pose to the current build's `torso` pose.
_AUXIVO_AUTHOR_TORSO_POS = (-0.099, 0.099977354, 1.041199998)
_AUXIVO_AUTHOR_TORSO_QUAT = (0.707034778, 0.707178777, 0.0, 0.0)
# Body welds coupling the suit to the trunk, with the original weld anchors.
_AUXIVO_WELDS = (("torso", "exo_torso", (-0.1, 0.2, 0.0)), ("lumbar4", "exo_lumbar4", (0.0, 0.0, 0.0)))

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


def _strip_scene_decor(human: "mujoco.MjSpec") -> None:
    """Drop the myo_sim scene decor so the env is model-only (a downstream scene/terrain
    supplies the ground + lighting) and exports cleanly.

    Delegates to the shared ``utils.strip_myosuite_scene_spec``, which removes worldbody
    geoms + lights + **cameras** and prunes now-orphaned backdrop/logo meshes -- more
    thorough than a geoms+lights-only strip (which left scene cameras + meshes in the
    Auxivo export)."""
    from .utils import strip_myosuite_scene_spec

    strip_myosuite_scene_spec(human)


def _build_human(arms: str, torso: str = "passive") -> "mujoco.MjSpec":
    """Torso + the selected muscled arm(s). ``torso="passive"`` is a locked muscle-less
    scaffold (default); ``"muscled"`` keeps the active ``myotorso`` (spine joints + muscles)."""
    from myo_sim.build.compose import (
        LEFT_ARM_ATTACH_SITE,
        MODEL_REGISTRY,
        RIGHT_ARM_ATTACH_SITE,
        find_site,
        load_mirrored_left_arm_spec,
        load_passive_torso_spec,
        load_right_arm_spec,
        load_torso_spec,
    )

    reg = MODEL_REGISTRY["myoarms"]
    human = load_passive_torso_spec(reg) if torso == "passive" else load_torso_spec(reg)
    _strip_scene_decor(human)
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
    from myo_sim.build.compose import load_legs_spec

    legs = load_legs_spec()
    for sensor in list(legs.sensors):  # no leg DOF here -> drop the legs' proprioceptive/touch sensors
        legs.delete(sensor)
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
    # Seat the floor at the lowest COLLISION-geom surface (AABB corner), not the geom
    # center (too high -> interpenetration) nor the lowest of all geoms (a low visual
    # geom would drag it too low), so the wheels rest on the plane.
    floor_z = _lowest_geom_z(human, collision_only=True)

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


def build_mpl():
    """Load the MPL (Modular Prosthetic Limb) bimanual robot env (``sally``).

    Unlike the other upper-body envs, MPL is a self-contained *robotic* model (its
    own meshes + actuators, no myo_sim human) relocated verbatim from the collaborator
    fork; it is loaded directly rather than composed. Returns ``(MjModel, MjData)``.
    See ``models/MPL/CONVERSION.md``.
    """
    model = mujoco.MjModel.from_xml_path(_MPL_XML)
    return model, mujoco.MjData(model)


def _q_mul(a, b):
    r = np.zeros(4)
    mujoco.mju_mulQuat(r, np.asarray(a, float), np.asarray(b, float))
    return r


def _q_conj(q):
    r = np.zeros(4)
    mujoco.mju_negQuat(r, np.asarray(q, float))
    return r


def _q_rot(q, v):
    r = np.zeros(3)
    mujoco.mju_rotVecQuat(r, np.asarray(v, float), np.asarray(q, float))
    return r


def _inline_includes(path: str, base_dir: str) -> str:
    """Textually resolve ``<include>`` (MuJoCo semantics: every include is relative to
    the MAIN model file, i.e. ``base_dir``) and strip the ``<mujocoinclude>`` wrappers,
    so a scene of nested include fragments becomes one string loadable via
    ``MjSpec.from_string`` after token replacement."""
    text = Path(path).read_text(encoding="utf-8", errors="replace")
    text = re.sub(r"<\?xml[^>]*\?>", "", text)
    text = re.sub(r"</?mujocoinclude[^>]*>", "", text)
    return re.sub(
        r'<include\s+file="([^"]+)"\s*/>',
        lambda m: _inline_includes(str(Path(base_dir) / m.group(1)), base_dir),
        text,
    )


def _bionic_scene_spec() -> "mujoco.MjSpec":
    """Load the static bionic-bimanual scene (MPL left prosthesis + YCB ``manip_object``
    + start/goal mocap pillars + touch sensor) as an ``MjSpec``, inlining its includes
    and replacing the ``__MPLMESH__`` / ``__YCB__`` / ``__MYOSIM__`` dir tokens with
    absolute, forward-slashed paths."""
    text = _inline_includes(_BIONIC_SCENE_XML, str(Path(_BIONIC_SCENE_XML).parent))
    text = (
        text.replace("__MPLMESH__", _MPL_MESH_DIR.replace("\\", "/"))
        .replace("__YCB__", _YCB_DIR.replace("\\", "/"))
        .replace("__MYOSIM__", str(_files("myo_sim").joinpath("models")).replace("\\", "/"))
    )
    return mujoco.MjSpec.from_string(text)


def _align_bionic_arm(human: "mujoco.MjSpec") -> None:
    """Reposition the fixed human root so ``humerus_r`` coincides with the original
    MyoChallenge arm world pose. The whole arm (a faithful ``_r``-renamed copy of the
    original) then registers to the fixed-world prosthesis/object/pillars exactly."""
    probe = human.compile()
    pd = mujoco.MjData(probe)
    mujoco.mj_forward(probe, pd)
    hid = mujoco.mj_name2id(probe, mujoco.mjtObj.mjOBJ_BODY, "humerus_r")
    rid = mujoco.mj_name2id(probe, mujoco.mjtObj.mjOBJ_BODY, "Full Body")
    hp, hq = np.array(pd.xpos[hid]), np.array(pd.xquat[hid])
    rp, rq = np.array(pd.xpos[rid]), np.array(pd.xquat[rid])
    # rigid map M taking the current humerus pose to the original's, applied to the root:
    mq = _q_mul(_BIONIC_HUMERUS_QUAT, _q_conj(hq))  # R = q_target * conj(q_humerus)
    mp = np.asarray(_BIONIC_HUMERUS_POS) - _q_rot(mq, hp)  # t = p_target - R * p_humerus
    fb = human.body("Full Body")
    fb.alt.type = mujoco.mjtOrientation.mjORIENTATION_QUAT
    fb.pos = (mp + _q_rot(mq, rp)).tolist()
    fb.quat = _q_mul(mq, rq).tolist()


def _freeze_legs_standing(human: "mujoco.MjSpec") -> None:
    """Attach both myo_sim legs muscle-less and freeze them rigid in the standing (qpos0)
    pose. Leg actuators/tendons/sensors and every leg joint (+ its knee couplings) are
    deleted, so the figure gains anatomical legs to stand on the base but NO leg DOF or
    actuators -- as in the original, whose lower body was a decorative shell. Keeps the
    exact nu / nq / nsensor match; adds only rigid leg bodies for grounding."""
    from myo_sim.build.compose import load_legs_spec

    legs = load_legs_spec()
    for sensor in list(legs.sensors):  # the legs ship proprioceptive sensors; env has only touch
        legs.delete(sensor)
    for actuator in list(legs.actuators):
        legs.delete(actuator)
    for tendon in list(legs.tendons):
        legs.delete(tendon)
    frame = human.body("Full Body").add_frame()
    frame.name = "legs_attach"
    human.attach(legs, prefix="", suffix="", frame=frame)
    # standing pose == qpos0 (authored frames); delete leg DOF -> rigid standing legs
    for joint in list(human.joints):
        if _is_leg_joint(joint.name):
            human.delete(joint)
    for eq in list(human.equalities):
        if eq.type == mujoco.mjtEq.mjEQ_JOINT and _is_leg_joint(eq.name1 or ""):
            human.delete(eq)


def _lowest_geom_z(spec: "mujoco.MjSpec", collision_only: bool = False) -> float:
    """World-z of the lowest surface point (AABB corner) of any geom in a throwaway
    compile of ``spec``. ``collision_only`` restricts to geoms that can contact
    (``contype``/``conaffinity`` set) -- use it to seat a ground plane under the lowest
    *colliding* geom, ignoring purely visual geoms that would drag the plane too low."""
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    zmin = np.inf
    for g in range(m.ngeom):
        if collision_only and m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:
            continue
        aabb = np.array(m.geom_aabb[g]).reshape(2, 3)
        corners = aabb[0] + signs * aabb[1]
        world = corners @ np.array(d.geom_xmat[g]).reshape(3, 3).T + np.array(d.geom_xpos[g])
        zmin = min(zmin, float(world[:, 2].min()))
    return zmin


def _ground_bionic(human: "mujoco.MjSpec", feet_low: float) -> None:
    """Seat the base pedestal so its top cap rests exactly under the feet, and trim the
    start/goal pillars to run from the pedestal top up to their original top height (the
    object rest height -- hence every keyframe -- is unchanged)."""
    ped = human.body("pedestal")
    ped.pos = [0.0, 0.0, feet_low - _PED_TOP_FROM_CENTER]
    for name in ("start", "goal"):
        body = human.body(name)
        bz = float(body.pos[2])
        for geom in body.geoms:  # each pillar body carries a single cylinder geom
            top = bz + float(geom.pos[2]) + float(geom.size[1])  # current pillar top (world z)
            geom.size = [float(geom.size[0]), (top - feet_low) / 2.0, float(geom.size[2])]
            geom.pos = [float(geom.pos[0]), float(geom.pos[1]), (top + feet_low) / 2.0 - bz]


def _add_bionic_keyframes(human: "mujoco.MjSpec") -> None:
    """Add the 4 original keyframes by NAME: each ORIGINAL joint value is written onto
    this build's qpos layout (human arm joints gain a ``_r`` suffix; ``prosthesis/*`` and
    ``manip_object/freejoint`` match by exact name). See ``bionic_bimanual_keyframes.json``."""
    model = human.compile()
    jnames = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    adr = {jnames[i]: int(model.jnt_qposadr[i]) for i in range(model.njnt)}
    keyframes = json.loads(Path(_BIONIC_KEYFRAMES_JSON).read_text(encoding="utf-8"))["keyframes"]
    for kf in keyframes:
        qmap = kf["qpos_by_joint"]
        q = model.qpos0.copy()
        for jname in jnames:
            vals = qmap.get(jname)
            if vals is None and jname.endswith("_r"):
                vals = qmap.get(jname[:-2])  # original side-neutral name for this arm joint
            if vals is None:
                continue
            q[adr[jname] : adr[jname] + len(vals)] = vals
        key = human.add_key()
        key.name = kf["name"]
        key.qpos = q.tolist()


def build_bionic_bimanual_spec() -> "mujoco.MjSpec":
    """Compose the MyoChallenge "bionic bimanual" env; return the uncompiled ``MjSpec``.

    A biological RIGHT arm (myo_sim, on a passive anatomical torso with rigid standing
    legs) faces an MPL LEFT prosthetic arm across a YCB gelatin box (``manip_object``,
    freejoint) between two ``start``/``goal`` mocap pillars, standing on a myosuite-sized
    base pedestal, with a touch sensor on the object. The static half (prosthesis, object,
    pillars, pedestal, sensor) is ``models/MPL/scenes/bionic_bimanual.xml``; the human is
    composed here because the current myo_sim ``myoarm_r`` cannot self-assemble
    (chest/thorax muscle origins moved to ``myotorso`` in 2026-06), so the original's
    decorative body shell is replaced by a passive-torso + rigid-legs backdrop. The arm is
    aligned to the original world pose and the 4 original keyframes are transcribed by
    joint name, so object / prosthesis / hand poses reproduce the original to float
    precision. See ``models/MPL/CONVERSION.md``.
    """
    human = _build_human("right", "passive")
    _freeze_legs_standing(human)
    _align_bionic_arm(human)
    feet_low = _lowest_geom_z(human)  # human-only (no scene yet) -> the feet
    frame = human.worldbody.add_frame()
    frame.pos = [0.0, 0.0, 0.0]
    frame.quat = [1.0, 0.0, 0.0, 0.0]
    human.attach(_bionic_scene_spec(), prefix="", suffix="", frame=frame)
    _ground_bionic(human, feet_low)
    # The original enables multiccd (multi-point convex contacts) so the box rests stably
    # on a pillar; MjSpec.attach drops the scene's <flag>, so re-assert it on the composite.
    human.option.enableflags |= int(mujoco.mjtEnableBit.mjENBL_MULTICCD)
    human.option.timestep = 0.002
    _add_bionic_keyframes(human)
    return human


def build_bionic_bimanual():
    """Compile the bionic-bimanual env. Returns ``(MjModel, MjData)``."""
    model = build_bionic_bimanual_spec().compile()
    return model, mujoco.MjData(model)


def build_auxivo_liftsuit_spec() -> "mujoco.MjSpec":
    """Compose the Auxivo Liftsuit env; return the uncompiled ``MjSpec``.

    The muscled myo_sim ``myotorso`` (spine joints + trunk muscles) is the base; the
    back-exosuit hardware fragment (``models/AuxivoLiftsuit/auxivo_liftsuit.xml``) is
    attached at the original exo->trunk pose via a rigid map from the authoring torso
    pose to this build's torso pose, then coupled with the two original body welds. No
    anatomical assets are housed here -- the human comes from the myo_sim import; only
    the three exosuit meshes live under ``models/AuxivoLiftsuit/mesh``. See CONVERSION.md.
    """
    import myo_sim

    human = myo_sim.build_spec("myotorso")
    _strip_scene_decor(human)  # model-only env; also lets it export cleanly

    probe = human.compile()  # read this build's torso world pose to place the suit
    pd = mujoco.MjData(probe)
    mujoco.mj_forward(probe, pd)
    tid = mujoco.mj_name2id(probe, mujoco.mjtObj.mjOBJ_BODY, "torso")
    p1 = np.asarray(pd.xpos[tid])
    q1 = np.asarray(pd.xquat[tid])

    # rigid map taking the authoring torso pose (p0, q0) to this build's torso pose:
    p0 = np.asarray(_AUXIVO_AUTHOR_TORSO_POS)
    q0i = np.zeros(4)
    mujoco.mju_negQuat(q0i, np.asarray(_AUXIVO_AUTHOR_TORSO_QUAT))
    map_quat = np.zeros(4)
    mujoco.mju_mulQuat(map_quat, q1, q0i)  # R = q1 * conj(q0)
    rp0 = np.zeros(3)
    mujoco.mju_rotVecQuat(rp0, p0, map_quat)
    map_pos = p1 - rp0  # t = p1 - R * p0

    with open(_AUXIVO_XML) as f:
        exo = mujoco.MjSpec.from_string(f.read().replace("__EXO__", _AUXIVO_MESH.replace("\\", "/")))
    frame = human.worldbody.add_frame()
    frame.pos = map_pos.tolist()
    frame.quat = map_quat.tolist()
    human.attach(exo, prefix="", suffix="", frame=frame)

    for body1, body2, anchor in _AUXIVO_WELDS:
        eq = human.add_equality()
        eq.type = mujoco.mjtEq.mjEQ_WELD
        eq.objtype = mujoco.mjtObj.mjOBJ_BODY
        eq.name1 = body1
        eq.name2 = body2
        eq.data[:3] = anchor  # relpose is left to auto-solve from the placed rest pose
    return human


def build_auxivo_liftsuit():
    """Compile the Auxivo Liftsuit env. Returns ``(MjModel, MjData)``."""
    model = build_auxivo_liftsuit_spec().compile()
    return model, mujoco.MjData(model)


def export_upper_body_xml(spec: "mujoco.MjSpec", output_path: str) -> None:
    """Serialize a composed upper-body ``MjSpec`` to a clean, standalone XML.

    A composed model (myo_sim human + attached device) must NOT be serialized with a
    raw ``spec.to_xml()``: the attached fragments' unnamed ``main`` defaults collapse
    into anonymous ``<default>`` blocks and the myo_sim asset dirs are stripped, so the
    output fails to reload ("empty class name" / "Error opening file"). Route it through
    ``utils.export_combined_xml`` (the same path the lower-limb devices use), which
    hoists/names those defaults and rewrites mesh paths absolute to the output. Use with
    the ``build_*_spec`` builders, e.g.::

        export_upper_body_xml(build_auxivo_liftsuit_spec(), "auxivo_liftsuit.xml")

    Writes a model-only XML (no scene/lighting); a downstream scene or terrain supplies
    those. The reloaded model reproduces the live build to float round-trip precision.
    """
    from .utils import export_combined_xml

    spec.compile()  # ensure meshdir/asset state is resolved before serializing
    models = _files("assist_sim").joinpath("models")
    # Candidate (modelfiledir, meshdir) pairs, tried in order until each mesh resolves:
    # this build's own dirs, the myo_sim anatomical meshes, then each device mesh dir.
    mesh_dirs = [
        (Path(spec.modelfiledir or "."), getattr(spec, "meshdir", "") or ""),
        (Path(str(_files("myo_sim").joinpath("models"))), ""),
    ]
    for name in ("Wheelchair", "AuxivoLiftsuit", "MPL"):
        for sub in ("mesh", "meshes"):
            mesh_dirs.append((Path(str(models.joinpath(name))), sub))
    export_combined_xml(spec, output_path, mesh_dirs=mesh_dirs)
