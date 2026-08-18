"""Derive the planar 22-muscle leg model (``myolegs22``) from ``myolegs26``.

:func:`reduce_myolegs26_to_22` takes a freshly-composed ``myolegs26`` ``MjSpec``
(the 26-muscle, passive-torso leg model that ``myo_sim.load_spec`` returns) and
transforms it *in place* into the planar 22-muscle ``myolegs22`` -- the same
model shape the reference documents.  It is a
joint / actuator / site transform: the 38-body tree is left untouched.

The transform (each step mirrors the reference model's recipe header):

1. **Face +x / up +z.**  Yaw the ``sacrum`` (torso root) and ``pelvis`` (leg
   root) sibling bodies to ``0.7071068 0.7071068 0 0`` and drop the pelvis base
   to the ground plane, so the planar vertical slide reads as absolute height.
2. **Planar root.**  Delete the 6-DOF ``root`` freejoint and add three
   single-DOF joints on the same body: ``pelvis_tx`` (fore-aft slide),
   ``pelvis_ty`` (vertical slide) and ``pelvis_tilt`` (sagittal hinge).
3. **Drop frontal-plane hip DOF.**  Delete ``hip_adduction_{r,l}`` and
   ``hip_rotation_{r,l}``, leaving ``hip_flexion`` as the sole hip DOF per side.
4. **26 -> 22 muscles.**  Delete the ``abd``/``add`` actuators and their spatial
   tendons.
5. **Strip orphaned muscle sites.**  Remove every site no longer referenced by a
   surviving tendon or sensor -- the abd/add via-points *and* the passive torso's
   dangling muscle via-points that ``myolegs26`` carries but never actuates.
6. **Widen via-point joint ranges + inject keyframes.**  Apply the reference's
   joint ranges (the muscle-routing / knee-translation joints are widened so the
   flexed keys stay in range) and the five ``stand`` / ``walk_left`` /
   ``walk_right`` / ``squat`` / ``lunge`` keyframes.

The via-point coupler equalities (``neq`` == 28) carry over unchanged.

Provenance: the widened joint ranges, EDL/FDL Fmax and keyframe qpos/qvel arrays
are sourced verbatim.  Because this transform reproduces that
model's joint order exactly, the keyframe arrays transfer by position.

The module is deliberately ``myo_sim``-agnostic (it only touches the ``MjSpec``
it is handed) so it can later lift into ``myo_sim`` unchanged.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Dict, List, Tuple

import numpy as np

if TYPE_CHECKING:
    import mujoco

# Face +x / up +z: 90-degree yaw applied to the torso + leg root sibling bodies.
_YAW_QUAT = [0.7071068, 0.7071068, 0.0, 0.0]

# 26 -> 22: the hip abductor / adductor actuators (and their ``*_tendon``).
_ABDADD = ("abd_r", "add_r", "abd_l", "add_l")

# Frontal-plane hip DOF removed for the sagittal-plane (planar) model.
_FRONTAL_HIP_JOINTS = ("hip_adduction_r", "hip_rotation_r", "hip_adduction_l", "hip_rotation_l")

# Corrected extensor/flexor-digitorum-longus peak isometric force (gainprm[2] /
# biasprm[2]).  Sourced from the reference model.
_EDL_FDL_FMAX = {"edl_r": 553.241, "edl_l": 553.241, "fdl_r": 332.13, "fdl_l": 332.13}

# Non-planar joint ranges, sourced verbatim from the reference model.  The
# muscle-routing (via-point) and knee-translation joints are *widened* relative
# to ``myolegs26`` so the flexed ``squat`` / ``lunge`` keys stay in range; the
# anatomical hip/knee/ankle/mtp ranges are identical to the source and listed
# for a complete, reference-anchored table (setting them is idempotent).
_JOINT_RANGES: Dict[str, Tuple[float, float]] = {
    "hip_flexion_r": (-0.349, 2.356),
    "knee_r_translation1": (-0.015789, 0.00812921),
    "knee_r_translation2": (-0.0321026, 0.0001),
    "knee_angle_r": (-2.531, 0.0),
    "ankle_angle_r": (-1.134, 0.349),
    "mtp_angle_r": (-0.523599, 0.523599),
    "hamstrings_r_semimem_r-P2_x": (-0.0001, 0.0081116),
    "hamstrings_r_semimem_r-P2_y": (-0.0129588, 0.0001),
    "rect_fem_r_rect_fem_r-P3_x": (-0.0795383, 0.0001),
    "rect_fem_r_rect_fem_r-P3_y": (-0.0001, 0.00515498),
    "vasti_r_vas_int_r-P4_x": (-0.0824418, 0.0001),
    "vasti_r_vas_int_r-P4_y": (-0.00353384, 0.00267964),
    "gastroc_r_med_gas_r-P2_x": (-0.00522854, 0.00295527),
    "gastroc_r_med_gas_r-P2_y": (-0.00443496, 0.00252012),
    "gastroc_r_med_gas_r-P2_z": (-0.00117145, 0.000700305),
    "hip_flexion_l": (-0.349, 2.356),
    "knee_l_translation1": (-0.015789, 0.00812921),
    "knee_l_translation2": (-0.0321026, 0.0001),
    "knee_angle_l": (-2.531, 0.0),
    "ankle_angle_l": (-1.134, 0.349),
    "mtp_angle_l": (-0.523599, 0.523599),
    "hamstrings_l_semimem_l-P2_x": (-0.0001, 0.0081116),
    "hamstrings_l_semimem_l-P2_y": (-0.0129588, 0.0001),
    "rect_fem_l_rect_fem_l-P3_x": (-0.0795383, 0.0001),
    "rect_fem_l_rect_fem_l-P3_y": (-0.0001, 0.00515498),
    "vasti_l_vas_int_l-P4_x": (-0.0824418, 0.0001),
    "vasti_l_vas_int_l-P4_y": (-0.00353384, 0.00267964),
    "gastroc_l_med_gas_l-P2_x": (-0.00522854, 0.00295527),
    "gastroc_l_med_gas_l-P2_y": (-0.00443496, 0.00252012),
    "gastroc_l_med_gas_l-P2_z": (-0.000700305, 0.00117145),
    "iliopsoas_r_psoas_r-P3_x": (-0.000579356, 0.00574366),
    "iliopsoas_r_psoas_r-P3_y": (-0.00235276, 0.0266106),
    "iliopsoas_r_psoas_r-P3_z": (-0.00653183, 0.000646401),
    "iliopsoas_l_psoas_l-P3_x": (-0.000579356, 0.00574366),
    "iliopsoas_l_psoas_l-P3_y": (-0.00235276, 0.0266106),
    "iliopsoas_l_psoas_l-P3_z": (-0.000646401, 0.00653183),
}

# Five keyframes (name, qpos, qvel), sourced verbatim from the reference model.
# The 39-long qpos arrays are ordered to the reduced-model joint order (which
# this transform reproduces exactly), so they transfer by position.
_KEYFRAMES: Tuple[Tuple[str, str, str], ...] = (
    (
        "stand",
        "0 0.91 0 0 0 0 0 0 -0.0143 0 0 0 0 0 0 0 0 0 0 0 0 0 -0.0143 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
        "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
    ),
    (
        "walk_left",
        "0 0.88 -0.262 -0.174 0.00313046 -0.00258514 -0.436 0 0 0.00244588 -0.00392612 -0.0156618 "
        "0.00359686 -0.0167642 0.00177277 -0.00268494 -0.00226985 -0.000561181 0.436 0.000567571 "
        "-0.000340948 -0.0873 -0.0737 0 0.000459471 -0.000737542 -0.00357334 0.000947603 -0.00386577 "
        "0.000503933 -0.000582272 -0.000492261 0.000121708 -0.000274088 -0.00128806 0.000312424 "
        "0.000973706 0.00457566 0.00110991",
        "1.5 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
    ),
    (
        "walk_right",
        "0 0.88 -0.262 0.436 0.000567571 -0.000340948 -0.0873 0 -0.0737 0.000459471 -0.000737542 "
        "-0.00357334 0.000947603 -0.00386577 0.000503933 -0.000582272 -0.000492261 -0.000121708 -0.174 "
        "0.00313046 -0.00258514 -0.436 0 0 0.00244588 -0.00392612 -0.0156618 0.00359686 -0.0167642 "
        "0.00177277 -0.00268494 -0.00226985 0.000561181 0.000973706 0.00457566 -0.00110991 -0.000274088 "
        "-0.00128806 -0.000312424",
        "1.5 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
    ),
    (
        "squat",
        "0 0.735 -0.611 1.309 0.00801825 -0.013877 -1.309 0 0.349 0.00678797 -0.0108957 -0.0307363 "
        "0.00489804 -0.0319356 0.00257792 -0.00511918 -0.00432696 -0.00106944 1.309 0.00801825 -0.013877 "
        "-1.309 0.349 0.125 0.00678797 -0.0108957 -0.0307363 0.00489804 -0.0319356 0.00257792 -0.00511918 "
        "-0.00432696 0.00106944 0.0036846 0.0173131 -0.00419992 0.0036846 0.0173131 0.00419992",
        "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
    ),
    (
        "lunge",
        "0 0.67 -0.558 0.698 0.00750741 -0.0181029 -1.56 0 0.349 0.00751197 -0.0120576 -0.0336899 "
        "0.00449775 -0.0347267 0.00244804 -0.00473821 -0.00400428 -0.000989454 1.57 0.00789372 -0.0124705 "
        "-1.222 0.174 0.06 0.00646838 -0.0103828 -0.0298205 0.00498543 -0.03108 0.00257653 -0.00512188 "
        "-0.0043294 0.0010701 0.00172256 0.00809451 -0.00196352 0.00447291 0.0210163 0.00509836",
        "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
    ),
)


def reduce_myolegs26_to_22(spec: "mujoco.MjSpec") -> "mujoco.MjSpec":
    """Reduce a ``myolegs26`` ``MjSpec`` to the planar 22-muscle ``myolegs22``, in place.

    Mutates and returns *spec*.  The result compiles to ``nq=39, nu=22, njnt=39,
    nkey=5, neq=28, nbody=38`` with joint / actuator name lists matching the
    reference ``myoLeg22_2D_myolegs26_rigid``.  The bundled myosuite scene (if
    still attached) is left for the caller's scene strip.

    Raises:
        ValueError: if *spec* is missing an element the transform expects (i.e.
            it is not a ``myolegs26``-shaped spec).
    """
    import mujoco

    # Which sites survive?  Compute from a *copy* -- compiling the working spec and then
    # editing it corrupts subsequent add/delete, so the model we mutate is never compiled
    # mid-transform.  This is load-bearing, not defensive: measured on mujoco 3.4 and 3.11,
    # running this very transform on a pre-compiled spec drops ``pelvis_ty`` (one of the
    # three planar-root joints it adds) and leaves ``hip_adduction_r`` in place (one of the
    # four frontal-plane joints it deletes).  Both counts still come out at 39, so the
    # result compiles and looks right while being a different model: no vertical slide, and
    # a frontal hip DOF the planar model must not have.  ``combine._decompose_keyframes``
    # probes a copy for the same reason.
    keep_sites = _surviving_site_names(spec.copy(), mujoco)

    # 1. Face +x / up +z: yaw the torso + leg root siblings, drop pelvis to ground.
    _set_body_quat(_require(spec.body("sacrum"), "body", "sacrum"), _YAW_QUAT, mujoco)
    _set_body_quat(_require(spec.body("pelvis"), "body", "pelvis"), _YAW_QUAT, mujoco)
    full_body = _require(spec.body("Full Body"), "body", "Full Body")
    full_body.pos = [0.0, 0.0, 0.0]

    # 2. Planar root: free root -> pelvis_tx / pelvis_ty / pelvis_tilt.
    spec.delete(_require(spec.joint("root"), "joint", "root"))
    _add_planar_root(full_body, mujoco)

    # 3. Drop frontal-plane hip DOF.
    for jname in _FRONTAL_HIP_JOINTS:
        spec.delete(_require(spec.joint(jname), "joint", jname))

    # 4. 26 -> 22 muscles: delete abd/add actuators + their spatial tendons.
    for name in _ABDADD:
        spec.delete(_require(spec.actuator(name), "actuator", name))
        tname = f"{name}_tendon"
        spec.delete(_require(spec.tendon(tname), "tendon", tname))

    # 5. Strip orphaned muscle sites (unreferenced by any surviving tendon/sensor).
    for site in list(spec.sites):
        if site.name not in keep_sites:
            spec.delete(site)

    # 6. EDL / FDL corrected peak force.
    for name, fmax in _EDL_FDL_FMAX.items():
        act = _require(spec.actuator(name), "actuator", name)
        act.gainprm = _with_index(act.gainprm, 2, fmax)
        act.biasprm = _with_index(act.biasprm, 2, fmax)

    # 7. Widen via-point joint ranges (from the reference).
    for jname, (lo, hi) in _JOINT_RANGES.items():
        _require(spec.joint(jname), "joint", jname).range = [lo, hi]

    # 8. Inject the five keyframes (from the reference).
    for name, qpos, qvel in _KEYFRAMES:
        key = spec.add_key()
        key.name = name
        key.qpos = np.fromstring(qpos, sep=" ")
        key.qvel = np.fromstring(qvel, sep=" ")

    return spec


# ----------------------------------------------------------------------
# Internal helpers
# ----------------------------------------------------------------------


def _surviving_site_names(probe: "mujoco.MjSpec", mujoco) -> set:
    """Names of sites referenced by a non-abd/add tendon or any sensor.

    Compiles *probe* (a throwaway copy) and reads the tendon-wrap + sensor site
    references directly from the model, so the keep-set is derived structurally
    rather than by name heuristics.  The abd/add tendons are excluded because
    they are about to be deleted, which is what orphans their via-point sites.
    """
    model = probe.compile()
    abdadd_tendons = {f"{name}_tendon" for name in _ABDADD}
    keep: set = set()

    for tid in range(model.ntendon):
        tname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TENDON, tid)
        if tname in abdadd_tendons:
            continue
        adr, num = model.tendon_adr[tid], model.tendon_num[tid]
        for w in range(adr, adr + num):
            if model.wrap_type[w] == mujoco.mjtWrap.mjWRAP_SITE:
                keep.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, model.wrap_objid[w]))

    for sid in range(model.nsensor):
        if model.sensor_objtype[sid] == mujoco.mjtObj.mjOBJ_SITE:
            keep.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, model.sensor_objid[sid]))

    return keep


def _set_body_quat(body: "mujoco.MjsBody", quat, mujoco) -> None:
    """Set *body*'s orientation to the absolute *quat*.

    ``MjsBody`` can carry an alternative orientation (``alt`` -- euler / xyaxes /
    zaxis / axisangle) that the compiler uses *in place of* ``quat`` when its
    ``type`` isn't ``mjORIENTATION_QUAT``.  ``sacrum`` ships an euler ``alt``, so
    assigning ``quat`` alone is silently ignored (the euler wins) and the torso
    seats on the pelvis at the wrong angle.  Reset ``alt`` to the quat form first
    so ``quat`` is authoritative, then assign it.
    """
    body.alt.type = mujoco.mjtOrientation.mjORIENTATION_QUAT
    body.quat = quat


def _add_planar_root(body: "mujoco.MjsBody", mujoco) -> None:
    """Add ``pelvis_tx`` / ``pelvis_ty`` / ``pelvis_tilt`` to *body*, in order.

    Three unlimited, undamped, zero-armature single-DOF joints replacing the
    freejoint: fore-aft slide (+x), vertical slide (+z), sagittal-plane hinge
    (about -y).  Adding them to the (jointless) root body first makes them the
    leading joints of the compiled model, matching the reference qpos ordering.
    """
    unlimited = mujoco.mjtLimited.mjLIMITED_FALSE
    slide, hinge = mujoco.mjtJoint.mjJNT_SLIDE, mujoco.mjtJoint.mjJNT_HINGE
    common = dict(pos=[0.0, 0.0, 0.0], limited=unlimited, damping=0.0, armature=0.0)
    body.add_joint(name="pelvis_tx", type=slide, axis=[1.0, 0.0, 0.0], **common)
    body.add_joint(name="pelvis_ty", type=slide, axis=[0.0, 0.0, 1.0], **common)
    body.add_joint(name="pelvis_tilt", type=hinge, axis=[0.0, -1.0, 0.0], **common)


def _with_index(arr, index: int, value: float) -> List[float]:
    """Return ``arr`` as a list with ``arr[index]`` replaced by ``value``."""
    out = list(arr)
    out[index] = value
    return out


def _require(element, kind: str, name: str):
    """Return *element*, or raise if the lookup came back empty (wrong source model)."""
    if element is None:
        raise ValueError(
            f"reduce_myolegs26_to_22 expected a myolegs26-shaped spec but found no "
            f"{kind} named {name!r}; got a different model?"
        )
    return element
