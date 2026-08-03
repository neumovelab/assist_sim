"""Smoke test for the myolegs26 -> myolegs22 planar reduction.

Builds ``myolegs22`` through the registry (``build_spec("myolegs26")`` ->
:func:`assist_sim.reduce_legs.reduce_myolegs26_to_22` -> scene strip) and pins
the reduced model against the reference ``myoLeg22_2D_myolegs26_rigid`` shape:
the compiled signature, the exact joint / actuator name lists, and that all five
keyframes are in range.  Requires myo_sim (skipped otherwise).
"""

from __future__ import annotations

import mujoco as mj
import pytest

from assist_sim import load_combined
from assist_sim import registry

from .conftest import needs_myo_sim

# Frozen targets from the reference planar-22 model.
_SIGNATURE = dict(nq=39, nu=22, njnt=39, nkey=5, neq=28, nbody=38)

_JOINTS = [
    "pelvis_tx",
    "pelvis_ty",
    "pelvis_tilt",
    "hip_flexion_r",
    "knee_r_translation1",
    "knee_r_translation2",
    "knee_angle_r",
    "ankle_angle_r",
    "mtp_angle_r",
    "hamstrings_r_semimem_r-P2_x",
    "hamstrings_r_semimem_r-P2_y",
    "rect_fem_r_rect_fem_r-P3_x",
    "rect_fem_r_rect_fem_r-P3_y",
    "vasti_r_vas_int_r-P4_x",
    "vasti_r_vas_int_r-P4_y",
    "gastroc_r_med_gas_r-P2_x",
    "gastroc_r_med_gas_r-P2_y",
    "gastroc_r_med_gas_r-P2_z",
    "hip_flexion_l",
    "knee_l_translation1",
    "knee_l_translation2",
    "knee_angle_l",
    "ankle_angle_l",
    "mtp_angle_l",
    "hamstrings_l_semimem_l-P2_x",
    "hamstrings_l_semimem_l-P2_y",
    "rect_fem_l_rect_fem_l-P3_x",
    "rect_fem_l_rect_fem_l-P3_y",
    "vasti_l_vas_int_l-P4_x",
    "vasti_l_vas_int_l-P4_y",
    "gastroc_l_med_gas_l-P2_x",
    "gastroc_l_med_gas_l-P2_y",
    "gastroc_l_med_gas_l-P2_z",
    "iliopsoas_r_psoas_r-P3_x",
    "iliopsoas_r_psoas_r-P3_y",
    "iliopsoas_r_psoas_r-P3_z",
    "iliopsoas_l_psoas_l-P3_x",
    "iliopsoas_l_psoas_l-P3_y",
    "iliopsoas_l_psoas_l-P3_z",
]

_ACTUATORS = [
    "hamstrings_r",
    "bifemsh_r",
    "edl_r",
    "fdl_r",
    "glutmax_r",
    "iliopsoas_r",
    "rectfem_r",
    "vasti_r",
    "gastroc_r",
    "soleus_r",
    "tibant_r",
    "hamstrings_l",
    "bifemsh_l",
    "edl_l",
    "fdl_l",
    "glutmax_l",
    "iliopsoas_l",
    "rectfem_l",
    "vasti_l",
    "gastroc_l",
    "soleus_l",
    "tibant_l",
]

_KEYFRAMES = ["stand", "walk_left", "walk_right", "squat", "lunge"]


@pytest.fixture(scope="module")
def reduced():
    """Compiled bare myolegs22 (registry-resolved, scene-stripped)."""
    spec = registry._resolve_msk("myolegs22")
    return spec.compile()


@needs_myo_sim
def test_signature(reduced):
    actual = {k: getattr(reduced, k) for k in _SIGNATURE}
    assert actual == _SIGNATURE


@needs_myo_sim
def test_joint_names_match_reference(reduced):
    names = [mj.mj_id2name(reduced, mj.mjtObj.mjOBJ_JOINT, i) for i in range(reduced.njnt)]
    assert names == _JOINTS


@needs_myo_sim
def test_actuator_names_match_reference(reduced):
    names = [mj.mj_id2name(reduced, mj.mjtObj.mjOBJ_ACTUATOR, i) for i in range(reduced.nu)]
    assert names == _ACTUATORS


@needs_myo_sim
def test_keyframes_present_and_in_range(reduced):
    names = [mj.mj_id2name(reduced, mj.mjtObj.mjOBJ_KEY, i) for i in range(reduced.nkey)]
    assert names == _KEYFRAMES

    data = mj.MjData(reduced)
    for ki in range(reduced.nkey):
        mj.mj_resetDataKeyframe(reduced, data, ki)
        out_of_range = 0
        for ji in range(reduced.njnt):
            if not reduced.jnt_limited[ji]:
                continue
            q = data.qpos[reduced.jnt_qposadr[ji]]
            lo, hi = reduced.jnt_range[ji]
            if q < lo - 1e-9 or q > hi + 1e-9:
                out_of_range += 1
        assert out_of_range == 0, (names[ki], out_of_range)


@needs_myo_sim
def test_edl_fdl_corrected_fmax(reduced):
    for name, fmax in (("edl_r", 553.241), ("fdl_r", 332.13), ("edl_l", 553.241), ("fdl_l", 332.13)):
        aid = mj.mj_name2id(reduced, mj.mjtObj.mjOBJ_ACTUATOR, name)
        assert reduced.actuator_gainprm[aid, 2] == pytest.approx(fmax)
        assert reduced.actuator_biasprm[aid, 2] == pytest.approx(fmax)


@needs_myo_sim
def test_load_combined_with_device_steps():
    """The reduced MSK combines with a compatible device and steps stably."""
    model, data = load_combined("myolegs22", "DephyExoBoot_L1")
    mj.mj_forward(model, data)
    for _ in range(5):
        mj.mj_step(model, data)
