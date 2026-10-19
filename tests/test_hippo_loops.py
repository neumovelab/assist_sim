"""Hippo_L1 keyframes close the cuff loops and respect the hip ranges the device sets.

Each Hippo thigh segment hangs from its motor on a hinge, and a ``connect`` equality holds its cuff to
a strap that slides along the thigh. The hinge and strap values that close each loop in the canonical
poses are solved offline and stored as per-MSK keyframe overrides, so they go stale when the canonical
poses or the MSK geometry change. A stale keyframe starts with a loop open, and the solver snaps it
shut on the first step. The device also restricts hip adduction and rotation with explicit joint
ranges, which every keyframe must respect.
"""

from __future__ import annotations

import mujoco as mj
import numpy as np
import pytest

from assist_sim import load_combined

from .conftest import needs_myo_sim

MSKS = ("myolegs22", "myolegs26", "myolegs", "myofullbody")
PREFIX = "Hippo_L1_"
MAX_LOOP_GAP_M = 1e-4


def _cuff_connects(model: mj.MjModel) -> list[int]:
    return [
        i
        for i in range(model.neq)
        if model.eq_type[i] == mj.mjtEq.mjEQ_CONNECT and model.body(model.eq_obj1id[i]).name.startswith(PREFIX)
    ]


@needs_myo_sim
@pytest.mark.parametrize("msk_key", MSKS)
def test_hippo_cuff_loops_close_at_every_keyframe(msk_key):
    model, data = load_combined(msk_key, "Hippo_L1")
    connects = _cuff_connects(model)
    assert len(connects) == 2
    for k in range(model.nkey):
        mj.mj_resetDataKeyframe(model, data, k)
        mj.mj_forward(model, data)
        for i in connects:
            b1, b2 = model.eq_obj1id[i], model.eq_obj2id[i]
            p1 = data.xpos[b1] + data.xmat[b1].reshape(3, 3) @ model.eq_data[i][0:3]
            p2 = data.xpos[b2] + data.xmat[b2].reshape(3, 3) @ model.eq_data[i][3:6]
            gap = float(np.linalg.norm(p1 - p2))
            assert gap < MAX_LOOP_GAP_M, (
                f"{msk_key} keyframe '{model.key(k).name}': the {model.body(b1).name} loop is open by "
                f"{gap * 1000:.2f} mm. Re-solve the Hippo keyframe_overrides for this MSK."
            )


@needs_myo_sim
@pytest.mark.parametrize("msk_key", MSKS)
def test_hippo_keyframes_respect_hip_and_device_ranges(msk_key):
    model, _ = load_combined(msk_key, "Hippo_L1")
    names = [
        n
        for n in ("hip_adduction_r", "hip_adduction_l", "hip_rotation_r", "hip_rotation_l")
        if mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n) >= 0
    ] + [model.joint(j).name for j in range(model.njnt) if model.joint(j).name.startswith(PREFIX)]
    assert len(names) >= 4
    for name in names:
        j = model.joint(name).id
        lo, hi = model.jnt_range[j]
        for k in range(model.nkey):
            q = float(model.key_qpos[k][model.jnt_qposadr[j]])
            assert lo - 1e-9 <= q <= hi + 1e-9, (
                f"{msk_key} keyframe '{model.key(k).name}': {name} = {q:.4f} is outside [{lo:.4f}, {hi:.4f}]."
            )
