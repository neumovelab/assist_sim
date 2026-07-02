"""Test that a composed MSK's authored keyframe values survive the in-memory
pipeline (decompose-by-name before surgery, rebuild after the final compile)."""

from __future__ import annotations

import mujoco as mj
import pytest

from assist_sim import load_combined

from .conftest import needs_myo_sim


@needs_myo_sim
def test_combined_keyframes_preserve_source_values():
    """End-to-end: a combined model's keyframes carry the MSK's authored joint
    values by name. Regression for the bug where only overridden joints survived
    and everything else was zeroed.

    The composed spec's ``stand`` keyframe is decomposed by joint name before
    surgery (which changes nq/nv) and rebuilt after the final compile.  myolegs26
    is a myosuite-convention model with a free ``root`` joint (not a ``pelvis_ty``
    slide), so DephyExoBoot's ``pelvis_ty`` override is a no-op here (the
    free-root height comes from the base stand keyframe)."""
    model, _ = load_combined("myolegs26", "DephyExoBoot_L1")
    kf = mj.mj_name2id(model, mj.mjtObj.mjOBJ_KEY, "stand")
    assert kf >= 0
    qpos = list(model.key_qpos[kf])

    # Free-root base, no pelvis_ty (the DephyExoBoot pelvis_ty override no-ops).
    assert mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "root") >= 0
    assert mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pelvis_ty") < 0

    # Non-trivial authored joint values from the myolegs26 stand keyframe are
    # preserved through surgery + attach + recompile.
    expected = {
        "knee_r_translation1": -0.003639,
        "knee_r_translation2": -0.395,
        "ankle_angle_r": -0.0143,
    }
    for name, value in expected.items():
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
        assert jid >= 0, f"joint {name} missing from compiled model"
        assert qpos[int(model.jnt_qposadr[jid])] == pytest.approx(value, abs=1e-6), f"keyframe lost authored value for {name}"
