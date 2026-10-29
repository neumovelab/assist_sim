"""Every keyframe closes the device equality constraints, with the device joints in their ranges.

A device equality is a ``connect``, ``weld`` or ``joint`` constraint with an object on a device
body. If one is open at a keyframe, a reset to that keyframe starts with a constraint violation,
and MuJoCo pulls the device shut on the first steps with large transient forces. The keyframe
rebuild therefore moves the device joints of each keyframe, with the MSK joints held, until the
device equalities close. The fixture tests check the solve itself. The MSK tests check every
shipped device that has equalities, on every MSK.
"""

from __future__ import annotations

import warnings
from functools import lru_cache
from pathlib import Path

import mujoco as mj
import numpy as np
import pytest

from assist_sim import load_combined
from assist_sim.combine import ModelCombiner
from assist_sim.config import DeviceConfig
from assist_sim.registry import resolve_device_config

from .conftest import FIXTURES, needs_myo_sim
from .test_smoke_combinations import EXPECTED

MAX_RESIDUAL = 1e-4  # m (connect, weld) or rad (joint)
RANGE_TOL = 1e-9

# Devices whose keyframes do not close. Strict, so the test fails once a device closes and its
# entry must go.
KNOWN_OPEN = {
    "UTAnkleExo_L2": (
        "A-S4: the UTAnkleExo_L2 connect anchors do not match the leg on myolegs22, myolegs26 and "
        "myofullbody. The least-squares closure leaves the connects open by up to 177 mm (myolegs22), "
        "40 mm (myolegs26), 16 mm (myofullbody) and 2.3 mm (myolegs)."
    ),
}


@lru_cache(maxsize=None)
def _has_equalities(msk_key: str, device_key: str) -> bool:
    config = DeviceConfig.from_yaml(str(resolve_device_config(msk_key, device_key)))
    return bool(config.resolve_equalities(msk_key))


def _params():
    for msk_key, device_key in EXPECTED:
        if not _has_equalities(msk_key, device_key):
            continue
        reason = KNOWN_OPEN.get(device_key)
        marks = [pytest.mark.xfail(strict=True, reason=reason)] if reason else []
        yield pytest.param(msk_key, device_key, marks=marks, id=f"{msk_key}-{device_key}")


PARAMS = list(_params())


def _device_equalities(model: mj.MjModel, prefix: str) -> list[int]:
    """Equalities with an object whose name has the device prefix (independent of the solver)."""
    ids = []
    for e in range(model.neq):
        names = []
        for obj in (model.eq_obj1id[e], model.eq_obj2id[e]):
            if obj < 0:
                continue
            if model.eq_type[e] == mj.mjtEq.mjEQ_JOINT:
                names.append(model.joint(obj).name)
            elif model.eq_objtype[e] == mj.mjtObj.mjOBJ_SITE:
                names.append(model.site(obj).name)
            else:
                names.append(model.body(obj).name)
        if any(n.startswith(prefix) for n in names):
            ids.append(e)
    return ids


def test_closure_matrix_covers_the_loop_devices():
    devices = {p.values[1] for p in PARAMS}
    assert {"STRIDE_L2", "Hippo_L1", "UTAnkleExo_L2"} <= devices


@needs_myo_sim
@pytest.mark.parametrize(("msk_key", "device_key"), PARAMS)
def test_keyframes_close_device_equalities(msk_key, device_key):
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        model, data = load_combined(msk_key, device_key)
    prefix = device_key + "_"
    eq_ids = _device_equalities(model, prefix)
    assert eq_ids, f"{msk_key} x {device_key}: no device equality found"
    joints = [
        j
        for j in range(model.njnt)
        if model.body(model.jnt_bodyid[j]).name.startswith(prefix)
        and model.jnt_limited[j]
        and model.jnt_type[j] in (mj.mjtJoint.mjJNT_HINGE, mj.mjtJoint.mjJNT_SLIDE)
    ]
    assert model.nkey > 0

    for k in range(model.nkey):
        key = model.key(k).name
        mj.mj_resetDataKeyframe(model, data, k)
        mj.mj_forward(model, data)
        rows = data.efc_type[: data.nefc] == mj.mjtConstraint.mjCNSTR_EQUALITY
        for e in eq_ids:
            residual = float(np.linalg.norm(data.efc_pos[: data.nefc][rows & (data.efc_id[: data.nefc] == e)]))
            assert residual < MAX_RESIDUAL, (
                f"{msk_key} x {device_key}, keyframe '{key}': equality {model.eq(e).name or e} is open by "
                f"{residual:.3g} m or rad."
            )
        for j in joints:
            q = float(data.qpos[model.jnt_qposadr[j]])
            lo, hi = model.jnt_range[j]
            assert lo - RANGE_TOL <= q <= hi + RANGE_TOL, (
                f"{msk_key} x {device_key}, keyframe '{key}': {model.joint(j).name} = {q:.4f} is outside [{lo:.4f}, {hi:.4f}]."
            )

    open_warnings = [str(w.message) for w in caught if "do not close" in str(w.message)]
    assert not open_warnings, open_warnings


# ----------------------------------------------------------------------
# The solve on the minimal fixture (no myo_sim)
# ----------------------------------------------------------------------

# One device hinge coupled to the MSK hip with a quadratic. In "home", hip = 0.1.
LOOP = """
device:
  name: "Loop"
  model_xml: "minimal_device.xml"
attachments:
  - device_body: "dev_a"
    parent_body: "pelvis"
  - device_body: "dev_b"
    parent_body: "thigh"
equality:
  - type: joint
    joint1: "dev_joint"
    joint2: "hip"
    polycoef: [0.05, 2, 0.5, 0, 0]
"""
LOOP_CLOSED = 0.05 + 2 * 0.1 + 0.5 * 0.1**2

# The authored "home" keyframe of the minimal human. Its own pp_x = knee equality is open there.
HOME = {"pelvis_ty": 0.9, "hip": 0.1, "knee": 0.2, "ankle": 0.0, "mtp": 0.0, "pp_x": 0.0, "pp_y": 0.01}


def _combine(tmp_path: Path, body: str) -> mj.MjModel:
    (tmp_path / "minimal_device.xml").write_bytes((FIXTURES / "minimal_device.xml").read_bytes())
    path = tmp_path / "L1config.yaml"
    path.write_text(body, encoding="utf-8")
    spec = mj.MjSpec.from_file(str(FIXTURES / "minimal_human.xml"))
    model, _ = ModelCombiner().combine(spec, DeviceConfig.from_yaml(str(path)))
    return model


def _home(model: mj.MjModel, joint: str) -> float:
    return float(model.key_qpos[model.key("home").id][model.jnt_qposadr[model.joint(joint).id]])


def test_device_joint_closes_its_equality_and_msk_joints_hold(tmp_path):
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        model = _combine(tmp_path, LOOP)
    assert _home(model, "Loop_dev_joint") == pytest.approx(LOOP_CLOSED, abs=1e-9)
    # The MSK joints keep their authored values exactly, and the open MSK equality stays open.
    assert {joint: _home(model, joint) for joint in HOME} == HOME


def test_device_joint_stays_in_range_and_warns_when_open(tmp_path):
    body = LOOP + 'joint_overrides:\n  - {name: "Loop_dev_joint", range: [-0.1, 0.1]}\n'
    with pytest.warns(RuntimeWarning, match=r"Loop on this MSK: .* keyframe 'home'"):
        model = _combine(tmp_path, body)
    assert _home(model, "Loop_dev_joint") == pytest.approx(0.1)


def test_device_without_equalities_is_not_moved(minimal_human, minimal_device_config):
    config = DeviceConfig.from_yaml(minimal_device_config)
    model, _ = ModelCombiner().combine(mj.MjSpec.from_file(minimal_human), config)
    jid = model.joint("TestDev_dev_joint").id
    assert _home(model, "TestDev_dev_joint") == model.qpos0[model.jnt_qposadr[jid]]
    assert not ModelCombiner._device_closure_targets(model, "TestDev_")[2].size
