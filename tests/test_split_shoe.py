"""The split shoe of ``Anatomics_L1``, ``Humotech_L1`` and ``STRIDE_L2`` is one shoe on every MSK.

The three devices bundle the same split shoe. The aft halves ride ``calcn`` and the fore halves
ride ``toes``, so ``mtp_angle`` sets the shape of the shoe. The fore halves were authored for
the myoLeg MTP axis (``myolegs22``, ``myolegs26``), where the shoe is flat at 0.2. The
80-muscle lineage (``myolegs``, ``myofullbody``) has the oblique gait2392 MTP axis, and its
positive angle lowers the toes. There the halves met at 21.8 deg at ``stand``, and no MTP
angle brought them closer than 6.6 deg. ``Anatomics_L1`` had no MTP override, so its halves
met at 11.4 deg at ``stand`` on all four MSKs.

Two checks per device x MSK, on both feet, for the soles and for the uppers:

- At ``stand`` and at both walk keyframes, the fore half lines up with the aft half within
  ``MAX_KINK_DEG``.
- The MTP range of the device runs from the flat shoe to a toe lift. One end is flat within
  ``MAX_KINK_DEG``, and at the other end the toe of the fore half points up. This check
  catches an MTP range with the wrong sign for the lineage.

Orientations are world-frame body orientations. The shoe meshes share the axes of their body
frames (x forward, y up), so the relative rotation of the two body frames is the kink.
"""

from __future__ import annotations

import mujoco as mj
import numpy as np
import pytest

from assist_sim import load_combined

from .conftest import needs_myo_sim

pytestmark = needs_myo_sim

DEVICES = ("Anatomics_L1", "Humotech_L1", "STRIDE_L2")
MSK_KEYS = ("myolegs22", "myolegs26", "myolegs", "myofullbody")
FLAT_KEYFRAMES = ("stand", "walk_left", "walk_right")
SIDES = ("r", "l")
KINDS = ("sole", "upper")
MAX_KINK_DEG = 1.0


@pytest.fixture(scope="module", params=[(m, d) for d in DEVICES for m in MSK_KEYS], ids=str)
def combined(request):
    msk_key, device_key = request.param
    model, data = load_combined(msk_key, device_key)
    return msk_key, device_key, model, data


def _body(model: mj.MjModel, name: str) -> int:
    body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name)
    assert body >= 0, f"no body {name}"
    return body


def _halves(model: mj.MjModel, device_key: str, side: str, kind: str) -> tuple[int, int]:
    return (
        _body(model, f"{device_key}_{side}_aft_{kind}"),
        _body(model, f"{device_key}_{side}_fore_{kind}"),
    )


def _relative(data: mj.MjData, aft: int, fore: int) -> np.ndarray:
    """The rotation of the fore half in the frame of the aft half."""
    return data.xmat[aft].reshape(3, 3).T @ data.xmat[fore].reshape(3, 3)


def _angle_deg(rotation: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))))


def test_shoe_halves_align_at_flat_keyframes(combined):
    msk_key, device_key, model, data = combined
    bad = []
    for keyframe in FLAT_KEYFRAMES:
        key = mj.mj_name2id(model, mj.mjtObj.mjOBJ_KEY, keyframe)
        assert key >= 0, f"{msk_key} x {device_key} has no '{keyframe}' keyframe"
        mj.mj_resetDataKeyframe(model, data, key)
        mj.mj_kinematics(model, data)
        for side in SIDES:
            for kind in KINDS:
                kink = _angle_deg(_relative(data, *_halves(model, device_key, side, kind)))
                if kink > MAX_KINK_DEG:
                    bad.append(f"{keyframe} {side} {kind}: {kink:.2f} deg")
    assert not bad, f"{msk_key} x {device_key}: the shoe halves meet at an angle ({'; '.join(bad)})"


def test_mtp_range_runs_from_flat_to_toe_lift(combined):
    msk_key, device_key, model, data = combined
    stand = mj.mj_name2id(model, mj.mjtObj.mjOBJ_KEY, "stand")
    for side in SIDES:
        joint = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, f"mtp_angle_{side}")
        assert model.jnt_limited[joint], f"{msk_key} x {device_key}: mtp_angle_{side} has no range"
        aft, fore = _halves(model, device_key, side, "sole")
        kinks, lifts = [], []
        for value in model.jnt_range[joint]:
            mj.mj_resetDataKeyframe(model, data, stand)
            data.qpos[model.jnt_qposadr[joint]] = value
            mj.mj_kinematics(model, data)
            rotation = _relative(data, aft, fore)
            kinks.append(_angle_deg(rotation))
            lifts.append(float(rotation[1, 0]))  # the fore x axis on the aft y (up) axis
        flat = int(np.argmin(kinks))
        assert kinks[flat] <= MAX_KINK_DEG, (
            f"{msk_key} x {device_key}: mtp_angle_{side} range {model.jnt_range[joint]} has no flat end "
            f"(kinks {kinks[0]:.2f} and {kinks[1]:.2f} deg)"
        )
        assert lifts[1 - flat] > 0, (
            f"{msk_key} x {device_key}: the free end of mtp_angle_{side} lowers the toe of the shoe "
            f"(range {model.jnt_range[joint]})"
        )
