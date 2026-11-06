"""UTAnkleExo_L2 sits on the leg: on every MSK, its hinge is on the ankle axis and its foot part is where myolegs has it.

The exo is free-rooted. Its ``connect`` equalities take their leg-side anchors from the
attachment pose at qpos0, so they report a zero residual wherever that pose puts the exo. That
hid a 0.9 m misplacement on ``myolegs22`` and 0.2-0.4 m on ``myolegs26`` and ``myofullbody``
(A-S4). These checks read the pose against the bones instead. ``myolegs`` is the reference,
because the source model fits the exo to it, and the left exo is the mirror image of the right.
The CO build (``planar_root``) turns the skeleton to the ``myolegs22`` frame, and the
world-parented exo must turn with it.
"""

from __future__ import annotations

from functools import lru_cache

import mujoco as mj
import numpy as np
import pytest

from assist_sim import load_combined

from .conftest import needs_myo_sim

pytestmark = needs_myo_sim

DEVICE = "UTAnkleExo_L2"
PREFIX = DEVICE + "_"
MSKS = ("myolegs22", "myolegs26", "myolegs", "myofullbody")
BUILDS = [pytest.param(False, id="free"), pytest.param(True, id="planar_root")]
MAX_RESIDUAL = 1e-4  # m
SIDES = (("r", "dx"), ("l", "sx"))
AXIS_TOL = 1e-4  # m and rad
POS_TOL = 0.001  # m
ANGLE_TOL = np.radians(0.2)
# Sites that the sx geometry mirrors from dx: the device pin and the part1 pivot on part3.
MIRROR_SITES = ("part2_to_part3_encoder_", "part3_to_part1_on_parent_")


def _foot_frame(model, data, side):
    """Ankle joint centre and (forward, left, up) axes: the same bones give the same frame in each lineage."""
    origin = data.xanchor[model.joint("ankle_angle_" + side).id].copy()
    forward = data.xpos[model.body("toes_" + side).id] - data.xpos[model.body("calcn_" + side).id]
    forward[2] = 0.0
    forward /= np.linalg.norm(forward)
    up = np.array([0.0, 0.0, 1.0])
    return origin, np.column_stack([forward, np.cross(up, forward), up])


@lru_cache(maxsize=None)
def _qpos0(msk_key, planar_root=False):
    model, data = load_combined(msk_key, DEVICE, planar_root=planar_root)
    mj.mj_forward(model, data)
    return model, data


def _foot_part_pose(msk_key, side, s, planar_root=False):
    model, data = _qpos0(msk_key, planar_root)
    origin, frame = _foot_frame(model, data, side)
    body = model.body(PREFIX + "part3_" + s).id
    return frame.T @ (data.xpos[body] - origin), frame.T @ data.xmat[body].reshape(3, 3)


@pytest.mark.parametrize("planar_root", BUILDS)
@pytest.mark.parametrize("msk_key", MSKS)
def test_hinge_is_on_the_ankle_axis(msk_key, planar_root):
    model, data = _qpos0(msk_key, planar_root)
    for side, s in SIDES:
        ankle = model.joint("ankle_angle_" + side).id
        hinge = model.joint(PREFIX + "joint2_" + s).id
        axis = data.xaxis[ankle]
        offset = data.xanchor[hinge] - data.xanchor[ankle]
        assert np.linalg.norm(np.cross(data.xaxis[hinge], axis)) < AXIS_TOL, (
            f"{msk_key} {s}: joint2 is not parallel to the ankle"
        )
        assert np.linalg.norm(np.cross(offset, axis)) < AXIS_TOL, f"{msk_key} {s}: joint2 is off the ankle axis"


@pytest.mark.parametrize("planar_root", BUILDS)
@pytest.mark.parametrize("msk_key", MSKS)
def test_foot_part_sits_where_myolegs_has_it(msk_key, planar_root):
    for side, s in SIDES:
        pos, rot = _foot_part_pose(msk_key, side, s, planar_root)
        ref_pos, ref_rot = _foot_part_pose("myolegs", side, s)
        shift = np.linalg.norm(pos - ref_pos)
        angle = np.arccos(np.clip((np.trace(ref_rot.T @ rot) - 1.0) / 2.0, -1.0, 1.0))
        assert shift < POS_TOL, f"{msk_key} {s}: part3 is {shift * 1000:.1f} mm from the myolegs pose"
        assert angle < ANGLE_TOL, f"{msk_key} {s}: part3 is turned {np.degrees(angle):.2f} deg from the myolegs pose"


@pytest.mark.parametrize("msk_key", MSKS)
def test_left_exo_mirrors_the_right(msk_key):
    model, data = _qpos0(msk_key)
    right, left = (_foot_frame(model, data, side) for side in ("r", "l"))
    for site in MIRROR_SITES:
        p_dx = right[1].T @ (data.site_xpos[model.site(PREFIX + site + "dx").id] - right[0])
        p_sx = left[1].T @ (data.site_xpos[model.site(PREFIX + site + "sx").id] - left[0])
        gap = np.linalg.norm(p_sx * [1.0, -1.0, 1.0] - p_dx)
        assert gap < POS_TOL, f"{msk_key}: {site}sx is {gap * 1000:.1f} mm from the mirror image of {site}dx"


@pytest.mark.parametrize("msk_key", MSKS)
def _is_device(model, e):
    """The connects are unnamed, so they are found by their bodies."""
    if model.eq_objtype[e] == mj.mjtObj.mjOBJ_SITE:
        bodies = [model.site_bodyid[model.eq_obj1id[e]], model.site_bodyid[model.eq_obj2id[e]]]
    else:
        bodies = [model.eq_obj1id[e], model.eq_obj2id[e]]
    return any(model.body(b).name.startswith(PREFIX) for b in bodies)


@pytest.mark.parametrize("msk_key", MSKS)
def test_planar_root_keyframes_close_the_connects(msk_key):
    model, data = _qpos0(msk_key, True)
    eqs = [e for e in range(model.neq) if model.eq_type[e] == mj.mjtEq.mjEQ_CONNECT and _is_device(model, e)]
    assert len(eqs) == 6, f"{msk_key}: expected 6 UT connects, found {len(eqs)}"
    for k in range(model.nkey):
        mj.mj_resetDataKeyframe(model, data, k)
        mj.mj_forward(model, data)
        n = data.nefc
        rows = data.efc_type[:n] == mj.mjtConstraint.mjCNSTR_EQUALITY
        for e in eqs:
            residual = float(np.linalg.norm(data.efc_pos[:n][rows & (data.efc_id[:n] == e)]))
            assert residual < MAX_RESIDUAL, (
                f"{msk_key} planar_root, keyframe '{model.key(k).name}': connect {e} is open by {residual * 1000:.2f} mm"
            )
