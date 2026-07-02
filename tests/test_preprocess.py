"""Tests for in-memory model surgery (removals via ``spec.delete``).

Model surgery moved from an ElementTree pass to in-memory ``MjSpec.delete``
(see :mod:`assist_sim.combine`), so these exercise the removal cascade + scene
strip end-to-end through ``load_combined`` on a composed MSK.
"""

from __future__ import annotations

import mujoco as mj

from assist_sim import load_combined

from .conftest import needs_myo_sim


@needs_myo_sim
def test_body_removal_cascades_on_composed_msk():
    """OSL_A removes ``talus_r``; its subtree (calcn_r, toes_r) and the
    muscles/sensors that referenced it cascade away via ``spec.delete``, while
    the parent ``tibia_r`` survives."""
    model, _ = load_combined("myolegs26", "OpenSourceLeg_A_L1")
    names = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)}
    assert not ({"talus_r", "calcn_r", "toes_r"} & names)  # amputated subtree gone
    assert "tibia_r" in names  # parent survives


@needs_myo_sim
def test_actuator_removal_and_scene_strip():
    """The ankle/foot muscles crossing the removed joint are gone (explicitly
    removed or cascaded), and the output is model-only (myosuite floor stripped)."""
    model, _ = load_combined("myolegs26", "OpenSourceLeg_A_L1")
    acts = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)}
    assert not ({"soleus_r", "tibant_r", "edl_r", "fdl_r"} & acts)
    geoms = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, i) for i in range(model.ngeom)}
    assert "floor" not in geoms  # scene stripped at resolution
