"""Frozen regression net: every supported MSK x device combination.

Asserts the compiled ``(nq, nu, nbody, nmesh)`` for each combination matches a
frozen expected tuple.  The MSK side of each pair is resolved via the
``myo_sim`` package (see ``assist_sim.registry``); tests requiring myo_sim are
skipped automatically when it isn't installed.

The numbers below were captured on ``mujoco==3.3.3`` and must hold there.
"""

from __future__ import annotations

import mujoco as mj
import pytest

from assist_sim import load_combined_model
from assist_sim.registry import resolve

from .conftest import needs_myo_sim

# (msk_key, device_key) -> (nq, nu, nbody, nmesh)
EXPECTED = {
    ("myoLeg22_2D", "DephyExoBoot_L1"):     (53, 24, 51, 44),
    ("myoLeg26_3D", "DephyExoBoot_L1"):     (60, 28, 51, 44),
    ("myoLeg22_2D", "OpenSourceLeg_A_L1"):  (52, 19, 38, 35),
    ("myoLeg26_3D", "OpenSourceLeg_A_L1"):  (59, 23, 38, 35),
    ("myoLeg22_2D", "OpenSourceLeg_KA_L1"): (44, 17, 37, 37),
    ("myoLeg26_3D", "OpenSourceLeg_KA_L1"): (51, 21, 37, 37),
    ("myoLeg80",    "DephyExoBoot_L1"):     (35, 82, 31, 31),
    ("myoLeg80",    "OpenSourceLeg_A_L1"):  (33, 69, 18, 22),
    ("myoLeg80",    "OpenSourceLeg_KA_L1"): (29, 56, 20, 24),
}


@needs_myo_sim
@pytest.mark.parametrize("keys,expected", list(EXPECTED.items()), ids=lambda x: str(x))
def test_combination_signature(keys, expected):
    msk_key, device_key = keys
    msk_path, device_path = resolve(msk_key, device_key)
    model, _ = load_combined_model(
        human_xml=str(msk_path),
        device_config=str(device_path),
        msk_key=msk_key,
    )
    actual = (model.nq, model.nu, model.nbody, model.nmesh)
    assert actual == expected


@needs_myo_sim
@pytest.mark.parametrize("keys", list(EXPECTED), ids=lambda x: str(x))
def test_combination_is_simulatable(keys):
    """A compiled combination steps without error (no rollout, just stability)."""
    msk_key, device_key = keys
    msk_path, device_path = resolve(msk_key, device_key)
    model, data = load_combined_model(
        human_xml=str(msk_path),
        device_config=str(device_path),
        msk_key=msk_key,
    )
    mj.mj_forward(model, data)
    for _ in range(5):
        mj.mj_step(model, data)


@needs_myo_sim
def test_hmedi_cable_tendons_and_actuators_imported():
    """HMEDI's device-XML <tendon>/<actuator> sections (cable_r/l + Exo_R/L)
    must reach the combined model with the device prefix."""
    msk_path, device_path = resolve("myoLeg22_2D", "HMEDI_L1")
    model, _ = load_combined_model(human_xml=str(msk_path), device_config=str(device_path))
    actuators = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)}
    tendons = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_TENDON, i) for i in range(model.ntendon)}
    assert "HMEDI_L1_Exo_R" in actuators
    assert "HMEDI_L1_Exo_L" in actuators
    assert "HMEDI_L1_cable_r" in tendons
    assert "HMEDI_L1_cable_l" in tendons


def test_hmedi_torso_per_msk_attachment_on_80(models_dir):
    """myoLeg80 attaches hmedi_torso to pelvis (not torso) with a compensating
    pos offset.  Pure config-resolution test -- doesn't require myo_sim to run."""
    from assist_sim.config import DeviceConfig
    config = DeviceConfig.from_yaml(str(models_dir / "HMEDI" / "L1config.yaml"))
    default_atts = {a.device_body: a for a in config.resolve_attachments()}
    msk80_atts = {a.device_body: a for a in config.resolve_attachments("myoLeg80")}
    assert default_atts["hmedi_torso"].parent_body == "torso"
    assert default_atts["hmedi_torso"].pos is None
    assert msk80_atts["hmedi_torso"].parent_body == "pelvis"
    assert msk80_atts["hmedi_torso"].pos == [-0.105, 0.08, 0]
    assert msk80_atts["hmedi_torso"].quat is None
