"""Frozen regression net: every supported MSK x device combination.

Asserts the compiled ``(nq, nu, nbody, nmesh)`` for each combination matches a
frozen expected tuple.  The MSK side of each pair is composed at runtime via the
``myo_sim`` package (see ``assist_sim.registry``); tests requiring myo_sim are
skipped automatically when it isn't installed.

Phase 1 of the myo_sim integration wires the legs-only ``myolegs26`` model,
which is the only MSK buildable on the pinned ``mujoco==3.3.3``.  ``myoLeg80``
(passive torso) needs ``MjSpec.delete`` from mujoco 3.3.4+ (Phase 2) and
``myoLeg22_2D`` has no source yet (a planned 26->22 mjspec reduction); both are
covered by :func:`test_gated_msk_raises`.  The numbers below were captured on
``mujoco==3.3.3`` and must hold there.
"""

from __future__ import annotations

import mujoco as mj
import pytest

from assist_sim import load_combined_model
from assist_sim.registry import _msk_available, resolve

from .conftest import needs_myo_sim

# (msk_key, device_key) -> (nq, nu, nbody, nmesh), captured on mujoco 3.3.3.
# assist_sim emits model-only XMLs: the bundled myosuite scene (floor, backdrop,
# pedestal, logo) is stripped, so no ground body / scene mesh is counted here.
EXPECTED = {
    ("myoLeg26_3D", "DephyExoBoot_L1"): (47, 28, 37, 28),
    ("myoLeg26_3D", "OpenSourceLeg_A_L1"): (46, 23, 24, 19),
    ("myoLeg26_3D", "OpenSourceLeg_KA_L1"): (38, 21, 23, 21),
}

# MSKs that are registered but not buildable in Phase 1, and the error each
# raises when resolved.  ValueError: no source yet; ImportError: needs 3.3.4.
GATED = {
    "myoLeg22_2D": ValueError,
    "myoLeg80": ImportError,
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
@pytest.mark.parametrize("msk_key,exc", list(GATED.items()))
def test_gated_msk_raises(msk_key, exc):
    """MSKs without a Phase-1 source raise a clear error (no silent fallback)."""
    with pytest.raises(exc):
        resolve(msk_key, "DephyExoBoot_L1")


@needs_myo_sim
@pytest.mark.skipif(
    not _msk_available("myoLeg80"),
    reason="HMEDI's torso band targets a torso'd MSK (myoLeg80), buildable only on mujoco>=3.3.4 (Phase 2)",
)
def test_hmedi_cable_tendons_and_actuators_imported():
    """HMEDI's device-XML <tendon>/<actuator> sections (cable_r/l + Exo_R/L)
    must reach the combined model with the device prefix."""
    msk_path, device_path = resolve("myoLeg80", "HMEDI_L1")
    model, _ = load_combined_model(human_xml=str(msk_path), device_config=str(device_path), msk_key="myoLeg80")
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
