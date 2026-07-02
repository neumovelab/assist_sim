"""Tests for the device autodiscovery + explicit MSK registry."""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest

from assist_sim import registry

FIXTURES = Path(__file__).resolve().parent / "fixtures"


# ----------------------------------------------------------------------
# Explicit MSK registry
# ----------------------------------------------------------------------


def test_compatible_msk_keys_are_locked():
    """The set of pipeline-compatible MSK keys is curated, not autodiscovered."""
    assert set(registry._COMPATIBLE_MSK_KEYS) == {
        "myoLeg22_2D",
        "myoLeg26_3D",
        "myoLeg80",
    }
    # Every entry binds a myo_sim composed model (or None when planned) and a
    # minimum MuJoCo version.
    for key, src in registry._COMPATIBLE_MSK_KEYS.items():
        assert src.myo_sim_model is None or isinstance(src.myo_sim_model, str), key
        assert len(src.min_mujoco) == 3, key
    # myoLeg26_3D is the Phase-1 model, backed by myo_sim's legs-only myolegs26.
    assert registry._COMPATIBLE_MSK_KEYS["myoLeg26_3D"].myo_sim_model == "myolegs26"


def test_unknown_msk_raises_with_suggestion():
    """Typo-style lookups should suggest the closest valid key."""
    with pytest.raises(ValueError, match="Did you mean.*myoLeg22_2D"):
        registry._resolve_msk("myoLeg22")


def test_planned_msk_raises_value_error():
    """A registered-but-not-yet-available MSK (no myo_sim source) errors, not
    warns, with an explanation."""
    with pytest.raises(ValueError, match="not available yet"):
        registry._resolve_msk("myoLeg22_2D")


def test_resolve_msk_composes_or_raises():
    """With myo_sim present, resolving the Phase-1 MSK composes a model-only XML
    on disk; without it, _resolve_msk raises an ImportError pointing at install."""
    try:
        result = registry._resolve_msk("myoLeg26_3D")
    except ImportError:
        return  # expected when myo_sim not installed
    assert isinstance(result, Path) and result.exists()


# ----------------------------------------------------------------------
# Device autodiscovery
# ----------------------------------------------------------------------


@pytest.fixture
def temp_models(tmp_path):
    """Build a temp models/ tree (devices only) and point the registry at it."""
    root = tmp_path / "models"

    # Device dir with config + model.
    dev = root / "DevDir"
    dev.mkdir(parents=True)
    shutil.copy(FIXTURES / "minimal_device.xml", dev / "L1model.xml")
    (dev / "L1config.yaml").write_text(
        'device:\n  name: "MyDev"\n  model_xml: "L1model.xml"\nattachments:\n  - device_body: dev_a\n    parent_body: pelvis\n',
        encoding="utf-8",
    )

    # Second device, only compatible with myoLeg22_2D.
    dev2 = root / "PickyDir"
    dev2.mkdir(parents=True)
    shutil.copy(FIXTURES / "minimal_device.xml", dev2 / "L1model.xml")
    (dev2 / "L1config.yaml").write_text(
        'device:\n  name: "Picky"\n  model_xml: "L1model.xml"\n'
        "  compatible_msk: [myoLeg22_2D]\n"
        "attachments:\n  - device_body: dev_a\n    parent_body: pelvis\n",
        encoding="utf-8",
    )

    original = registry.MODELS_ROOT
    registry.MODELS_ROOT = root
    registry.refresh()
    try:
        yield root
    finally:
        registry.MODELS_ROOT = original
        registry.refresh()


def test_discovers_devices(temp_models):
    assert "DevDir_L1" in registry.DEVICE_CONFIGS
    assert "PickyDir_L1" in registry.DEVICE_CONFIGS


def test_device_name_alias_registered(temp_models):
    """`device.name` is registered as an alias to the filename-derived key."""
    assert registry._DEVICE_ALIASES.get("MyDev") == "DevDir_L1"
    assert registry._DEVICE_ALIASES.get("Picky") == "PickyDir_L1"


def test_compatibility_filter_recorded(temp_models):
    """`device.compatible_msk` is captured for filtering by get_available_combinations."""
    assert registry._COMPATIBLE_MSK.get("PickyDir_L1") == ["myoLeg22_2D"]
    assert registry._COMPATIBLE_MSK.get("DevDir_L1") is None  # no restriction


def test_unknown_device_key_raises_with_suggestion(temp_models):
    with pytest.raises(ValueError, match="Did you mean.*DevDir_L1"):
        registry._resolve_device_key("DevDir_l1")  # case typo


def test_real_repo_devices_discovered():
    """Smoke: the live models/ scan finds the actual repo devices."""
    expected = {
        "DephyExoBoot_L1",
        "HMEDI_L1",
        "Humotech_L1",
        "OpenExo_L1",
        "OpenSourceLeg_A_L1",
        "OpenSourceLeg_KA_L1",
        "Tutorial_L1",
    }
    assert expected.issubset(set(registry.DEVICE_CONFIGS))
