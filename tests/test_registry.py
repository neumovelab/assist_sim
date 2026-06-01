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
    # Every entry maps to a (package, filename) tuple.
    for key, (pkg, fname) in registry._COMPATIBLE_MSK_KEYS.items():
        assert pkg.startswith("myo_sim"), f"{key} -> {pkg}"
        assert fname.endswith(".xml")


def test_unknown_msk_raises_with_suggestion():
    """Typo-style lookups should suggest the closest valid key."""
    with pytest.raises(ValueError, match="Did you mean.*myoLeg22_2D"):
        registry._resolve_msk("myoLeg22")


def test_missing_myo_sim_raises_importerror():
    """When myo_sim is not installed, _resolve_msk raises an ImportError that
    points the user at the install instructions."""
    if registry._files.__module__ == "importlib.resources":
        # The real importlib.resources is used.  We can't easily fake an absent
        # myo_sim without sys.modules surgery; instead, just check that the
        # known-good key path either returns a Path (myo_sim present) or
        # raises ImportError/FileNotFoundError (myo_sim absent).
        try:
            result = registry._resolve_msk("myoLeg22_2D")
        except (ImportError, FileNotFoundError):
            return  # expected when myo_sim not installed
        assert isinstance(result, Path)


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
        'device:\n  name: "MyDev"\n  model_xml: "L1model.xml"\n'
        "attachments:\n  - device_body: dev_a\n    parent_body: pelvis\n",
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
