"""Shared pytest fixtures for the assist_sim test suite."""

from __future__ import annotations

import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURES = Path(__file__).resolve().parent / "fixtures"
MODELS = REPO_ROOT / "models"

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


# ----------------------------------------------------------------------
# myo_sim availability gate
# ----------------------------------------------------------------------
# MSK models live in the myo_sim package (not in this repo). Tests that
# need a real MSK file are skipped automatically when myo_sim isn't
# installed -- avoids forcing every contributor / CI lane to have the
# (currently unpublished) wheel before they can run anything.

HAS_MYO_SIM = importlib.util.find_spec("myo_sim") is not None

needs_myo_sim = pytest.mark.skipif(
    not HAS_MYO_SIM,
    reason="requires myo_sim package (not yet installed)",
)


@pytest.fixture
def repo_root() -> Path:
    return REPO_ROOT


@pytest.fixture
def models_dir() -> Path:
    """Local device-models directory (devices only; MSKs come from myo_sim)."""
    return MODELS


@pytest.fixture
def minimal_human() -> str:
    return str(FIXTURES / "minimal_human.xml")


@pytest.fixture
def minimal_device_config() -> str:
    return str(FIXTURES / "minimal_device_config.yaml")


@pytest.fixture
def human_tree(minimal_human) -> ET.Element:
    """Parsed ElementTree root of the minimal human fixture."""
    return ET.parse(minimal_human).getroot()
