"""Tests for the standalone config validator and runtime raise policy."""

from __future__ import annotations

import pytest

from assist_sim.config import DeviceConfig
from assist_sim.combine import ModelCombiner
from assist_sim.validate import validate_config


def test_clean_config_has_no_issues(minimal_human, minimal_device_config):
    config = DeviceConfig.from_yaml(minimal_device_config)
    assert validate_config(minimal_human, config) == []


def test_typo_body_removal_reported(minimal_human, minimal_device_config):
    config = DeviceConfig.from_yaml(minimal_device_config)
    config.body_removals = ["thiigh"]
    issues = validate_config(minimal_human, config)
    assert any("body_removals" in i and "thiigh" in i for i in issues)
    assert any("thigh" in i for i in issues)  # suggestion present


def test_typo_joint_override_reported(minimal_human, minimal_device_config):
    from assist_sim.config import JointOverride

    config = DeviceConfig.from_yaml(minimal_device_config)
    config.joint_overrides = [JointOverride(name="kneee", range=[-1, 0])]
    issues = validate_config(minimal_human, config)
    assert any("joint_overrides" in i and "kneee" in i for i in issues)


def test_runtime_combine_raises_with_suggestion(minimal_human, minimal_device_config):
    config = DeviceConfig.from_yaml(minimal_device_config)
    config.body_removals = ["thiigh"]
    with pytest.raises(ValueError, match="Did you mean.*'thigh'"):
        ModelCombiner().combine(minimal_human, config)
