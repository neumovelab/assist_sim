"""Tests for the WrapEdit tendon-modification schema."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from assist_sim.config import DeviceConfig, TendonModification, WrapEdit
from assist_sim.preprocess import preprocess_human_xml


def _config(**kwargs) -> DeviceConfig:
    return DeviceConfig(name="TestDev", model_xml="minimal_device.xml", attachments=[], **kwargs)


def _run(human, config):
    result = preprocess_human_xml(human, config)
    root = ET.parse(result.path).getroot()
    Path(result.path).unlink(missing_ok=True)
    return root


def _wrap_sites(root, tendon_name):
    spatial = next(s for s in root.iter("spatial") if s.get("name") == tendon_name)
    return [w.get("site") for w in spatial if w.tag == "site"]


def _site_parent_body(root, site_name):
    for body in root.iter("body"):
        for child in body:
            if child.tag == "site" and child.get("name") == site_name:
                return body.get("name")
    return None


def test_drop_site(minimal_human):
    mod = TendonModification("calf_tendon", [WrapEdit(op="drop_site", site="shank_site")])
    root = _run(minimal_human, _config(tendon_modifications=[mod]))
    assert _wrap_sites(root, "calf_tendon") == ["thigh_site", "foot_site"]


def test_reposition_site(minimal_human):
    mod = TendonModification(
        "calf_tendon",
        [WrapEdit(op="reposition_site", site="shank_site", pos=[0.0, 0.0, -0.25])],
    )
    root = _run(minimal_human, _config(tendon_modifications=[mod]))
    wraps = _wrap_sites(root, "calf_tendon")
    assert "shank_site__mod" in wraps
    # new site is created on the SAME body the original sat on (shank)
    assert _site_parent_body(root, "shank_site__mod") == "shank"


def test_replace_site_lands_on_named_body(minimal_human):
    mod = TendonModification(
        "calf_tendon",
        [WrapEdit(op="replace_site", site="foot_site", new_body="thigh", pos=[0.1, 0.0, 0.0])],
    )
    root = _run(minimal_human, _config(tendon_modifications=[mod]))
    wraps = _wrap_sites(root, "calf_tendon")
    assert "foot_site__mod" in wraps
    assert _site_parent_body(root, "foot_site__mod") == "thigh"


def test_tendon_attributes_preserved_through_edit(minimal_human):
    mod = TendonModification("calf_tendon", [WrapEdit(op="drop_site", site="foot_site")])
    root = _run(minimal_human, _config(tendon_modifications=[mod]))
    spatial = next(s for s in root.iter("spatial") if s.get("name") == "calf_tendon")
    assert spatial.get("springlength") == "0.3"
    assert spatial.get("stiffness") == "5"
    assert spatial.get("damping") == "0.1"
    assert spatial.get("width") == "0.005"


def test_repositioning_same_wrap_twice_raises(minimal_human):
    # The first edit rewrites the wrap to the synthesized site, so a second
    # edit of the original site can no longer resolve -> raises (per the doc's
    # "same site repositioned twice -> raises" rule).
    mod = TendonModification(
        "calf_tendon",
        [
            WrapEdit(op="reposition_site", site="shank_site", pos=[0, 0, -0.25]),
            WrapEdit(op="reposition_site", site="shank_site", pos=[0, 0, -0.26]),
        ],
    )
    with pytest.raises(ValueError):
        preprocess_human_xml(minimal_human, _config(tendon_modifications=[mod]))


def test_unknown_tendon_raises(minimal_human):
    mod = TendonModification("nope_tendon", [WrapEdit(op="drop_site", site="x")])
    with pytest.raises(ValueError, match="unknown tendon 'nope_tendon'"):
        preprocess_human_xml(minimal_human, _config(tendon_modifications=[mod]))
