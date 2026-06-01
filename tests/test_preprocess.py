"""Unit tests for the XML preprocess layer (each pass in isolation)."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from assist_sim.config import DeviceConfig
from assist_sim.preprocess import preprocess_human_xml


def _config(**kwargs) -> DeviceConfig:
    """Build a bare DeviceConfig for exercising individual preprocess passes."""
    return DeviceConfig(name="TestDev", model_xml="minimal_device.xml", attachments=[], **kwargs)


def _run(human: str, config: DeviceConfig):
    """Run preprocess and return (parsed_root, PreprocessResult); cleans temp."""
    result = preprocess_human_xml(human, config)
    root = ET.parse(result.path).getroot()
    Path(result.path).unlink(missing_ok=True)
    return root, result


def _names(root, tag):
    return {e.get("name") for e in root.iter(tag) if e.get("name")}


def _find_body(root, name):
    for body in root.iter("body"):
        if body.get("name") == name:
            return body
    return None


def test_body_removal_drops_subtree(minimal_human):
    root, result = _run(minimal_human, _config(body_removals=["foot"]))
    bodies = _names(root, "body")
    assert "foot" not in bodies and "toe" not in bodies
    assert {"pelvis", "thigh", "shank", "pp"} <= bodies
    # joints on removed bodies are gone
    joints = {j.get("name") for j in root.iter("joint")}
    assert "ankle" not in joints and "mtp" not in joints
    assert result.removed_bodies == {"foot", "toe"}
    assert {"ankle", "mtp"} <= result.removed_joints


def test_body_removal_cascades_contact_and_sensor(minimal_human):
    root, _ = _run(minimal_human, _config(body_removals=["foot"]))
    pairs = [(p.get("geom1"), p.get("geom2")) for p in root.iter("pair")]
    assert ("foot_geom", "toe_geom") not in pairs
    assert ("pelvis_geom", "shank_geom") in pairs  # untouched pair survives
    sensor_sites = {s.get("site") for s in root.iter("touch")}
    assert "foot_site" not in sensor_sites


def test_body_removal_auto_prunes_tendon_wrap(minimal_human):
    root, _ = _run(minimal_human, _config(body_removals=["foot"]))
    spatial = next(s for s in root.iter("spatial") if s.get("name") == "calf_tendon")
    wrap_sites = [w.get("site") for w in spatial if w.tag == "site"]
    assert wrap_sites == ["thigh_site", "shank_site"]  # foot_site dropped, order kept
    # non-default tendon attributes preserved
    assert spatial.get("springlength") == "0.3"
    assert spatial.get("stiffness") == "5"
    assert spatial.get("damping") == "0.1"


def test_equality_cascade_on_joint_removal(minimal_human):
    root, _ = _run(minimal_human, _config(body_removals=["pp"]))
    equality = root.find("equality")
    joints_eq = list(equality) if equality is not None else []
    assert joints_eq == []  # the only equality referenced pp_x


def test_actuator_removal(minimal_human):
    root, _ = _run(minimal_human, _config(actuator_removals=["knee_act"]))
    act_names = {a.get("name") for a in root.find("actuator")}
    assert "knee_act" not in act_names
    assert "calf_act" in act_names


def test_tendon_removal(minimal_human):
    root, _ = _run(minimal_human, _config(tendon_removals=["calf_tendon"]))
    tendon = root.find("tendon")
    assert tendon is None or all(s.get("name") != "calf_tendon" for s in tendon)


def test_keyframe_section_stripped(minimal_human):
    root, result = _run(minimal_human, _config())
    assert root.find("keyframe") is None
    assert "home" in result.keyframes  # parsed before stripping


def test_unknown_body_removal_raises(minimal_human):
    with pytest.raises(ValueError, match="unknown body 'fooot'"):
        preprocess_human_xml(minimal_human, _config(body_removals=["fooot"]))


from .conftest import needs_myo_sim  # noqa: E402


@needs_myo_sim
def test_inline_includes_split_msk():
    """Split chain/assets MSKs have includes expanded so body_removals resolve."""
    from assist_sim.registry import resolve

    human = str(resolve("myoLeg80", "DephyExoBoot_L1")[0])
    cfg = _config(body_removals=["talus_r"])
    result = preprocess_human_xml(human, cfg, msk_key="myoLeg80")
    root = ET.parse(result.path).getroot()
    Path(result.path).unlink(missing_ok=True)
    assert _find_body(root, "talus_r") is None
    assert _find_body(root, "tibia_r") is not None
    # chain/assets includes are inlined; no nested include tags remain under worldbody.
    wb = root.find("worldbody")
    assert wb is not None
    assert list(wb.iter("include")) == []
