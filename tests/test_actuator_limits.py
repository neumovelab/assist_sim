"""Actuator force limits, and device-XML actuators that the import cannot carry.

``forcerange`` in a YAML actuator used to be rejected by the schema, so a joint motor could not
be force-limited from any file: a device-XML joint actuator was skipped without a message, and
an XML attribute the import does not read (``kp``, ``class``, ``lengthrange``) changed nothing.
Each is now either carried into the combined model or a ``ValueError``.
"""

from __future__ import annotations

from pathlib import Path

import mujoco as mj
import pytest

from assist_sim.combine import ModelCombiner
from assist_sim.config import DeviceConfig

FIXTURES = Path(__file__).resolve().parent / "fixtures"

CONFIG = """
device:
  name: "TestDev"
  model_xml: "{xml}"
attachments:
  - device_body: "dev_a"
    parent_body: "pelvis"
  - device_body: "dev_b"
    parent_body: "thigh"
actuators:
  - name: "dev_act"
    type: "general"
    joint: "dev_joint"
{extra}
"""

# The minimal device plus a cable between its two bodies, driven from the XML.
CABLE_XML = """<mujoco model="cable_device">
  <compiler angle="radian"/>
  <worldbody>
    <body name="dev_a" pos="0 0 0">
      <joint name="dev_joint" type="hinge" axis="0 1 0"/>
      <geom name="dev_a_geom" type="sphere" size="0.02" mass="0.1"/>
      <site name="s1" pos="0.05 0 0"/>
    </body>
    <body name="dev_b" pos="0 0 0">
      <geom name="dev_b_geom" type="sphere" size="0.02" mass="0.1"/>
      <site name="s2" pos="0.05 0 -0.2"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="cable"><site site="s1"/><site site="s2"/></spatial>
  </tendon>
  <actuator>
    {actuator}
  </actuator>
</mujoco>
"""


def _config(tmp_path: Path, extra: str = "", actuator: str | None = None) -> DeviceConfig:
    if actuator is None:
        xml = "minimal_device.xml"
        (tmp_path / xml).write_bytes((FIXTURES / xml).read_bytes())
    else:
        xml = "cable_device.xml"
        (tmp_path / xml).write_text(CABLE_XML.format(actuator=actuator), encoding="utf-8")
    path = tmp_path / "L1config.yaml"
    path.write_text(CONFIG.format(xml=xml, extra=extra), encoding="utf-8")
    return DeviceConfig.from_yaml(str(path))


def _combine(config: DeviceConfig):
    return ModelCombiner().combine(mj.MjSpec.from_file(str(FIXTURES / "minimal_human.xml")), config)


def test_yaml_forcerange_limits_the_actuator(tmp_path):
    model, data = _combine(_config(tmp_path, "    forcerange: [-5, 5]"))
    aid = model.actuator("dev_act").id
    assert model.actuator_forcelimited[aid] == 1
    assert list(model.actuator_forcerange[aid]) == [-5.0, 5.0]
    data.ctrl[aid] = 100.0  # gain 1, so 100 unclamped
    mj.mj_forward(model, data)
    assert data.actuator_force[aid] == pytest.approx(5.0)


def test_forcelimited_true_with_a_range_is_accepted(tmp_path):
    model, _ = _combine(_config(tmp_path, "    forcerange: [-5, 5]\n    forcelimited: true"))
    assert model.actuator_forcelimited[model.actuator("dev_act").id] == 1


def test_without_forcerange_the_actuator_is_unlimited(tmp_path):
    model, data = _combine(_config(tmp_path))
    aid = model.actuator("dev_act").id
    assert model.actuator_forcelimited[aid] == 0
    data.ctrl[aid] = 100.0
    mj.mj_forward(model, data)
    assert data.actuator_force[aid] == pytest.approx(100.0)


@pytest.mark.parametrize(
    "extra,match",
    [
        ("    forcerange: [5, -5]", "lo <= hi"),
        ("    forcerange: [-5, 0, 5]", "lo <= hi"),
        ("    forcelimited: true", "without 'forcerange"),
        ("    forcerange: [-5, 5]\n    forcelimited: false", "forcelimited: false"),
    ],
)
def test_inconsistent_force_limits_are_rejected(tmp_path, extra, match):
    with pytest.raises(ValueError, match=match):
        _config(tmp_path, extra)


def test_unknown_gaintype_is_rejected(tmp_path):
    config = _config(tmp_path, '    gaintype: "fixd"')
    with pytest.raises(ValueError, match="fixd"):
        _combine(config)


def test_xml_cable_actuator_carries_its_force_limit(tmp_path):
    config = _config(tmp_path, actuator='<general name="pull" tendon="cable" gainprm="10" forcerange="-3 0"/>')
    model, _ = _combine(config)
    aid = model.actuator("TestDev_pull").id
    assert model.actuator_forcelimited[aid] == 1
    assert list(model.actuator_forcerange[aid]) == [-3.0, 0.0]


def test_xml_joint_actuator_is_rejected(tmp_path):
    config = _config(tmp_path, actuator='<motor name="hip_motor" joint="dev_joint" forcerange="-5 5"/>')
    with pytest.raises(ValueError, match="joint transmission"):
        _combine(config)


@pytest.mark.parametrize(
    "actuator,match",
    [
        ('<general name="pull" tendon="cable" gainprm="10" lengthrange="0 1"/>', "lengthrange"),
        ('<general name="pull" tendon="cable" class="cables"/>', "class"),
        ('<position name="pull" tendon="cable" kp="50"/>', "<position>"),
        ('<general name="pull" tendon="cable" biastype="afine"/>', "afine"),
    ],
)
def test_xml_actuator_attributes_the_import_cannot_carry_are_rejected(tmp_path, actuator, match):
    config = _config(tmp_path, actuator=actuator)
    with pytest.raises(ValueError, match=match):
        _combine(config)
