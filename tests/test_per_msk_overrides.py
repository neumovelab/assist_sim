"""Tests for per-MSK config override resolution (phase 11 schema)."""

from __future__ import annotations

import shutil
from pathlib import Path

from assist_sim.config import DeviceConfig

FIXTURES = Path(__file__).resolve().parent / "fixtures"

PER_MSK_YAML = """\
device:
  name: "TestDev"
  model_xml: "minimal_device.xml"

attachments:
  - device_body: "dev_a"
    parent_body: "pelvis"

actuator_removals:
  default:
    - "soleus_r"
  myolegs:
    - "soleus_r"
    - "soleus80_r"

tendon_modifications:
  default:
    - name: "calf_tendon"
      wraps:
        - drop_site: "foot_site"
  myolegs:
    - name: "calf80_tendon"
      wraps:
        - drop_site: "foot80_site"

keyframe_overrides:
  default:
    stand:
      pelvis_ty: 0.91
  myolegs:
    stand:
      pelvis_ty: 0.95
"""

DEFAULT_YAML = """\
device:
  name: "TestDev"
  model_xml: "minimal_device.xml"

attachments:
  - device_body: "dev_a"
    parent_body: "pelvis"

actuator_removals:
  - "soleus_r"

keyframe_overrides:
  stand:
    pelvis_ty: 0.91
"""


def _write_config(tmp_path, text) -> str:
    shutil.copy(FIXTURES / "minimal_device.xml", tmp_path / "minimal_device.xml")
    cfg = tmp_path / "config.yaml"
    cfg.write_text(text, encoding="utf-8")
    return str(cfg)


def test_per_msk_actuator_removals(tmp_path):
    config = DeviceConfig.from_yaml(_write_config(tmp_path, PER_MSK_YAML))
    assert config.resolve_actuator_removals() == ["soleus_r"]
    assert config.resolve_actuator_removals("myolegs22") == ["soleus_r"]  # falls back
    assert config.resolve_actuator_removals("myolegs") == ["soleus_r", "soleus80_r"]


def test_per_msk_tendon_modifications(tmp_path):
    config = DeviceConfig.from_yaml(_write_config(tmp_path, PER_MSK_YAML))
    assert config.resolve_tendon_modifications()[0].name == "calf_tendon"
    assert config.resolve_tendon_modifications("myolegs")[0].name == "calf80_tendon"


def test_per_msk_keyframe_overrides(tmp_path):
    config = DeviceConfig.from_yaml(_write_config(tmp_path, PER_MSK_YAML))
    assert config.resolve_keyframe_overrides()["stand"].joint_values == {"pelvis_ty": 0.91}
    assert config.resolve_keyframe_overrides("myolegs")["stand"].joint_values == {"pelvis_ty": 0.95}


def test_default_form_still_works(tmp_path):
    config = DeviceConfig.from_yaml(_write_config(tmp_path, DEFAULT_YAML))
    # any msk_key falls back to the single default form
    assert config.resolve_actuator_removals("anything") == ["soleus_r"]
    assert config.resolve_keyframe_overrides("anything")["stand"].joint_values == {"pelvis_ty": 0.91}


PER_MSK_ATTACH_YAML = """\
device:
  name: "TestDev"
  model_xml: "minimal_device.xml"

attachments:
  default:
    - device_body: "dev_a"
      parent_body: "pelvis"
  myolegs:
    - device_body: "dev_a"
      parent_body: "pelvis"
      quat: [0.5, 0.5, 0.5, 0.5]
"""


def test_per_msk_attachments_resolve(tmp_path):
    config = DeviceConfig.from_yaml(_write_config(tmp_path, PER_MSK_ATTACH_YAML))
    # default has no quat
    default_atts = config.resolve_attachments()
    assert len(default_atts) == 1
    assert default_atts[0].quat is None
    # myolegs overrides with quat
    msk80 = config.resolve_attachments("myolegs")
    assert msk80[0].quat == [0.5, 0.5, 0.5, 0.5]
    # unknown msk falls back to default
    assert config.resolve_attachments("unknown")[0].quat is None
