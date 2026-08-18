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
        - replace_site: "foot_site"
          new_body: "shank"
          pos: [0.0, -0.2, 0.0]
  myolegs:
    - name: "calf80_tendon"
      wraps:
        - replace_geom: "calf80_wrap"
          new_body: "shank"
          pos: [0.0, -0.2, 0.0]

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


def test_public_field_is_what_the_pipeline_reads(tmp_path):
    """Mutating a public ``DeviceConfig`` field must reach ``resolve_*``.

    ``_resolve`` used to read ``_<section>_by_msk["default"]`` before falling back, and
    ``from_yaml`` always populates that key -- so every public list field on the documented
    dataclass became a decoy once a config was loaded. Assigning one was a silent no-op for
    the pipeline while remaining visible to ``validate_config``, which reads the public
    fields; the two disagreed about the same config. ``docs/how-to/debug-a-combined-model.md``
    even suggests the assignment for scoping the validator.
    """
    config = DeviceConfig.from_yaml(_write_config(tmp_path, PER_MSK_YAML))
    assert config.resolve_actuator_removals() == ["soleus_r"]

    config.actuator_removals = ["gastroc_r"]
    assert config.resolve_actuator_removals() == ["gastroc_r"]
    # An MSK-specific block still wins over the mutated default.
    assert config.resolve_actuator_removals("myolegs") == ["soleus_r", "soleus80_r"]


def test_per_msk_keyframe_overrides_merge_onto_default(tmp_path):
    """A per-MSK ``keyframe_overrides`` block patches the default rather than replacing it.

    Every other per-MSK section replaces. This one merges because it is already a patch: a
    lineage typically needs one joint of one pose changed (the 80-muscle and full-body knees
    flex positive where the myoLeg knee flexes negative), and under replace semantics that
    meant restating all five poses per MSK -- around sixty duplicated values across the
    shipped devices, every one free to drift.
    """
    yaml_text = """\
device:
  name: "TestDev"
  model_xml: "minimal_device.xml"

attachments:
  - device_body: "dev_a"
    parent_body: "pelvis"

keyframe_overrides:
  default:
    stand:
      pelvis_ty: 0.9
    lunge:
      pelvis_ty: 0.675
      knee_angle_l: -1.25
  myolegs:
    lunge:
      knee_angle_l: 1.25
"""
    config = DeviceConfig.from_yaml(_write_config(tmp_path, yaml_text))

    base = config.resolve_keyframe_overrides()
    assert base["lunge"].joint_values == {"pelvis_ty": 0.675, "knee_angle_l": -1.25}

    merged = config.resolve_keyframe_overrides("myolegs")
    # The knee is patched; pelvis_ty and the untouched 'stand' pose survive from default.
    assert merged["lunge"].joint_values == {"pelvis_ty": 0.675, "knee_angle_l": 1.25}
    assert merged["stand"].joint_values == {"pelvis_ty": 0.9}

    # An MSK with no block of its own still gets the default, unmodified.
    assert config.resolve_keyframe_overrides("myolegs26")["lunge"].joint_values["knee_angle_l"] == -1.25
    # And merging must not have mutated the default in place.
    assert config.resolve_keyframe_overrides()["lunge"].joint_values["knee_angle_l"] == -1.25
