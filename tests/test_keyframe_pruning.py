"""Tests for the joint index pre-walk used to slice keyframes by joint."""

from __future__ import annotations

import xml.etree.ElementTree as ET

import mujoco as mj
import pytest

from assist_sim import load_combined_model
from assist_sim.preprocess import (
    build_joint_index_table,
    inline_mujoco_includes,
    merge_top_level_duplicates,
    parse_keyframes,
)
from assist_sim.registry import resolve

from .conftest import needs_myo_sim

MSK_KEYS_22_26 = ["myoLeg22_2D", "myoLeg26_3D"]


@needs_myo_sim
@pytest.mark.parametrize("msk_key", MSK_KEYS_22_26)
def test_prewalk_matches_compiled_offsets(msk_key):
    """The document-order joint table must align with the compiled
    ``jnt_qposadr`` / ``jnt_dofadr`` arrays for every named joint."""
    path = str(resolve(msk_key, "DephyExoBoot_L1")[0])
    root = ET.parse(path).getroot()
    table = build_joint_index_table(root.find("worldbody"))
    model = mj.MjModel.from_xml_path(path)

    assert sum(j.qpos_width for j in table) == model.nq
    assert sum(j.qvel_width for j in table) == model.nv

    for j in table:
        if not j.name:
            continue
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, j.name)
        assert jid >= 0, f"joint {j.name} missing"
        assert int(model.jnt_qposadr[jid]) == j.qpos_start
        assert int(model.jnt_dofadr[jid]) == j.qvel_start


def test_keyframe_decomposition_by_joint(minimal_human):
    root = ET.parse(minimal_human).getroot()
    table = build_joint_index_table(root.find("worldbody"))
    keyframes = parse_keyframes(root, table)

    assert "home" in keyframes
    home = keyframes["home"]
    # qpos="0.9 0.1 -0.2 0.0 0.0 0.0 0.01" in document order
    assert home.qpos_by_joint["pelvis_ty"] == [0.9]
    assert home.qpos_by_joint["hip"] == [0.1]
    assert home.qpos_by_joint["knee"] == [-0.2]
    assert home.qpos_by_joint["pp_y"] == [0.01]


@needs_myo_sim
def test_include_inlining_merges_duplicate_worldbodies():
    """Regression: a terrain include with its own <worldbody> would shadow the
    MSK's worldbody.  ``merge_top_level_duplicates`` (called from
    ``inline_mujoco_includes``) must collapse them so the joint table builds
    correctly."""
    path = resolve("myoLeg22_2D", "DephyExoBoot_L1")[0]
    root = ET.parse(str(path)).getroot()
    inline_mujoco_includes(root, path.parent)
    assert len(root.findall("worldbody")) == 1
    table = build_joint_index_table(root.find("worldbody"))
    assert sum(j.qpos_width for j in table) > 0, "joint table empty after inline"


def test_merge_top_level_duplicates_idempotent():
    root = ET.fromstring(
        """<mujoco><worldbody><body name="a"/></worldbody>
                    <worldbody><body name="b"/></worldbody></mujoco>"""
    )
    merge_top_level_duplicates(root)
    assert len(root.findall("worldbody")) == 1
    names = [b.get("name") for b in root.find("worldbody").findall("body")]
    assert names == ["a", "b"]
    # Running again is a no-op.
    merge_top_level_duplicates(root)
    assert len(root.findall("worldbody")) == 1


@needs_myo_sim
def test_combined_keyframes_preserve_source_values():
    """End-to-end: a combined model's keyframes carry the MSK's authored
    joint values, modulo keyframe_overrides. Regression for the bug where
    only the override (pelvis_ty) survived and everything else was zero."""
    msk_path, device_path = resolve("myoLeg22_2D", "DephyExoBoot_L1")
    model, _ = load_combined_model(
        human_xml=str(msk_path),
        device_config=str(device_path),
    )
    kf = mj.mj_name2id(model, mj.mjtObj.mjOBJ_KEY, "stand")
    assert kf >= 0
    qpos = list(model.key_qpos[kf])

    # pelvis_ty has an override (0.96) in DephyExoBoot's L1config.yaml.
    pty = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pelvis_ty")
    assert qpos[int(model.jnt_qposadr[pty])] == pytest.approx(0.96)

    # Non-overridden joints carry their MSK-authored values.
    expected = {
        "knee_r_translation1": 0.00411,
        "knee_r_translation2": -0.395,
        "ankle_angle_r": -0.0143,
    }
    for name, value in expected.items():
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
        assert jid >= 0, f"joint {name} missing from compiled model"
        assert qpos[int(model.jnt_qposadr[jid])] == pytest.approx(value, abs=1e-6), (
            f"keyframe lost authored value for {name}"
        )


def test_widths_for_slide_and_hinge(minimal_human):
    root = ET.parse(minimal_human).getroot()
    table = {j.name: j for j in build_joint_index_table(root.find("worldbody"))}
    assert table["pelvis_ty"].qpos_width == 1  # slide
    assert table["hip"].qpos_width == 1  # hinge
    # document-order start indices
    assert table["pelvis_ty"].qpos_start == 0
    assert table["pp_x"].qpos_start == 5
    assert table["pp_y"].qpos_start == 6
