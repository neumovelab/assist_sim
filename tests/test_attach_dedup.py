"""Tests for delete-free multi-body attach and mesh dedup."""

from __future__ import annotations

import xml.etree.ElementTree as ET

import mujoco as mj

from assist_sim.config import DeviceConfig
from assist_sim.combine import ModelCombiner


def test_multibody_attach_dedups_shared_mesh(minimal_human, minimal_device_config, tmp_path):
    config = DeviceConfig.from_yaml(minimal_device_config)
    out = tmp_path / "combined.xml"
    model, data = ModelCombiner().combine(minimal_human, config, export_xml=str(out))

    # dev_a and dev_b both reference the one device mesh -> exactly one mesh entry
    assert model.nmesh == 1
    mesh_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_MESH, 0)
    assert mesh_name == "TestDev_dev_mesh"

    # both device bodies attached
    bodies = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)}
    assert {"TestDev_dev_a", "TestDev_dev_b"} <= bodies

    # exported XML has a single mesh asset, both geoms remap to it
    root = ET.parse(str(out)).getroot()
    meshes = [m.get("name") for m in root.iter("mesh")]
    assert meshes.count("TestDev_dev_mesh") == 1
    geom_meshes = {g.get("mesh") for g in root.iter("geom") if g.get("mesh")}
    assert geom_meshes == {"TestDev_dev_mesh"}


def test_device_actuator_joint_is_prefixed(minimal_human, minimal_device_config):
    config = DeviceConfig.from_yaml(minimal_device_config)
    model, _ = ModelCombiner().combine(minimal_human, config)
    act_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "dev_act")
    assert act_id >= 0
    # the actuator targets the prefixed device joint
    joint_id = int(model.actuator_trnid[act_id, 0])
    assert mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, joint_id) == "TestDev_dev_joint"


def test_no_stray_empty_keyframe(minimal_human, minimal_device_config):
    config = DeviceConfig.from_yaml(minimal_device_config)
    model, _ = ModelCombiner().combine(minimal_human, config)
    key_names = [mj.mj_id2name(model, mj.mjtObj.mjOBJ_KEY, i) for i in range(model.nkey)]
    assert key_names == ["home"]  # exactly one named key, no empty placeholder


def test_keyframe_override_applied(minimal_human, minimal_device_config):
    config = DeviceConfig.from_yaml(minimal_device_config)
    model, _ = ModelCombiner().combine(minimal_human, config)
    kid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_KEY, "home")
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "pelvis_ty")
    adr = int(model.jnt_qposadr[jid])
    assert abs(model.key_qpos[kid, adr] - 0.5) < 1e-9  # override pelvis_ty -> 0.5
