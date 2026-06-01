"""Standalone config validator (test-only).

``validate_config`` statically resolves every name reference in a
:class:`~pipeline.config.DeviceConfig` against the human MSK XML and the
device XML, returning a list of human-readable issues (empty = clean).  It
does not compile anything and is never called from ``combine()`` -- it exists
so test fixtures can fail fast when a config drifts from the models it targets.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Set

from .config import DeviceConfig
from .errors import closest_matches


@dataclass
class _Names:
    """Resolved name sets parsed from an XML model (no compile)."""
    bodies: Set[str] = field(default_factory=set)
    joints: Set[str] = field(default_factory=set)
    geoms: Set[str] = field(default_factory=set)
    sites: Set[str] = field(default_factory=set)
    actuators: Set[str] = field(default_factory=set)
    tendons: Set[str] = field(default_factory=set)
    keyframes: Set[str] = field(default_factory=set)
    meshes: Set[str] = field(default_factory=set)


def _section_names(root: ET.Element, section: str) -> Set[str]:
    elem = root.find(section)
    if elem is None:
        return set()
    return {c.get("name") for c in elem if c.get("name")}


def _collect_names(xml_path: str) -> _Names:
    root = ET.parse(str(xml_path)).getroot()
    names = _Names()
    names.bodies = {b.get("name") for b in root.iter("body") if b.get("name")}
    for tag in ("joint", "freejoint"):
        names.joints |= {j.get("name") for j in root.iter(tag) if j.get("name")}
    names.geoms = {g.get("name") for g in root.iter("geom") if g.get("name")}
    names.sites = {s.get("name") for s in root.iter("site") if s.get("name")}
    names.actuators = _section_names(root, "actuator")
    names.tendons = _section_names(root, "tendon")
    names.keyframes = _section_names(root, "keyframe")
    asset = root.find("asset")
    if asset is not None:
        names.meshes = {m.get("name") for m in asset.findall("mesh") if m.get("name")}
    return names


def _check(name: str, valid: Set[str], kind: str, section: str) -> Optional[str]:
    if name in valid:
        return None
    suggestions = closest_matches(name, valid)
    msg = f"{section}: unknown {kind} '{name}'"
    if suggestions:
        msg += " (did you mean " + ", ".join(f"'{s}'" for s in suggestions) + "?)"
    return msg


def validate_config(human_xml: str, config: DeviceConfig) -> List[str]:
    """Return every config reference that cannot be resolved; empty = clean.

    Args:
        human_xml: path to the baseline musculoskeletal model XML.
        config: the device config to validate against it.
    """
    issues: List[str] = []
    human = _collect_names(human_xml)
    device = _collect_names(str(config.model_xml_path))
    prefix = config.name + "_"

    # Device joints are namespaced once attached.
    device_joints_prefixed = {prefix + j for j in device.joints}

    def add(issue: Optional[str]) -> None:
        if issue:
            issues.append(issue)

    for name in config.body_removals:
        add(_check(name, human.bodies, "body", "body_removals"))
    for name in config.actuator_removals:
        add(_check(name, human.actuators, "actuator", "actuator_removals"))
    for name in config.tendon_removals:
        add(_check(name, human.tendons, "tendon", "tendon_removals"))

    for mod in config.tendon_modifications:
        add(_check(mod.name, human.tendons, "tendon", "tendon_modifications"))
        for edit in mod.wraps:
            if edit.op == "replace_site" and edit.new_body:
                add(
                    _check(
                        edit.new_body,
                        human.bodies,
                        "body",
                        f"tendon_modifications[{mod.name}].replace_site.new_body",
                    )
                )

    for mr in config.mesh_replacements:
        add(_check(mr.geom, human.geoms, "geom", "mesh_replacements"))
        add(_check(mr.mesh, device.meshes, "mesh", "mesh_replacements"))

    for att in config.attachments:
        add(_check(att.parent_body, human.bodies, "body", "attachments.parent_body"))
        add(
            _check(
                att.device_body, device.bodies, "device body", "attachments.device_body"
            )
        )

    for jo in config.joint_overrides:
        add(_check(jo.name, human.joints, "joint", "joint_overrides"))

    valid_act_joints = human.joints | device_joints_prefixed | device.joints
    for act in config.actuators:
        add(_check(act.joint, valid_act_joints, "joint", f"actuators[{act.name}].joint"))

    valid_kf_joints = human.joints | device_joints_prefixed | device.joints
    for kf_name, override in config.keyframe_overrides.items():
        add(_check(kf_name, human.keyframes, "keyframe", "keyframe_overrides"))
        for joint_name in override.joint_values:
            add(
                _check(
                    joint_name,
                    valid_kf_joints,
                    "joint",
                    f"keyframe_overrides[{kf_name}]",
                )
            )

    return issues
