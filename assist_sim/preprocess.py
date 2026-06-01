"""XML preprocess layer.

All *removal* and *cascade* operations on the human musculoskeletal model
happen here, as an ``xml.etree.ElementTree`` pass that runs before
``MjSpec.from_file``.  This keeps the MjSpec phase purely additive (attach,
add actuator, edit attributes, rebuild keyframes), which is required to run
on ``mujoco==3.3.3`` where ``MjSpec.delete`` does not exist.

The original file on disk is never modified.  Temp files are written into the
same directory as the source so relative ``<include>`` and mesh ``file=``
paths still resolve.  Callers are responsible for cleanup.
"""

from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

from .config import DeviceConfig
from .errors import unknown_reference

_MAX_INCLUDE_DEPTH = 32

# qpos / qvel widths per joint type (matches MuJoCo compile-time layout).
_QPOS_WIDTH = {"free": 7, "ball": 4, "slide": 1, "hinge": 1}
_QVEL_WIDTH = {"free": 6, "ball": 3, "slide": 1, "hinge": 1}


@dataclass
class JointIndex:
    """A joint's position in the document-order qpos / qvel layout."""
    name: Optional[str]
    jtype: str
    qpos_start: int
    qpos_width: int
    qvel_start: int
    qvel_width: int


@dataclass
class KeyframeData:
    """A keyframe decomposed into per-joint qpos / qvel slices (by name)."""
    time: float
    qpos_by_joint: Dict[str, List[float]] = field(default_factory=dict)
    qvel_by_joint: Dict[str, List[float]] = field(default_factory=dict)


@dataclass
class PreprocessResult:
    """Output of :func:`preprocess_human_xml`."""
    path: str
    keyframes: Dict[str, KeyframeData]
    removed_joints: Set[str]
    removed_geoms: Set[str]
    removed_sites: Set[str]
    removed_bodies: Set[str]
    terrain_paths: List[Path] = field(default_factory=list)


# ----------------------------------------------------------------------
# ElementTree helpers
# ----------------------------------------------------------------------

def _build_parent_map(root: ET.Element) -> Dict[ET.Element, ET.Element]:
    """Map every element to its parent (ElementTree has no upward link)."""
    return {child: parent for parent in root.iter() for child in parent}


def _joint_type(joint_elem: ET.Element) -> str:
    if joint_elem.tag == "freejoint":
        return "free"
    return joint_elem.get("type", "hinge")


def build_joint_index_table(worldbody: ET.Element) -> List[JointIndex]:
    """Walk the worldbody depth-first, replicating MuJoCo's qpos/qvel order.

    Within a body, joints are indexed in document order; then child bodies
    are visited in document order.  This matches ``model.jnt_qposadr`` /
    ``model.jnt_dofadr`` of the compiled model (verified in tests).
    """
    table: List[JointIndex] = []
    state = {"qpos": 0, "qvel": 0}

    def walk(body: ET.Element) -> None:
        for child in body:
            if child.tag in ("joint", "freejoint"):
                jt = _joint_type(child)
                qw = _QPOS_WIDTH.get(jt, 1)
                vw = _QVEL_WIDTH.get(jt, 1)
                table.append(
                    JointIndex(
                        name=child.get("name"),
                        jtype=jt,
                        qpos_start=state["qpos"],
                        qpos_width=qw,
                        qvel_start=state["qvel"],
                        qvel_width=vw,
                    )
                )
                state["qpos"] += qw
                state["qvel"] += vw
        for child in body:
            if child.tag == "body":
                walk(child)

    for body in worldbody.findall("body"):
        walk(body)
    return table


def _parse_floats(text: Optional[str]) -> Optional[List[float]]:
    if not text:
        return None
    return [float(x) for x in text.split()]


def parse_keyframes(
    root: ET.Element,
    table: List[JointIndex],
) -> Dict[str, KeyframeData]:
    """Decompose each ``<key>`` into per-joint qpos / qvel slices by name.

    Uses the joint index table built from the *original* (pre-removal) tree
    so the slices line up with the keyframe arrays as authored.
    """
    keyframe_root = root.find("keyframe")
    result: Dict[str, KeyframeData] = {}
    if keyframe_root is None:
        return result

    for key in keyframe_root.findall("key"):
        name = key.get("name")
        if not name:
            continue  # empty-name keys are dropped (rebuilt explicitly)
        qpos = _parse_floats(key.get("qpos"))
        qvel = _parse_floats(key.get("qvel"))
        time = float(key.get("time", 0.0))

        data = KeyframeData(time=time)
        for j in table:
            if not j.name:
                continue
            if qpos is not None and j.qpos_start + j.qpos_width <= len(qpos):
                data.qpos_by_joint[j.name] = qpos[
                    j.qpos_start : j.qpos_start + j.qpos_width
                ]
            if qvel is not None and j.qvel_start + j.qvel_width <= len(qvel):
                data.qvel_by_joint[j.name] = qvel[
                    j.qvel_start : j.qvel_start + j.qvel_width
                ]
        result[name] = data
    return result


def _collect_terrain_include_paths(root: ET.Element, base_dir: Path) -> List[Path]:
    """Find every ``<include file="...terrain_config*.xml"/>`` reachable from
    *root* and return the resolved absolute paths.

    Heuristic: an include is "terrain" if its filename starts with
    ``terrain_config``. Used by the export step to strip inlined terrain
    content and re-emit a bare ``<include>`` directive instead.
    """
    paths: List[Path] = []
    for inc in root.iter("include"):
        f = inc.get("file", "")
        if not f:
            continue
        name = Path(f).name
        if name.startswith("terrain_config"):
            paths.append((base_dir / f).resolve())
    return paths


def inline_mujoco_includes(root: ET.Element, base_dir: Path, depth: int = 0) -> None:
    """Expand ``<include file="..."/>`` tags in-place (split chain/assets MSKs).

    MuJoCo merges the included file's children at the include site.  For
    ``mujocoinclude`` / ``mujoco`` roots we splice all direct children.  Relative
    paths resolve from *base_dir* (the directory of the file being processed).

    After splicing, calls :func:`merge_top_level_duplicates` on the root to
    mirror MuJoCo's compile-time merge of duplicate top-level sections (e.g.
    multiple ``<worldbody>`` / ``<asset>`` siblings produced by an included
    file).  Without this, ``root.find("worldbody")`` would return only the
    *first* section and silently drop everything after.
    """
    if depth > _MAX_INCLUDE_DEPTH:
        raise ValueError("MuJoCo include depth exceeded (possible cycle)")

    for child in list(root):
        if child.tag == "include":
            rel = child.get("file")
            if not rel:
                root.remove(child)
                continue
            inc_path = (base_dir / rel).resolve()
            if not inc_path.exists():
                raise FileNotFoundError(f"include not found: {inc_path}")
            inc_root = ET.parse(str(inc_path)).getroot()
            if inc_root.tag not in ("mujocoinclude", "mujoco"):
                raise ValueError(
                    f"unsupported include root <{inc_root.tag}> in {inc_path}"
                )
            idx = list(root).index(child)
            root.remove(child)
            for offset, imported in enumerate(list(inc_root)):
                inline_mujoco_includes(imported, inc_path.parent, depth + 1)
                root.insert(idx + offset, imported)
        else:
            inline_mujoco_includes(child, base_dir, depth + 1)

    if depth == 0:
        merge_top_level_duplicates(root)


# Top-level sections that MuJoCo merges by appending children.  Anything in this
# set may legally appear multiple times at the same depth after include inlining.
_MERGEABLE_SECTIONS = frozenset({
    "worldbody", "asset", "default", "contact", "sensor",
    "tendon", "actuator", "equality", "keyframe", "size", "custom",
})


def merge_top_level_duplicates(root: ET.Element) -> None:
    """Collapse duplicate top-level sections by appending their children.

    Called once at the root of the model after include inlining.  If two
    ``<worldbody>`` elements exist as siblings of ``<mujoco>``, the children
    of the second are appended to the first and the second is removed.
    Mirrors MuJoCo's compile-time section merging.
    """
    by_tag: Dict[str, ET.Element] = {}
    to_remove: List[ET.Element] = []
    for child in list(root):
        if child.tag in _MERGEABLE_SECTIONS:
            if child.tag in by_tag:
                first = by_tag[child.tag]
                for sub in list(child):
                    first.append(sub)
                to_remove.append(child)
            else:
                by_tag[child.tag] = child
    for elem in to_remove:
        root.remove(elem)


def _find_body(root: ET.Element, name: str) -> Optional[ET.Element]:
    for body in root.iter("body"):
        if body.get("name") == name:
            return body
    return None


def _find_named_child(parent: Optional[ET.Element], name: str) -> Optional[ET.Element]:
    if parent is None:
        return None
    for child in parent:
        if child.get("name") == name:
            return child
    return None


def _collect_descendants(
    body_elem: ET.Element,
) -> Tuple[Set[str], Set[str], Set[str], Set[str]]:
    """Recursively collect joint / geom / site / body names in a subtree."""
    joints: Set[str] = set()
    geoms: Set[str] = set()
    sites: Set[str] = set()
    bodies: Set[str] = set()

    def walk(b: ET.Element) -> None:
        if b.get("name"):
            bodies.add(b.get("name"))
        for child in b:
            tag = child.tag
            nm = child.get("name")
            if tag in ("joint", "freejoint") and nm:
                joints.add(nm)
            elif tag == "geom" and nm:
                geoms.add(nm)
            elif tag == "site" and nm:
                sites.add(nm)
            elif tag == "body":
                walk(child)

    walk(body_elem)
    return joints, geoms, sites, bodies


# ----------------------------------------------------------------------
# Removal / cascade passes
# ----------------------------------------------------------------------

def _all_body_names(root: ET.Element) -> List[str]:
    return [b.get("name") for b in root.iter("body") if b.get("name")]


def _remove_bodies(
    root: ET.Element,
    parent_map: Dict[ET.Element, ET.Element],
    body_names: List[str],
) -> Tuple[Set[str], Set[str], Set[str], Set[str]]:
    rj: Set[str] = set()
    rg: Set[str] = set()
    rs: Set[str] = set()
    rb: Set[str] = set()
    for name in body_names:
        elem = _find_body(root, name)
        if elem is None:
            raise unknown_reference(
                name, _all_body_names(root), section="body_removals", kind="body"
            )
        j, g, s, b = _collect_descendants(elem)
        rj |= j
        rg |= g
        rs |= s
        rb |= b
        parent_map[elem].remove(elem)
    return rj, rg, rs, rb


def _remove_geoms_by_name(root: ET.Element, names: List[str]) -> Set[str]:
    """Delete ``<geom name=X>`` elements with names in *names*.  Returns the
    set of names actually removed (for cascade cleanup of contact pairs).

    Used by ``geom_removals``: surgical removal of geoms that should not
    accompany a mesh-replaced parent body (e.g. dropping the separate fibula
    geom on transtibial amputation where the residual stump mesh covers both
    tibia and fibula).  Errors on unknown names with a suggestion.
    """
    if not names:
        return set()
    available: List[str] = [
        g.get("name") for g in root.iter("geom") if g.get("name")
    ]
    removed: Set[str] = set()
    parent_map = _build_parent_map(root)
    for name in names:
        elem = next(
            (g for g in root.iter("geom") if g.get("name") == name), None
        )
        if elem is None:
            raise unknown_reference(
                name, available, section="geom_removals", kind="geom"
            )
        parent_map[elem].remove(elem)
        removed.add(name)
    return removed


def _remove_named(
    section_root: Optional[ET.Element],
    names: List[str],
    *,
    section: str,
    kind: str,
) -> None:
    if not names:
        return
    available = (
        [c.get("name") for c in section_root if c.get("name")]
        if section_root is not None
        else []
    )
    for name in names:
        elem = _find_named_child(section_root, name)
        if elem is None:
            raise unknown_reference(name, available, section=section, kind=kind)
        section_root.remove(elem)


def _find_wrap_site(spatial: ET.Element, site_name: str) -> Optional[ET.Element]:
    for wrap in spatial:
        if wrap.tag == "site" and wrap.get("site") == site_name:
            return wrap
    return None


def _site_body_map(root: ET.Element) -> Tuple[Dict[str, ET.Element], Set[str]]:
    """Map every surviving site name to its parent body; collect all names."""
    site_to_body: Dict[str, ET.Element] = {}
    names: Set[str] = set()
    for parent in root.iter():
        for child in parent:
            if child.tag == "site" and child.get("name"):
                site_to_body[child.get("name")] = parent
                names.add(child.get("name"))
    return site_to_body, names


def _apply_tendon_modifications(
    root: ET.Element,
    tendon_root: Optional[ET.Element],
    modifications,
) -> None:
    """Apply WrapEdit ops (drop / reposition / replace) to spatial tendons.

    Synthesized sites are named ``{site}__mod`` and created on the relevant
    surviving body.  All other tendon attributes are preserved (edits are
    in place).  Runs after body removal so removed sites are already gone
    from the tree, but the wrap references still exist on the tendon.
    """
    if not modifications:
        return
    available_tendons = (
        [c.get("name") for c in tendon_root if c.get("name")]
        if tendon_root is not None
        else []
    )
    site_to_body, all_site_names = _site_body_map(root)

    for mod in modifications:
        spatial = _find_named_child(tendon_root, mod.name)
        if spatial is None:
            raise unknown_reference(
                mod.name,
                available_tendons,
                section="tendon_modifications",
                kind="tendon",
            )

        for edit in mod.wraps:
            wrap = _find_wrap_site(spatial, edit.site)
            wrap_sites = [w.get("site") for w in spatial if w.tag == "site"]

            if edit.op == "drop_site":
                if wrap is None:
                    raise unknown_reference(
                        edit.site,
                        wrap_sites,
                        section=f"tendon_modifications[{mod.name}].drop_site",
                        kind="wrap site",
                    )
                spatial.remove(wrap)
                continue

            if wrap is None:
                raise unknown_reference(
                    edit.site,
                    wrap_sites,
                    section=f"tendon_modifications[{mod.name}].{edit.op}",
                    kind="wrap site",
                )

            new_name = f"{edit.site}__mod"
            if new_name in all_site_names:
                raise ValueError(
                    f"tendon_modifications[{mod.name}]: synthesized site "
                    f"'{new_name}' already exists (collision)."
                )

            if edit.op == "reposition_site":
                target_body = site_to_body.get(edit.site)
                if target_body is None:
                    raise unknown_reference(
                        edit.site,
                        sorted(all_site_names),
                        section=f"tendon_modifications[{mod.name}].reposition_site",
                        kind="surviving site",
                    )
            else:  # replace_site
                target_body = _find_body(root, edit.new_body)
                if target_body is None:
                    raise unknown_reference(
                        edit.new_body,
                        _all_body_names(root),
                        section=f"tendon_modifications[{mod.name}].replace_site",
                        kind="body",
                    )

            pos_str = " ".join(str(v) for v in edit.pos)
            ET.SubElement(target_body, "site", {"name": new_name, "pos": pos_str})
            all_site_names.add(new_name)
            site_to_body[new_name] = target_body
            wrap.set("site", new_name)


def _auto_prune_wraps(
    tendon_root: Optional[ET.Element],
    removed_sites: Set[str],
    removed_geoms: Set[str],
) -> None:
    """Drop wrap sites/geoms that live on removed bodies, keeping all else."""
    if tendon_root is None or not (removed_sites or removed_geoms):
        return
    for spatial in tendon_root.findall("spatial"):
        for wrap in list(spatial):
            if wrap.tag == "site" and wrap.get("site") in removed_sites:
                spatial.remove(wrap)
            elif wrap.tag == "geom" and wrap.get("geom") in removed_geoms:
                spatial.remove(wrap)


def _cascade_cleanup(
    root: ET.Element,
    removed_joints: Set[str],
    removed_geoms: Set[str],
    removed_sites: Set[str],
    removed_bodies: Set[str],
) -> None:
    """Remove equality / contact-pair / sensor entries referencing removals."""
    joint_refs = removed_joints
    body_refs = removed_bodies

    equality = root.find("equality")
    if equality is not None:
        for e in list(equality):
            refs = {
                e.get("joint1"),
                e.get("joint2"),
            }
            body_ref = {e.get("body1"), e.get("body2")}
            site_ref = {e.get("site1"), e.get("site2")}
            if (refs & joint_refs) or (body_ref & body_refs) or (
                site_ref & removed_sites
            ):
                equality.remove(e)

    contact = root.find("contact")
    if contact is not None:
        for p in list(contact):
            if p.tag == "pair" and (
                p.get("geom1") in removed_geoms or p.get("geom2") in removed_geoms
            ):
                contact.remove(p)
            elif p.tag == "exclude" and (
                p.get("body1") in removed_bodies or p.get("body2") in removed_bodies
            ):
                contact.remove(p)

    sensor = root.find("sensor")
    if sensor is not None:
        for s in list(sensor):
            refs = {
                s.get("objname"),
                s.get("site"),
                s.get("joint"),
                s.get("geom"),
                s.get("body"),
            }
            if (refs & joint_refs) or (refs & removed_sites) or (
                refs & removed_geoms
            ) or (refs & removed_bodies):
                sensor.remove(s)


def _remove_keyframes(root: ET.Element) -> None:
    """Strip the entire ``<keyframe>`` section (rebuilt post-compile)."""
    for kf in root.findall("keyframe"):
        root.remove(kf)


def _write_temp_xml(root: ET.Element, src_path: Path, tag: str) -> str:
    """Serialize *root* to a temp XML next to *src_path*; return its path."""
    fd, tmp_path = tempfile.mkstemp(
        suffix=".xml", prefix=f"{src_path.stem}__{tag}_", dir=str(src_path.parent)
    )
    os.close(fd)
    ET.ElementTree(root).write(tmp_path, encoding="utf-8", xml_declaration=False)
    return tmp_path


# ----------------------------------------------------------------------
# Public entry points
# ----------------------------------------------------------------------

def preprocess_human_xml(
    human_xml: str,
    config: DeviceConfig,
    msk_key: Optional[str] = None,
) -> PreprocessResult:
    """Apply all removals/cascades to the human XML; return a temp file path.

    Args:
        human_xml: path to the baseline musculoskeletal model XML.
        config: the device config driving the removals.
        msk_key: optional MSK key for per-MSK config overrides (phase 11).

    Returns:
        A :class:`PreprocessResult` whose ``path`` is a temp XML (caller must
        delete) and whose ``keyframes`` are the parsed per-joint slices.
    """
    src = Path(human_xml).resolve()
    tree = ET.parse(str(src))
    root = tree.getroot()
    # Capture terrain include paths BEFORE inlining so we can re-emit them as
    # external <include> directives on export (keeps the exported XML small
    # and decoupled from the terrain package).
    terrain_paths = _collect_terrain_include_paths(root, src.parent)
    inline_mujoco_includes(root, src.parent)
    parent_map = _build_parent_map(root)

    worldbody = root.find("worldbody")
    table = build_joint_index_table(worldbody) if worldbody is not None else []
    keyframes = parse_keyframes(root, table)

    body_removals = config.resolve_body_removals(msk_key)
    actuator_removals = config.resolve_actuator_removals(msk_key)
    tendon_removals = config.resolve_tendon_removals(msk_key)
    tendon_modifications = config.resolve_tendon_modifications(msk_key)
    geom_removals = config.resolve_geom_removals(msk_key)

    rj, rg, rs, rb = _remove_bodies(root, parent_map, body_removals)
    rg |= _remove_geoms_by_name(root, geom_removals)

    _remove_named(
        root.find("actuator"),
        actuator_removals,
        section="actuator_removals",
        kind="actuator",
    )
    tendon_root = root.find("tendon")
    _remove_named(
        tendon_root, tendon_removals, section="tendon_removals", kind="tendon"
    )

    _apply_tendon_modifications(root, tendon_root, tendon_modifications)
    _auto_prune_wraps(tendon_root, rs, rg)

    _cascade_cleanup(root, rj, rg, rs, rb)
    _remove_keyframes(root)

    path = _write_temp_xml(root, src, "human_pp")
    return PreprocessResult(
        path=path,
        keyframes=keyframes,
        terrain_paths=terrain_paths,
        removed_joints=rj,
        removed_geoms=rg,
        removed_sites=rs,
        removed_bodies=rb,
    )


def prepare_device_xml(device_xml: str, strip_meshes: bool = False) -> str:
    """Write a temp copy of the device XML for attachment.

    Always strips ``<keyframe>`` (devices must not contribute keys; and we
    cannot delete them from the spec on 3.3.3).  When ``strip_meshes`` is
    True, also removes ``<mesh>`` assets -- used for every attach after the
    first so device meshes are not re-added (which would raise a duplicate
    name error on 3.3.3).
    """
    src = Path(device_xml).resolve()
    tree = ET.parse(str(src))
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    if compiler.get("angle") is None:
        compiler.set("angle", "radian")
    # Keep device meshes local when the human MSK sets a global meshdir (myoLeg80).
    if (src.parent / "mesh").is_dir():
        compiler.set("meshdir", str(src.parent))

    for kf in root.findall("keyframe"):
        root.remove(kf)

    if strip_meshes:
        asset = root.find("asset")
        if asset is not None:
            for mesh in list(asset.findall("mesh")):
                asset.remove(mesh)

    # Always strip device-side tendons and actuators from the attachment specs:
    # spanning tendons (e.g. HMEDI cable_r whose sites live on multiple device
    # bodies that get attached to different human bodies) are not handled by
    # ``MjSpec.attach_body``'s subtree-scoped migration, and partial migration
    # would raise ``repeated name`` on subsequent attaches.  ``ModelCombiner``
    # imports them from the source XML once, after all attachments complete.
    for section in ("tendon", "actuator"):
        for elem in list(root.findall(section)):
            root.remove(elem)

    tag = "dev_nomesh" if strip_meshes else "dev_full"
    return _write_temp_xml(root, src, tag)
