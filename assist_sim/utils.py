"""Utility functions for XML export, mesh deduplication, and post-processing."""

import os
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco as mj


def export_combined_xml(
    spec: mj.MjSpec,
    output_path: str,
    mesh_dirs: list[tuple[Path, str]] | list[Path] | None = None,
    terrain_paths: list[Path] | None = None,
) -> None:
    """Export a combined MjSpec to a clean XML file.

    Performs:
      - Default class deduplication
      - Mesh deduplication (same file path -> single mesh definition)
      - Mesh path resolution (honors compiler ``meshdir``) + rewriting relative
        to the output file location
      - Compiler ``meshdir`` / ``texturedir`` strip (paths are now absolute
        relative to the output file)
      - Terrain stripping: every element defined in the source MSK's terrain
        include (ground body, ground-plane geom, floor texture/material,
        hfield, terrain-referencing contact pairs) is removed from the export.
        assist_sim emits model-only XMLs; downstream consumers (e.g.
        ``myoassist.terrains``) layer the scene on top.
      - Minimal-visual fallback: if no ``<visual>`` block survives in the
        export, a small default is emitted so the model renders sensibly in
        a viewer.

    Args:
        spec: The combined MjSpec to export.
        output_path: Destination file path for the XML.
        mesh_dirs: A list of ``(modelfiledir, meshdir)`` tuples (or bare
            ``Path`` for legacy callers).  The model directory is the
            ``MjSpec.modelfiledir`` of each source spec; the meshdir is the
            value of ``<compiler meshdir="..."/>`` in that source.  Both are
            tried when resolving ``<mesh file="..."/>`` references.
        terrain_paths: Absolute paths to the terrain XML(s) the source MSK
            included.  Used to identify which named elements to strip from
            the export (texture / material / hfield / body / geom).  Not
            re-emitted as includes; assist_sim outputs are model-only.
    """
    output_path = Path(output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    xml_string = spec.to_xml()

    root = ET.fromstring(xml_string)
    # Hoist each attached fragment's unnamed "main" default into the root so
    # class-less elements (the leg muscle tendons + massless muscle-routing
    # joints, authored with no class) keep the rgba / armature / damping they
    # inherit in the live spec; then name any remaining nested unnamed blocks and
    # dedup repeated class names.
    _hoist_nested_defaults(root)
    _name_nested_defaults(root)
    _deduplicate_defaults(root)
    _deduplicate_meshes(root)
    if mesh_dirs:
        normalized = [md if isinstance(md, tuple) else (md, "") for md in mesh_dirs]
        _rewrite_mesh_paths(root, output_path.parent, normalized)
        _strip_resource_dirs(root)

    if terrain_paths:
        _strip_terrain(root, terrain_paths)
    _strip_orphan_scene_assets(root)
    _strip_scene_visual(root)
    _ensure_minimal_visual(root)

    final_xml = ET.tostring(root, encoding="unicode")
    output_path.write_text(final_xml, encoding="utf-8")


# ------------------------------------------------------------------
# Internal helpers
# ------------------------------------------------------------------


def _strip_terrain(
    root: ET.Element,
    terrain_paths: list[Path],
) -> None:
    """Remove every element contributed by a terrain include from the export.

    Driven by the terrain XML itself: every named element it declares
    (``<texture>`` / ``<material>`` / ``<hfield>`` / ``<body>`` / ``<geom>``
    / ``<site>``) is removed from *root* by matching ``(tag, name)`` tuples.
    Geoms nested inside a removed body subtree count too, so contact pairs
    referencing them are also scrubbed.  Any ``<include file="terrain_*"/>``
    directive surviving in the export is dropped.

    assist_sim's outputs are model-only; downstream consumers (e.g.
    ``myoassist.terrains``) layer the scene on top.  Nothing is re-emitted
    here.
    """
    if not terrain_paths:
        return

    # Textures + materials are intentionally NOT stripped: MuJoCo's renderer
    # requires at least one non-skybox texture bound to a material for the
    # skybox texture to render at all (without it, the viewer falls back to a
    # white clear color regardless of whether a skybox is defined).  Keeping
    # the terrain-config-derived 2D texture + its material around -- with no
    # geom referencing them -- is harmless and keeps the skybox visible.
    terrain_info: dict[str, set[str]] = {tag: set() for tag in ("hfield", "body", "geom", "site")}
    for tp in terrain_paths:
        if not tp.exists():
            continue
        t_root = ET.parse(str(tp)).getroot()
        for elem in t_root.iter():
            tag, name = elem.tag, elem.get("name")
            if tag in terrain_info and name:
                terrain_info[tag].add(name)

    # Collect elements to drop and the set of geom/body names being removed
    # (used to scrub contact pairs that would otherwise reference dangling ids).
    parent_map = {c: p for p in root.iter() for c in p}
    removed_geom_names: set[str] = set()
    to_remove: list[ET.Element] = []
    for elem in root.iter():
        nm = elem.get("name")
        if elem.tag in terrain_info and nm in terrain_info[elem.tag]:
            to_remove.append(elem)
            if elem.tag == "body":
                for g in elem.iter("geom"):
                    if g.get("name"):
                        removed_geom_names.add(g.get("name"))
            elif elem.tag == "geom":
                removed_geom_names.add(nm)

    removed_geom_names |= terrain_info["geom"]
    removed_body_names = terrain_info["body"]

    for elem in to_remove:
        parent = parent_map.get(elem)
        if parent is not None:
            parent.remove(elem)

    # Scrub contact pairs referencing removed geoms / bodies.
    for contact_root in root.findall("contact"):
        for pair in list(contact_root.findall("pair")):
            g1, g2 = pair.get("geom1"), pair.get("geom2")
            b1, b2 = pair.get("body1"), pair.get("body2")
            if g1 in removed_geom_names or g2 in removed_geom_names or b1 in removed_body_names or b2 in removed_body_names:
                contact_root.remove(pair)

    # Drop any pre-existing terrain <include> directives surviving the round-trip.
    for inc in list(root.findall("include")):
        f = inc.get("file", "")
        if Path(f).name.startswith("terrain_config"):
            root.remove(inc)


def strip_myosuite_scene_spec(spec) -> None:
    """Remove the bundled myosuite scene from a composed ``MjSpec`` in place.

    ``myo_sim.build_spec`` output carries the shared myosuite scene (floor
    plane, decorative backdrop mesh, pedestal cylinder, logo, scene lights and
    cameras).  assist_sim emits model-only models -- downstream consumers (e.g.
    ``myoassist.terrains``) layer the scene on top -- so this drops the scene
    while leaving the musculoskeletal model untouched.

    The model lives under worldbody child ``<body>`` subtrees; the scene
    contributes worldbody-*direct* geoms / lights / cameras (ground + backdrop),
    which are deleted structurally so unnamed scene primitives (e.g. the
    pedestal cylinder) are caught too.  Meshes no longer referenced by any
    surviving geom (the backdrop + logo meshes) are then deleted.  Requires
    ``mujoco>=3.3.4`` for ``MjSpec.delete``.
    """
    worldbody = spec.worldbody
    for collection in (worldbody.geoms, worldbody.lights, worldbody.cameras):
        for elem in list(collection):
            spec.delete(elem)

    referenced = {g.meshname for g in spec.geoms if g.meshname}
    for mesh in list(spec.meshes):
        if mesh.name not in referenced:
            spec.delete(mesh)


def _strip_scene_visual(root: ET.Element) -> None:
    """Remove scene-lighting styling from ``<visual>`` so a downstream scene owns it.

    myo_sim models carry the myosuite ``<visual>`` headlight + default camera
    (``<global>``).  assist_sim emits model-only models and expects the consumer
    (e.g. ``myoassist.terrains``) to supply lighting; leaving the myosuite
    headlight in place makes MuJoCo merge it -- per attribute -- with the
    downstream scene's ``<visual>``, so the myosuite ``ambient`` (0.5) survives
    and washes out the scene's intended lighting.  Drop ``<headlight>`` and
    ``<global>``; keep model-visualization settings (actuator ``<rgba>``,
    ``<scale>``, ``<map>``, ``<quality>``).  MuJoCo's built-in default headlight
    covers bare (scene-less) viewing.
    """
    visual = root.find("visual")
    if visual is None:
        return
    for tag in ("headlight", "global"):
        for elem in visual.findall(tag):
            visual.remove(elem)


def _ensure_minimal_visual(root: ET.Element) -> None:
    """Emit a baseline ``<visual>`` block if none survived the export.

    MSKs typically carry their own visual section (headlight, scale, haze)
    that passes through unchanged.  But if a model was authored without one
    -- or if the terrain strip removed the only visual block (e.g. when the
    skybox texture was the only ``<visual>`` content) -- this default keeps
    the model viewable in a MuJoCo viewer.
    """
    if root.find("visual") is not None:
        return
    visual = ET.Element("visual")
    ET.SubElement(
        visual,
        "headlight",
        {"diffuse": "0.6 0.6 0.6", "specular": "0.3 0.3 0.3", "ambient": "0.3 0.3 0.3"},
    )
    ET.SubElement(visual, "rgba", {"haze": "0.15 0.15 0.15 1"})
    ET.SubElement(visual, "scale", {"framelength": "0.5", "framewidth": "0.01"})
    # Insert near the top so it lands before bodies / assets.
    root.insert(0, visual)


def _rewrite_mesh_paths(
    root: ET.Element,
    output_dir: Path,
    mesh_dirs: list[tuple[Path, str]],
) -> None:
    """Rewrite mesh file paths so they resolve correctly from the output location.

    For each ``<mesh file="..."/>`` element, the original relative path is
    resolved against each candidate ``(modelfiledir, meshdir)`` pair until a
    file is found on disk.  Resolution mirrors MuJoCo's compiler: first
    ``modelfiledir / meshdir / file`` (when meshdir is set), then
    ``modelfiledir / file`` as a fallback for sources without meshdir.  The
    found absolute path is then rewritten as a path relative to *output_dir*.
    """
    asset_elem = root.find("asset")
    if asset_elem is None:
        return

    for mesh in asset_elem.findall("mesh"):
        rel = mesh.get("file")
        if not rel:
            continue

        resolved = _resolve_resource(rel, mesh_dirs)
        if resolved is None:
            continue

        try:
            new_rel = resolved.relative_to(output_dir)
        except ValueError:
            new_rel = Path(os.path.relpath(resolved, output_dir))

        mesh.set("file", str(new_rel).replace("\\", "/"))


def _resolve_resource(
    rel: str,
    mesh_dirs: list[tuple[Path, str]],
) -> Path | None:
    """Try (modelfiledir / meshdir / rel) then (modelfiledir / rel) for each pair."""
    for base, resource_dir in mesh_dirs:
        if resource_dir:
            candidate = (base / resource_dir / rel).resolve()
            if candidate.exists():
                return candidate
        candidate = (base / rel).resolve()
        if candidate.exists():
            return candidate
    return None


def _strip_resource_dirs(root: ET.Element) -> None:
    """Remove compiler ``meshdir`` / ``texturedir`` after path rewrite.

    The rewrite already produced paths relative to the output XML location;
    leaving meshdir in place would cause MuJoCo to re-prepend it on load.
    """
    compiler = root.find("compiler")
    if compiler is None:
        return
    for attr in ("meshdir", "texturedir"):
        compiler.attrib.pop(attr, None)


def _deduplicate_defaults(root: ET.Element) -> None:
    """Remove duplicate default class definitions (globally, by class name).

    Two things create repeated class names in the serialized default tree:
    multi-body ``attach_body`` re-creating a device's class tree, and myo_sim's
    composed ``to_xml`` re-emitting a fragment's classes once per attach (e.g.
    the right arm and its mirrored left arm both carry the ``myoarm*`` classes).
    MJCF class names are global, so any repeat is rejected on reload.  Keep the
    first occurrence of each name and drop later ones (with their subtree);
    elements referencing the name resolve to the survivor.  Where a duplicated
    class differed between copies (e.g. a mirrored side's collision-geom
    material), the first copy wins -- a cosmetic loss on the dropped side.
    """
    seen_classes: set[str] = set()

    def _dedup_children(parent: ET.Element) -> None:
        to_remove: list[ET.Element] = []
        for child in list(parent):
            if child.tag == "default":
                cls_name = child.get("class", "")
                # Unnamed <default> blocks are scope wrappers, never duplicates.
                if cls_name and cls_name in seen_classes:
                    to_remove.append(child)
                else:
                    if cls_name:
                        seen_classes.add(cls_name)
                    _dedup_children(child)
        for elem in to_remove:
            parent.remove(elem)

    default_root = root.find("default")
    if default_root is not None:
        _dedup_children(default_root)


def _hoist_nested_defaults(root: ET.Element) -> None:
    """Unwrap each unnamed nested ``<default>`` directly under the root default.

    myo_sim's composed ``to_xml`` emits every attached fragment's top (``main``)
    default as an UNNAMED ``<default>`` nested under the root.  In the live spec a
    fragment's class-less elements (e.g. the leg muscle tendons and the massless
    muscle-routing joints, authored with no ``class``) inherit their ``rgba`` /
    ``armature`` / ``damping`` from that block.  MuJoCo rejects an unnamed nested
    default on reload, and merely *naming* it severs that inheritance -- the
    class-less elements fall back to the (empty) root and lose their color and
    joint dynamics (grey muscles; NaN from zero-armature coupled joints).

    Hoisting fixes both: each unnamed block's element-level defaults (``<geom>`` /
    ``<joint>`` / ``<tendon>`` / ``<site>`` / ...) are merged into the root's
    corresponding element (attributes unioned, block wins), its named subclass
    children are re-parented onto the root, and the emptied block is removed.
    Class-less elements then inherit from the root exactly as they did in the live
    spec, named subclasses keep their inheritance chain, and the model
    round-trips.  (Class-less elements come from a single leg fragment in the
    composed models assist_sim exports, so merging is unambiguous.)
    """
    default_root = root.find("default")
    if default_root is None:
        return
    for block in list(default_root):
        if block.tag != "default" or block.get("class"):
            continue  # only unnamed nested <default> blocks (fragment "main"s)
        for child in list(block):
            if child.tag == "default":
                default_root.append(child)  # promote named subclass to the root
            else:
                existing = default_root.find(child.tag)
                if existing is None:
                    default_root.append(child)
                else:
                    existing.attrib.update(child.attrib)  # merge element-default
        default_root.remove(block)


def _name_nested_defaults(root: ET.Element) -> None:
    """Give every nested UNNAMED ``<default>`` a unique synthetic class name.

    myo_sim's composed-model ``to_xml`` can emit a ``<default>`` with no ``class``
    nested inside another ``<default>`` (fragment defaults merged under an
    anonymous scope).  MuJoCo rejects that on reload with "empty class name" -- a
    *nested* default must be named; only the single top-level default may be
    anonymous.

    Naming these blocks is lossless, where hoisting their children up is not:
    the inheritance chain (and each block's own element-level ``<geom>`` /
    ``<tendon>`` / ... defaults) is preserved intact, and multi-fragment models
    (``myofullbody``) don't collapse several blocks' element defaults into one
    parent (which violates MuJoCo's "one element per default" schema).  Since an
    unnamed default cannot be referenced by ``class=`` anyway, the synthetic name
    changes nothing referentially.  This makes torso'd / multi-fragment composed
    models round-trip.
    """
    counter = [0]

    def _name(default_elem: ET.Element, is_top: bool) -> None:
        if not is_top and not default_elem.get("class"):
            counter[0] += 1
            default_elem.set("class", f"_assist_default_{counter[0]}")
        for child in default_elem.findall("default"):
            _name(child, is_top=False)

    for top in root.findall("default"):
        _name(top, is_top=True)


def _strip_orphan_scene_assets(root: ET.Element) -> None:
    """Drop scene textures/materials orphaned by the model-only scene strip.

    ``strip_myosuite_scene_spec`` removes the myosuite scene's geoms but leaves
    its 2D textures + materials in the asset block.  Once ``texturedir`` is
    stripped on export their ``file="scene/*.png"`` paths dangle and the reload
    fails with "Error opening file".  Any material not referenced by a geom
    (directly or through a ``<default>`` geom class) and any non-skybox texture
    not referenced by a surviving material is scene residue -- drop it.  A
    ``skybox`` texture is kept regardless (it aids rendering and carries no file
    dependency in the composed models).
    """
    asset = root.find("asset")
    if asset is None:
        return

    referenced_materials: set[str] = {g.get("material") for g in root.iter("geom") if g.get("material")}
    for default in root.iter("default"):
        for geom in default.findall("geom"):
            if geom.get("material"):
                referenced_materials.add(geom.get("material"))

    referenced_textures: set[str] = {
        m.get("texture") for m in asset.findall("material") if m.get("name") in referenced_materials and m.get("texture")
    }

    for material in list(asset.findall("material")):
        if material.get("name") not in referenced_materials:
            asset.remove(material)
    for texture in list(asset.findall("texture")):
        if texture.get("type") == "skybox":
            continue
        if texture.get("name") not in referenced_textures:
            asset.remove(texture)


def _deduplicate_meshes(root: ET.Element) -> None:
    """Remove duplicate mesh definitions.

    Two ``<mesh>`` entries are duplicates only if they reference the same file
    *and* carry identical transform attributes.  Crucially, a mirrored mesh
    reuses its twin's file with ``scale="1 1 -1"`` (this is how myo_sim reflects
    the left side of a body) -- so the dedup key must include ``scale`` (and any
    other attribute), or the left mesh would be collapsed into the unreflected
    right one, silently destroying the mirror.  Genuine duplicates (the same
    device mesh re-added by multiple ``attach_body`` calls) share every
    attribute and still collapse.
    """
    asset_elem = root.find("asset")
    if asset_elem is None:
        return

    seen: dict[tuple, str] = {}  # full attribute signature -> first mesh name
    to_remove = []

    for mesh in asset_elem.findall("mesh"):
        file_path = mesh.get("file")
        if not file_path:
            continue

        attrs = {k: v for k, v in mesh.attrib.items() if k != "name"}
        attrs["file"] = file_path.lower().replace("\\", "/")
        sig = tuple(sorted(attrs.items()))
        first_name = seen.get(sig)

        if first_name is not None:
            dup_name = mesh.get("name")
            if dup_name and first_name != dup_name:
                for geom in root.iter("geom"):
                    if geom.get("mesh") == dup_name:
                        geom.set("mesh", first_name)
            to_remove.append(mesh)
        else:
            seen[sig] = mesh.get("name", "")

    for mesh in to_remove:
        asset_elem.remove(mesh)
