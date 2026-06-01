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
    _deduplicate_defaults(root)
    _deduplicate_meshes(root)
    if mesh_dirs:
        normalized = [
            md if isinstance(md, tuple) else (md, "") for md in mesh_dirs
        ]
        _rewrite_mesh_paths(root, output_path.parent, normalized)
        _strip_resource_dirs(root)

    if terrain_paths:
        _strip_terrain(root, terrain_paths)
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
    terrain_info: dict[str, set[str]] = {
        tag: set()
        for tag in ("hfield", "body", "geom", "site")
    }
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
            if (
                g1 in removed_geom_names
                or g2 in removed_geom_names
                or b1 in removed_body_names
                or b2 in removed_body_names
            ):
                contact_root.remove(pair)

    # Drop any pre-existing terrain <include> directives surviving the round-trip.
    for inc in list(root.findall("include")):
        f = inc.get("file", "")
        if Path(f).name.startswith("terrain_config"):
            root.remove(inc)


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
    """Remove duplicate default class definitions created by multi-body attach.

    When multiple bodies are attached from the same device spec, each
    attach_body call re-creates the device's default class tree.  This
    function keeps only the first occurrence of each class name.
    """
    def _dedup_children(parent: ET.Element) -> None:
        seen_classes: set[str] = set()
        to_remove: list[ET.Element] = []

        for child in list(parent):
            if child.tag == "default":
                cls_name = child.get("class", "")
                if cls_name in seen_classes:
                    to_remove.append(child)
                else:
                    seen_classes.add(cls_name)
                    _dedup_children(child)

        for elem in to_remove:
            parent.remove(elem)

    default_root = root.find("default")
    if default_root is not None:
        _dedup_children(default_root)


def _deduplicate_meshes(root: ET.Element) -> None:
    """Remove duplicate mesh definitions that reference the same file."""
    asset_elem = root.find("asset")
    if asset_elem is None:
        return

    seen_files: dict[str, str] = {}  # normalized_path -> first mesh name
    to_remove = []

    for mesh in asset_elem.findall("mesh"):
        file_path = mesh.get("file")
        if not file_path:
            continue

        key = file_path.lower().replace("\\", "/")
        first_name = seen_files.get(key)

        if first_name is not None:
            dup_name = mesh.get("name")
            if dup_name and first_name != dup_name:
                for geom in root.iter("geom"):
                    if geom.get("mesh") == dup_name:
                        geom.set("mesh", first_name)
            to_remove.append(mesh)
        else:
            seen_files[key] = mesh.get("name", "")

    for mesh in to_remove:
        asset_elem.remove(mesh)


