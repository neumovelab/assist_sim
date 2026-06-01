"""Tests for terrain stripping + minimal-visual fallback in the export step.

assist_sim emits *model-only* combined XMLs: anything contributed by a
terrain include is dropped. Downstream consumers (myoassist.terrains) layer
the scene on top. These tests verify the strip is exhaustive and that a
baseline ``<visual>`` block is emitted when no MSK-provided one survives.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from assist_sim.utils import _ensure_minimal_visual, _strip_terrain


def _write_fake_terrain(tmp_path: Path) -> Path:
    """A minimal terrain config mirroring the real one's element shapes."""
    p = tmp_path / "terrain_config.xml"
    p.write_text(
        """<mujocoinclude>
          <asset>
            <texture name="skybox" type="skybox" builtin="gradient" width="32" height="32"/>
            <texture name="texfloor" type="2d" builtin="flat" width="8" height="8"/>
            <material name="matfloor" texture="texfloor"/>
            <hfield name="terrain" size="1 1 0.1 0.01"/>
          </asset>
          <worldbody>
            <body name="ground" pos="0 0 0">
              <geom name="ground-plane" type="plane" size="5 5 0.1" material="matfloor"/>
            </body>
            <geom name="terrain" type="hfield" hfield="terrain"/>
          </worldbody>
        </mujocoinclude>""",
        encoding="utf-8",
    )
    return p


def test_strip_drops_all_terrain_named_elements(tmp_path):
    terrain = _write_fake_terrain(tmp_path)
    root = ET.fromstring(
        """<mujoco>
          <asset>
            <texture name="skybox" type="skybox"/>
            <texture name="texfloor" type="2d"/>
            <material name="matfloor" texture="texfloor"/>
            <hfield name="terrain" size="1 1 0.1 0.01"/>
            <mesh name="pelvis" file="pelvis.stl"/>
          </asset>
          <worldbody>
            <body name="ground"><geom name="ground-plane" type="plane"/></body>
            <body name="pelvis"><geom name="pelvis_geom" type="mesh" mesh="pelvis"/></body>
            <geom name="terrain" type="hfield" hfield="terrain"/>
          </worldbody>
        </mujoco>"""
    )
    _strip_terrain(root, [terrain])

    # All terrain-derived elements gone.
    assert root.find(".//body[@name='ground']") is None
    assert root.find(".//geom[@name='terrain']") is None
    assert root.find(".//hfield[@name='terrain']") is None
    # Textures + materials are intentionally PRESERVED -- MuJoCo's renderer
    # requires a texture+material binding for the skybox texture to render.
    # Leaving these around as orphans (no geom references them) is harmless.
    assert root.find(".//texture[@name='texfloor']") is not None
    assert root.find(".//texture[@name='skybox']") is not None
    assert root.find(".//material[@name='matfloor']") is not None

    # Model content survives.
    assert root.find(".//body[@name='pelvis']") is not None
    assert root.find(".//mesh[@name='pelvis']") is not None


def test_strip_scrubs_contact_pairs_referencing_removed_geoms(tmp_path):
    terrain = _write_fake_terrain(tmp_path)
    root = ET.fromstring(
        """<mujoco>
          <worldbody>
            <body name="ground"><geom name="ground-plane" type="plane"/></body>
            <body name="pelvis"><geom name="pelvis_geom" type="mesh"/></body>
          </worldbody>
          <contact>
            <pair geom1="ground-plane" geom2="pelvis_geom"/>
            <pair geom1="pelvis_geom" geom2="femur_geom"/>
            <pair geom1="terrain" geom2="pelvis_geom"/>
          </contact>
        </mujoco>"""
    )
    _strip_terrain(root, [terrain])

    pairs = root.find("contact").findall("pair")
    # Only the pelvis<->femur pair survives.
    assert len(pairs) == 1
    assert pairs[0].get("geom1") == "pelvis_geom"
    assert pairs[0].get("geom2") == "femur_geom"


def test_strip_drops_lingering_terrain_include_directives(tmp_path):
    terrain = _write_fake_terrain(tmp_path)
    root = ET.fromstring(
        """<mujoco>
          <include file="../terrain_config.xml"/>
          <include file="../../some_other.xml"/>
          <worldbody/>
        </mujoco>"""
    )
    _strip_terrain(root, [terrain])

    includes = [inc.get("file") for inc in root.findall("include")]
    assert "../terrain_config.xml" not in includes
    assert "../../some_other.xml" in includes  # non-terrain include untouched


def test_strip_noop_when_no_terrain_paths():
    root = ET.fromstring(
        """<mujoco><worldbody><body name="ground"/></worldbody></mujoco>"""
    )
    _strip_terrain(root, [])
    # Without terrain paths, nothing is stripped (the strip is driven by the
    # terrain XML's contents).
    assert root.find(".//body[@name='ground']") is not None


def test_ensure_minimal_visual_inserts_when_missing():
    root = ET.fromstring("<mujoco><worldbody/></mujoco>")
    _ensure_minimal_visual(root)
    visual = root.find("visual")
    assert visual is not None
    assert visual.find("headlight") is not None
    assert visual.find("scale") is not None


def test_ensure_minimal_visual_noop_when_present():
    root = ET.fromstring(
        """<mujoco>
          <visual><headlight diffuse="0.9 0.9 0.9"/></visual>
          <worldbody/>
        </mujoco>"""
    )
    _ensure_minimal_visual(root)
    visuals = root.findall("visual")
    # Existing visual untouched, no second one added.
    assert len(visuals) == 1
    assert visuals[0].find("headlight").get("diffuse") == "0.9 0.9 0.9"
