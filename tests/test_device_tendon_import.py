"""Device-XML tendons are imported whole, not just their site wraps.

``MjSpec.attach_body`` copies a body subtree and leaves every top-level section behind, so
``ModelCombiner._import_device_tendons_actuators`` reads the device's ``<tendon>`` section
back out of the XML.  It used to handle ``<spatial>`` tendons with ``<site>`` wraps only:
a ``<fixed>`` tendon vanished, and a ``<pulley>`` or ``<geom>`` wrap inside a spatial
tendon was dropped in silence.  Both change the mechanics -- a pulley scales the length
contribution of the run after it, a geom wrap bends the path around a cylinder -- so the
device ended up with a tendon it did not author and no error was raised.

The test compares the combined model's wrap structure against the device XML compiled on
its own, which is the reference MuJoCo itself produces.
"""

from __future__ import annotations

import mujoco as mj
import pytest

from assist_sim import load_combined_model

HUMAN = """<mujoco model="h">
  <compiler angle="radian"/>
  <worldbody>
    <body name="thigh" pos="0 0 1">
      <joint name="hip" type="hinge" axis="0 1 0"/>
      <geom name="tg" type="capsule" fromto="0 0 0 0 0 -.4" size=".05" mass="5"/>
    </body>
  </worldbody>
</mujoco>
"""

# One tendon of every shape the importer must handle: a fixed tendon (joint wrap), a
# spatial tendon routed around a wrap geom with a side site, and a spatial tendon with a
# pulley. The pulley form is the one MuJoCo requires (site, site, pulley, site, site).
DEVICE = """<mujoco model="d">
  <compiler angle="radian"/>
  <worldbody>
    <body name="widget" pos="0 0 0">
      <joint name="wj" type="hinge" axis="0 1 0"/>
      <geom name="wg" type="cylinder" size=".01 .05" mass=".1"/>
      <site name="s1" pos="0 0 0"/>
      <site name="s2" pos="0 0 .05"/>
      <site name="s3" pos=".03 0 .05"/>
      <site name="side" pos=".02 0 .02"/>
    </body>
  </worldbody>
  <tendon>
    <fixed name="fixed_t">
      <joint joint="wj" coef="0.75"/>
    </fixed>
    <spatial name="geomwrap_t" width=".003">
      <site site="s1"/>
      <geom geom="wg" sidesite="side"/>
      <site site="s2"/>
    </spatial>
    <spatial name="pulley_t" width=".003">
      <site site="s1"/>
      <site site="s2"/>
      <pulley divisor="2"/>
      <site site="s3"/>
      <site site="s2"/>
    </spatial>
  </tendon>
</mujoco>
"""

CONFIG = """device:
  name: Dev
  model_xml: dev.xml
attachments:
  - device_body: widget
    parent_body: thigh
"""

PREFIX = "Dev_"


def _wrap_structure(model, strip: str = "") -> dict:
    """``{tendon name: [wrap kind, ...]}`` in routing order."""
    out = {}
    for i in range(model.ntendon):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_TENDON, i)
        adr, num = model.tendon_adr[i], model.tendon_num[i]
        kinds = [str(mj.mjtWrap(int(model.wrap_type[w]))).split("_")[-1] for w in range(adr, adr + num)]
        out[name.replace(strip, "", 1)] = kinds
    return out


@pytest.fixture
def combined(tmp_path):
    (tmp_path / "human.xml").write_text(HUMAN, encoding="utf-8")
    (tmp_path / "dev.xml").write_text(DEVICE, encoding="utf-8")
    (tmp_path / "L1config.yaml").write_text(CONFIG, encoding="utf-8")
    model, _ = load_combined_model(str(tmp_path / "human.xml"), str(tmp_path / "L1config.yaml"))
    reference = mj.MjModel.from_xml_path(str(tmp_path / "dev.xml"))
    return model, reference


def test_every_tendon_and_wrap_kind_survives_the_import(combined):
    """The combined model reproduces the device's own wrap structure exactly."""
    model, reference = combined
    assert _wrap_structure(model, PREFIX) == _wrap_structure(reference)


def test_fixed_tendon_is_imported(combined):
    """A ``<fixed>`` tendon used to be skipped entirely by a ``findall('spatial')``."""
    model, _ = combined
    assert mj.mj_name2id(model, mj.mjtObj.mjOBJ_TENDON, PREFIX + "fixed_t") >= 0


def test_geom_wrap_keeps_its_prefixed_side_site(combined):
    """The wrap geom's side site is device-local, so it must be prefixed like the rest."""
    model, _ = combined
    tid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_TENDON, PREFIX + "geomwrap_t")
    adr, num = model.tendon_adr[tid], model.tendon_num[tid]
    sidesites = [
        mj.mj_id2name(model, mj.mjtObj.mjOBJ_SITE, int(model.wrap_prm[w]))
        for w in range(adr, adr + num)
        if str(mj.mjtWrap(int(model.wrap_type[w]))).endswith("CYLINDER")
    ]
    assert sidesites == [PREFIX + "side"]


def test_pulley_divisor_is_carried_through(combined):
    """A dropped pulley silently rescaled the tendon; pin the divisor."""
    model, _ = combined
    tid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_TENDON, PREFIX + "pulley_t")
    adr, num = model.tendon_adr[tid], model.tendon_num[tid]
    divisors = [
        float(model.wrap_prm[w]) for w in range(adr, adr + num) if str(mj.mjtWrap(int(model.wrap_type[w]))).endswith("PULLEY")
    ]
    assert divisors == [2.0]


def test_unsupported_wrap_raises(tmp_path):
    """An unrecognised wrap child is an error, not a silent skip."""
    (tmp_path / "human.xml").write_text(HUMAN, encoding="utf-8")
    bad = DEVICE.replace(
        '<site site="s2"/>\n    </spatial>\n    <spatial name="pulley_t"',
        '<bogus x="1"/>\n    </spatial>\n    <spatial name="pulley_t"',
    )
    (tmp_path / "dev.xml").write_text(bad, encoding="utf-8")
    (tmp_path / "L1config.yaml").write_text(CONFIG, encoding="utf-8")
    with pytest.raises(ValueError, match="unsupported wrap"):
        load_combined_model(str(tmp_path / "human.xml"), str(tmp_path / "L1config.yaml"))


def test_unnamed_device_tendon_raises(tmp_path):
    """An unnamed tendon cannot be referenced once prefixed, so reject it."""
    (tmp_path / "human.xml").write_text(HUMAN, encoding="utf-8")
    (tmp_path / "dev.xml").write_text(DEVICE.replace('name="fixed_t"', ""), encoding="utf-8")
    (tmp_path / "L1config.yaml").write_text(CONFIG, encoding="utf-8")
    with pytest.raises(ValueError, match="no 'name'"):
        load_combined_model(str(tmp_path / "human.xml"), str(tmp_path / "L1config.yaml"))


def test_shipped_device_tendons_unchanged():
    """The bundled devices use site-only spatial tendons; their import must not shift.

    Humotech ships 8 cable tendons of two site wraps each -- the shape the old importer
    handled -- so this pins that the wider dispatch did not perturb them.
    """
    pytest.importorskip("myo_sim")
    from assist_sim import load_combined

    model, _ = load_combined("myolegs22", "Humotech_L1")
    cables = {n: k for n, k in _wrap_structure(model).items() if n.startswith("Humotech_L1_")}
    assert len(cables) == 8, cables
    assert all(kinds == ["SITE", "SITE"] for kinds in cables.values()), cables
