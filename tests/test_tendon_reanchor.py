"""Surgical re-anchoring: ``tendon_modifications`` applied before the removals.

Pins the four wrap-edit ops, the muscle survival they exist to produce, and the
``ctrl`` ordering guarantee.
"""

from __future__ import annotations

import mujoco as mj
import pytest

from assist_sim import load_combined
from assist_sim.combine import ModelCombiner
from assist_sim.config import DeviceConfig, TendonModification, WrapEdit

from .conftest import needs_myo_sim

pytest.importorskip("myo_sim")

from assist_sim.registry import _resolve_msk  # noqa: E402


def _owner(model, kind, name):
    """Name of the body that owns a site or geom in the compiled model."""
    obj = mj.mjtObj.mjOBJ_SITE if kind == "site" else mj.mjtObj.mjOBJ_GEOM
    oid = mj.mj_name2id(model, obj, name)
    owner = model.site_bodyid[oid] if kind == "site" else model.geom_bodyid[oid]
    return mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, owner)


def _apply(spec, mods):
    """Run only the re-anchor stage, with a config carrying just these mods."""
    config = DeviceConfig.from_yaml("assist_sim/models/OpenSourceLeg/KA_L1config.yaml")
    config.tendon_modifications = mods
    config._tendon_modifications_by_msk = {"default": mods}
    ModelCombiner._apply_tendon_modifications(spec, config, msk_key=None)
    return spec


@needs_myo_sim
def test_replace_site_moves_the_wrap_to_the_new_body():
    """The wrap resolves its site by name, so renaming the replacement moves it."""
    spec = _resolve_msk("myolegs26")
    assert _owner(spec.compile(), "site", "rect_fem_r_rect_fem_r-P3") != "femur_r"

    _apply(
        spec,
        [
            TendonModification(
                name="rect_fem_r_tendon",
                wraps=[
                    WrapEdit(
                        op="replace_site",
                        target="rect_fem_r_rect_fem_r-P3",
                        new_body="femur_r",
                        pos=[0.025, -0.275, 0.0075],
                    )
                ],
            )
        ],
    )
    model = spec.compile()
    assert _owner(model, "site", "rect_fem_r_rect_fem_r-P3") == "femur_r"
    # The tendon still wraps through it -- the edit did not orphan the path.
    tid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_TENDON, "rect_fem_r_tendon")
    adr, num = model.tendon_adr[tid], model.tendon_num[tid]
    wrapped = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_SITE, model.wrap_objid[w]) for w in range(adr, adr + num)}
    assert "rect_fem_r_rect_fem_r-P3" in wrapped


@needs_myo_sim
def test_reposition_site_moves_in_place_and_changes_tendon_length():
    spec = _resolve_msk("myolegs26")
    before = spec.compile()
    tid = mj.mj_name2id(before, mj.mjtObj.mjOBJ_TENDON, "rect_fem_r_tendon")
    length_before = before.tendon_length0[tid]

    _apply(
        spec,
        [
            TendonModification(
                name="rect_fem_r_tendon",
                wraps=[WrapEdit(op="reposition_site", target="rect_fem_r_rect_fem_r-P2", pos=[0.045, -0.2, 0.005])],
            )
        ],
    )
    after = spec.compile()
    assert _owner(after, "site", "rect_fem_r_rect_fem_r-P2") == "femur_r"  # unchanged owner
    assert after.tendon_length0[tid] != pytest.approx(length_before)


@needs_myo_sim
def test_replace_geom_moves_a_wrap_cylinder():
    """The 80-muscle hamstrings cross condylar wrap cylinders, not just sites."""
    spec = _resolve_msk("myolegs")
    assert _owner(spec.compile(), "geom", "SM_at_condyles_wrap_r") == "tibia_r"

    _apply(
        spec,
        [
            TendonModification(
                name="semimem_r_tendon",
                wraps=[
                    WrapEdit(
                        op="replace_geom",
                        target="SM_at_condyles_wrap_r",
                        new_body="femur_r",
                        pos=[0.01464, -0.27, 0.00916],
                    )
                ],
            )
        ],
    )
    assert _owner(spec.compile(), "geom", "SM_at_condyles_wrap_r") == "femur_r"


@needs_myo_sim
def test_a_wrap_geom_left_behind_still_cascades_the_tendon():
    """Moving only the sites is not enough: the cylinder anchors the tendon too.
    This is the failure mode that makes ``replace_geom`` necessary."""
    spec = _resolve_msk("myolegs")
    _apply(
        spec,
        [
            TendonModification(
                name="semimem_r_tendon",
                wraps=[
                    WrapEdit(op="replace_site", target="semimem-P2_r", new_body="femur_r", pos=[0.013, -0.283, 0.012]),
                ],
            )
        ],
    )
    spec.delete(spec.body("tibia_r"))
    assert spec.tendon("semimem_r_tendon") is None
    assert spec.actuator("semimem_r") is None


@needs_myo_sim
@pytest.mark.parametrize(
    "msk,muscles",
    [
        ("myolegs26", ["rectfem_r", "hamstrings_r"]),
        ("myolegs22", ["rectfem_r", "hamstrings_r"]),
        ("myolegs", ["recfem_r", "semimem_r", "semiten_r", "grac_r", "sart_r", "tfl_r", "bflh_r"]),
    ],
)
def test_biarticular_muscles_survive_transfemoral_amputation(msk, muscles):
    """Before re-anchoring ran ahead of the removals, the cascade destroyed all
    of these, leaving the residual hip with one flexor and one extensor."""
    model, _ = load_combined(msk, "OpenSourceLeg_KA_L1")
    names = {model.actuator(i).name for i in range(model.nu)}
    missing = [m for m in muscles if m not in names]
    assert not missing, f"{msk}: re-anchored muscles lost to the cascade: {missing}"


@needs_myo_sim
def test_gastroc_survives_transtibial_amputation():
    model, _ = load_combined("myolegs26", "OpenSourceLeg_A_L1")
    names = {model.actuator(i).name for i in range(model.nu)}
    assert "gastroc_r" in names


@needs_myo_sim
@pytest.mark.parametrize(
    "msk,device,muscles",
    [
        ("myolegs26", "OpenSourceLeg_KA_L1", ["rectfem_r", "hamstrings_r"]),
        (
            "myolegs",
            "OpenSourceLeg_KA_L1",
            ["recfem_r", "semimem_r", "semiten_r", "grac_r", "sart_r", "tfl_r", "bflh_r", "addmagIsch_r"],
        ),
        ("myolegs26", "OpenSourceLeg_A_L1", ["gastroc_r"]),
        ("myolegs", "OpenSourceLeg_A_L1", ["gasmed_r", "gaslat_r"]),
    ],
)
def test_reanchoring_preserves_the_dominant_moment_arm(msk, device, muscles):
    """A re-anchored muscle must still act the same way at the joint it keeps.

    Moment arm is ``-ten_J``.  For each muscle, take the joint with the largest
    intact arm *among the joints the amputation keeps*, since the largest arm
    overall usually belongs to the joint that surgery removes.  That joint must
    still carry the largest arm, with the same sign and a similar magnitude.
    A clean compile does not prove this.

    Only the dominant action is pinned.  Hip rotation arms are 0.5 mm to 5 mm
    against 40 mm to 55 mm primary arms, and four of them change sign; see
    AMPUTEE_PIPELINE_AUDIT.md Part II section G.
    """

    def arms(model, data, tendon):
        tid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_TENDON, tendon)
        mj.mj_forward(model, data)
        out = {}
        for j in range(model.njnt):
            name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, j)
            if name and name.endswith("_r"):
                out[name] = -float(data.ten_J[tid, model.jnt_dofadr[j]])
        return out

    baseline = _resolve_msk(msk).compile()
    base_data = mj.MjData(baseline)
    model, data = load_combined(msk, device)

    for muscle in muscles:
        aid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, muscle)
        tendon = mj.mj_id2name(model, mj.mjtObj.mjOBJ_TENDON, int(model.actuator_trnid[aid][0]))
        before, after = arms(baseline, base_data, tendon), arms(model, data, tendon)

        surviving = [j for j in before if j in after]
        joint = max(surviving, key=lambda j: abs(before[j]))
        assert before[joint] * after[joint] > 0, (
            f"{muscle}: dominant action at {joint} changed sign, {before[joint]:+.4f} -> {after[joint]:+.4f}"
        )
        ratio = after[joint] / before[joint]
        assert 0.6 < ratio < 1.6, f"{muscle}: {joint} moment arm scaled by {ratio:.2f}"
        assert joint == max(after, key=lambda j: abs(after[j])), (
            f"{muscle}: {joint} is no longer the dominant action after surgery"
        )


@needs_myo_sim
def test_reanchoring_preserves_actuator_order():
    """Rebuilding tendons would move each actuator to the end of the list,
    silently permuting ``ctrl`` indices for a policy trained on the baseline."""
    baseline = _resolve_msk("myolegs26").compile()
    order = [baseline.actuator(i).name for i in range(baseline.nu)]

    model, _ = load_combined("myolegs26", "OpenSourceLeg_KA_L1")
    survivors = [model.actuator(i).name for i in range(model.nu)]
    muscles = [a for a in survivors if a in set(order)]
    assert muscles == [a for a in order if a in set(muscles)]


@needs_myo_sim
def test_reanchored_muscle_starts_inside_its_operating_range():
    """Without ``actuator_overrides``, rectfem_r's authored range [0.321, 0.510]
    does not contain its own rest length after surgery."""
    for msk, muscle in (("myolegs26", "rectfem_r"), ("myolegs", "recfem_r")):
        model, data = load_combined(msk, "OpenSourceLeg_KA_L1")
        mj.mj_forward(model, data)
        aid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, muscle)
        lo, hi = model.actuator_lengthrange[aid]
        rest = data.ten_length[model.actuator_trnid[aid][0]]
        assert lo <= rest <= hi, f"{msk}/{muscle}: rest length {rest} outside [{lo}, {hi}]"


@needs_myo_sim
def test_transfemoral_prosthetic_limb_is_lighter_than_the_intact_one():
    """Without the femur ``body_overrides`` the prosthetic side came out heavier."""

    def limb_mass(model, root):
        rid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, root)
        ids, total = {rid}, 0.0
        for b in range(rid, model.nbody):
            if b == rid or model.body_parentid[b] in ids:
                ids.add(b)
                total += model.body_mass[b]
        return total

    for msk in ("myolegs26", "myolegs"):
        model, _ = load_combined(msk, "OpenSourceLeg_KA_L1")
        assert limb_mass(model, "femur_r") < limb_mass(model, "femur_l")


@needs_myo_sim
def test_patella_is_not_left_floating_after_transfemoral_amputation():
    """patella_r is a sibling of tibia_r, so the cascade misses it."""
    for msk in ("myolegs", "myofullbody"):
        model, _ = load_combined(msk, "OpenSourceLeg_KA_L1")
        assert mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "patella_r") < 0


def test_retired_drop_site_raises_with_a_reason():
    from assist_sim.config import _parse_wrap_edit

    with pytest.raises(ValueError, match="no longer supported.*editable wrap list"):
        _parse_wrap_edit({"drop_site": "some_site"})


def test_replace_ops_require_a_new_body_and_pos():
    from assist_sim.config import _parse_wrap_edit

    with pytest.raises(ValueError, match="requires 'new_body'"):
        _parse_wrap_edit({"replace_site": "s", "pos": [0, 0, 0]})
    with pytest.raises(ValueError, match="requires 'pos'"):
        _parse_wrap_edit({"replace_geom": "g", "new_body": "femur_r"})


def test_wrap_edit_kind_dispatches_on_the_op():
    assert WrapEdit(op="replace_site", target="s").kind == "site"
    assert WrapEdit(op="reposition_site", target="s").kind == "site"
    assert WrapEdit(op="replace_geom", target="g").kind == "geom"
    assert WrapEdit(op="reposition_geom", target="g").kind == "geom"


@needs_myo_sim
def test_unknown_tendon_and_site_are_reported_with_the_section():
    spec = _resolve_msk("myolegs26")
    with pytest.raises(ValueError, match="tendon_modifications"):
        _apply(spec, [TendonModification(name="not_a_tendon", wraps=[])])

    spec = _resolve_msk("myolegs26")
    with pytest.raises(ValueError, match="tendon_modifications"):
        _apply(
            spec,
            [
                TendonModification(
                    name="rect_fem_r_tendon",
                    wraps=[WrapEdit(op="reposition_site", target="not_a_site", pos=[0, 0, 0])],
                )
            ],
        )
