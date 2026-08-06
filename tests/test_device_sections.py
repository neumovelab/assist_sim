"""Coverage for the device-config sections added for the NEU environments.

``MjSpec.attach_body`` copies a body subtree and the assets it references; every
top-level MJCF section stays behind.  Tendons and tendon-driven actuators are
read back out of the device XML by the combiner, but ``<equality>``,
``<contact>`` and ``<sensor>`` are not -- they are authored in YAML against the
*combined* model, where device names carry the device prefix and human names are
bare.  Each test below pins one of those paths, plus the two inertial/mass
sections (``body_overrides``, ``sensor_removals``) and the actuator force-limit
carry-through.

``NEUankle_L1`` (powered transtibial prosthesis) and ``STRIDE_L2`` (closed-chain
cable ankle exo) are the two devices that exercise all of it.
"""

from __future__ import annotations

import mujoco as mj
import numpy as np
import pytest

from assist_sim import load_combined
from assist_sim.config import DeviceConfig

from .conftest import needs_myo_sim

STRIDE = "STRIDE_L2_"
NEU = "NEUankle_L1_"

# The five quartics that close one side of the STRIDE linkage, master joint last.
# Duplicated from the config on purpose: the test is meant to fail if the config's
# coefficients change without someone re-checking the kinematics.
_AG = [2.06729283e-02, -8.75475095e-01, 6.11187224e-02, 3.43951230e-02, 1.08428608e-01]
_CHAIN = {
    "shank_r__link_bcd_r": [3.24554488e-05, 1.12958661e00, -2.34045106e-02, -2.01019565e-02, 8.60299503e-02],
    "link_ag_r__link_gcf_r": [-5.54206730e-05, -7.95143663e-01, -1.29857805e-01, 1.30052004e-01, -2.08304297e-01],
    "link_bcd_r__link_de_r": [-4.43707369e-05, -7.75739561e-01, -9.82875335e-02, 1.14068059e-01, -1.66046221e-01],
    "link_gcf_r__foot_r": [2.39048948e-05, 9.44849648e-01, 1.92238275e-02, -7.68532585e-02, 1.08527030e-01],
}


def _poly(coef, x):
    return sum(a * x**i for i, a in enumerate(coef))


def _qadr(model, name):
    return model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)]


# ----------------------------------------------------------------------
# body_overrides
# ----------------------------------------------------------------------


@needs_myo_sim
@pytest.mark.parametrize("device", ["KFoot_L1", "OpenSourceLeg_A_L1", "NEUankle_L1"])
@pytest.mark.parametrize("msk", ["myolegs22", "myolegs26", "myolegs", "myofullbody"])
def test_transtibial_residual_tibia_mass(msk, device):
    """Every transtibial device reduces tibia_r to the residuum.

    Without ``body_overrides`` the amputated side keeps the *intact* shank's mass
    (3.7075 kg on the 26/22-muscle lineage, 3.80 kg on the 80-muscle one), because
    ``spec.delete`` removes the distal subtree but cannot know the segment was
    transected.  All three devices share the same residuum, so the intact side is
    the control: it must still differ.
    """
    model, _ = load_combined(msk, device)
    assert float(model.body("tibia_r").mass[0]) == pytest.approx(1.85375)
    np.testing.assert_allclose(model.body("tibia_r").inertia, [0.0125, 0.0125, 0.00225], rtol=1e-9)
    assert float(model.body("tibia_l").mass[0]) > 3.0  # intact side untouched


@needs_myo_sim
def test_body_override_needs_inertia_when_body_has_none(models_dir, monkeypatch):
    """Setting mass on a compiler-derived-inertia body raises rather than zeroing it.

    MuJoCo derives a body's inertia from its geoms unless the body is marked
    ``explicitinertial``.  Flipping that flag while supplying only a mass would
    silently leave the inertia at zero, so the combiner refuses.
    """
    from assist_sim.combine import ModelCombiner
    from assist_sim.config import BodyOverride

    spec = mj.MjSpec()
    body = spec.worldbody.add_body(name="plain")
    body.add_geom(type=mj.mjtGeom.mjGEOM_SPHERE, size=[0.1, 0, 0])
    assert not body.explicitinertial

    config = DeviceConfig.from_yaml(str(models_dir / "NEUankle" / "L1config.yaml"))
    config.body_overrides = [BodyOverride(name="plain", mass=1.0)]
    config._body_overrides_by_msk = {"default": config.body_overrides}

    with pytest.raises(ValueError, match="derives from its geoms"):
        ModelCombiner._apply_body_overrides(spec, config, prefix="X_", msk_key=None)


# ----------------------------------------------------------------------
# equality: type joint
# ----------------------------------------------------------------------


@needs_myo_sim
@pytest.mark.parametrize("msk", ["myolegs26", "myofullbody"])
def test_stride_joint_couplings_present_and_mixed(msk):
    """Ten joint couplings land, and the exo-to-leg pair spans device and human.

    Four per side close the six-bar loop; the fifth ties the master hinge to the
    biological ankle and is the *only* connection between the exo and the leg
    below the shank cuff -- the foot plate is deliberately not welded to the shoe.
    """
    model, _ = load_combined(msk, "STRIDE_L2")
    couplings = []
    for e in range(model.neq):
        if model.eq_type[e] != mj.mjtEq.mjEQ_JOINT:
            continue
        n1 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, model.eq_obj1id[e]) or ""
        n2 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, model.eq_obj2id[e]) or ""
        if STRIDE in n1 or STRIDE in n2:
            couplings.append((n1, n2))
    assert len(couplings) == 10

    # The exo-to-ankle coupling: device joint (prefixed) -> human joint (bare).
    for side in ("r", "l"):
        pair = (f"{STRIDE}shank_{side}__link_ag_{side}", f"ankle_angle_{side}")
        assert pair in couplings, f"missing exo-to-leg coupling for {side}"


@needs_myo_sim
@pytest.mark.parametrize("msk", ["myolegs26", "myofullbody"])
def test_stride_polycoef_reaches_eq_data(msk):
    """The authored quartics survive into ``eq_data[0:5]`` in order."""
    model, _ = load_combined(msk, "STRIDE_L2")
    for joint, coef in _CHAIN.items():
        found = None
        for e in range(model.neq):
            if mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, model.eq_obj1id[e]) == STRIDE + joint:
                found = model.eq_data[e][:5]
        assert found is not None, f"no coupling on {joint}"
        np.testing.assert_allclose(found, coef, rtol=1e-9)


@needs_myo_sim
@pytest.mark.parametrize("msk", ["myolegs26", "myofullbody"])
def test_stride_plate_tracks_the_sole(msk):
    """The foot plate rotates in lockstep with the shoe sole across the ankle ROM.

    Posing the linkage straight from the quartics (no solver in the loop) isolates
    the geometry: if the exo hinge is coaxial with the ankle, the plate-to-sole
    relative rotation is *constant*, and any axis mismatch shows up as drift.  On
    the 80-muscle lineage the ankle is the anatomical talocrural axis, 11.7 deg out
    of sagittal, so this only holds because the attachment frame realigns the
    device onto it.
    """
    model, data = load_combined(msk, "STRIDE_L2")
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "ankle_angle_r")
    plate = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, STRIDE + "foot_r")
    sole = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, STRIDE + "r_aft_sole")

    angles = []
    for q in np.linspace(*model.jnt_range[jid], 9):
        data.qpos[:] = model.qpos0
        data.qvel[:] = 0
        data.qpos[_qadr(model, "ankle_angle_r")] = q
        ag = _poly(_AG, q)
        data.qpos[_qadr(model, STRIDE + "shank_r__link_ag_r")] = ag
        for joint, coef in _CHAIN.items():
            data.qpos[_qadr(model, STRIDE + joint)] = _poly(coef, ag)
        mj.mj_forward(model, data)
        rel = data.xmat[sole].reshape(3, 3).T @ data.xmat[plate].reshape(3, 3)
        angles.append(np.arccos(np.clip((np.trace(rel) - 1) / 2, -1, 1)))

    spread = np.degrees(np.ptp(angles))
    assert spread < 0.05, f"plate drifts {spread:.3f} deg relative to the sole"


@needs_myo_sim
@pytest.mark.parametrize("msk", ["myolegs26", "myofullbody"])
def test_stride_hinge_is_coaxial_with_the_ankle(msk):
    """The master hinge is exactly antiparallel to the biological ankle axis.

    Antiparallel, not parallel: the device hinge points along -z in the tibia
    frame while ``ankle_angle`` points along +z, and that relative sense is what
    the couplings were fitted with.
    """
    model, data = load_combined(msk, "STRIDE_L2")
    mj.mj_forward(model, data)
    for side in ("r", "l"):
        ankle = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, f"ankle_angle_{side}")
        hinge = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, f"{STRIDE}shank_{side}__link_ag_{side}")
        dot = float(np.dot(data.xaxis[ankle], data.xaxis[hinge]))
        assert dot == pytest.approx(-1.0, abs=1e-6), f"{side}: axes {dot:+.6f} off coaxial"


# ----------------------------------------------------------------------
# contact
# ----------------------------------------------------------------------


@needs_myo_sim
@pytest.mark.parametrize("msk", ["myolegs26", "myofullbody"])
def test_stride_contacts_emitted_and_intra_device_filtered(msk):
    """Forced pairs and design-overlap excludes land; links never test each other.

    The shank is welded to tibia, whose weld parent is femur, and MuJoCo
    auto-excludes parent-child body pairs -- so the actuator would sink into the
    thigh without an explicit ``<pair>``.  Conversely the links are coaxial by
    design, and rather than one exclude per non-adjacent pair they are filtered by
    ``contype=2 conaffinity=1``, which must hold for every device geom.
    """
    model, _ = load_combined(msk, "STRIDE_L2")

    femur_geom = "femur_r_geom_1" if msk in ("myolegs22", "myolegs26") else "femur_r"
    pairs = {
        (
            mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, model.pair_geom1[p]),
            mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, model.pair_geom2[p]),
        )
        for p in range(model.npair)
    }
    assert any({STRIDE + "shank_r_main_geom", femur_geom} == set(p) for p in pairs)

    excludes = {
        tuple(sorted((model.exclude_signature[e] >> 16, model.exclude_signature[e] & 0xFFFF))) for e in range(model.nexclude)
    }
    assert len(excludes) >= 4  # plate + gcf against each calcaneus

    # Two device geoms must never be collision candidates.
    for gid in range(model.ngeom):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, gid) or ""
        if not name.startswith(STRIDE):
            continue
        assert model.geom_contype[gid] == 2, name
        assert model.geom_conaffinity[gid] == 1, name


@needs_myo_sim
@pytest.mark.parametrize("msk", ["myolegs26", "myofullbody"])
def test_stride_no_device_penetration_at_rest(msk):
    """No device geom starts the model interpenetrating anything.

    The baseline full-body model has its own humerus/thorax overlap at qpos0, so
    only device-involved contacts are checked here.
    """
    model, data = load_combined(msk, "STRIDE_L2")
    mj.mj_forward(model, data)
    offenders = []
    for i in range(data.ncon):
        c = data.contact[i]
        g1 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, c.geom1) or ""
        g2 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, c.geom2) or ""
        if (STRIDE in g1 or STRIDE in g2) and c.dist < 0:
            offenders.append((g1, g2, float(c.dist)))
    assert not offenders, f"device geoms penetrating at rest: {offenders}"


# ----------------------------------------------------------------------
# sensors / sensor_removals
# ----------------------------------------------------------------------


@needs_myo_sim
@pytest.mark.parametrize("device", ["KFoot_L1", "OpenSourceLeg_A_L1", "NEUankle_L1"])
@pytest.mark.parametrize("msk", ["myolegs22", "myolegs26"])
def test_prosthetic_side_sensors_restored(msk, device):
    """Amputation cascades away the right-side sensors; the config restores them.

    Deleting ``talus_r`` takes ``r_foot`` / ``r_toes`` (touch sites on the removed
    calcn/toes) and ``r_ankle_sensor`` / ``r_mtp_sensor`` with it, which left the
    baseline's 12 sensors at 8 and nothing reading the prosthetic side while the
    intact side kept all four counterparts.  ``r_mtp_sensor`` stays gone on
    purpose: the prosthetic foot is one plate with no toe joint.
    """
    model, _ = load_combined(msk, device)
    names = {mj.mj_id2name(model, mj.mjtObj.mjOBJ_SENSOR, i) for i in range(model.nsensor)}
    assert {"r_foot", "r_toes", "r_ankle_sensor"} <= names
    assert "r_mtp_sensor" not in names
    # The restored touch sensors read the device's own sites, not the removed ones.
    for sname in ("r_foot", "r_toes"):
        sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SENSOR, sname)
        assert model.sensor_objtype[sid] == mj.mjtObj.mjOBJ_SITE
        site = mj.mj_id2name(model, mj.mjtObj.mjOBJ_SITE, model.sensor_objid[sid])
        assert site.startswith(device.replace("OpenSourceLeg_A", "OSL_A") + "_")


@needs_myo_sim
@pytest.mark.parametrize("msk", ["myolegs26", "myofullbody"])
def test_stride_touch_sensors_repointed_to_the_soles(msk):
    """The shod model senses ground contact at the sole, not the bare foot.

    ``sensor_removals`` + ``sensors`` is how a sensor gets *re-pointed*.  It
    matters because the baseline ``r_foot_touch`` box spans y in [-0.022, 0.018]
    on ``calcn_r`` while the sole's contact surface sits 35 mm below the bottom of
    that box, so a sensor left on the baseline site reads zero all through stance.
    """
    model, _ = load_combined(msk, "STRIDE_L2")
    for sname in ("r_foot", "r_toes", "l_foot", "l_toes"):
        sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SENSOR, sname)
        assert sid >= 0, f"{sname} missing"
        site = mj.mj_id2name(model, mj.mjtObj.mjOBJ_SITE, model.sensor_objid[sid])
        assert site.startswith(STRIDE), f"{sname} still reads the baseline site {site}"
        body = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, model.site_bodyid[model.sensor_objid[sid]])
        assert "sole" in body, f"{sname} reads {body}, expected a sole"


# ----------------------------------------------------------------------
# tendon-actuator force limits
# ----------------------------------------------------------------------


@needs_myo_sim
@pytest.mark.parametrize("msk", ["myolegs26", "myofullbody"])
def test_cable_forcerange_survives_import(msk):
    """``forcerange`` / ``forcelimited`` reach the combined model.

    They used to be dropped, which mattered more than it sounds: ``forcerange``
    clamps in *actuator* space, before ``gear`` is applied, so the natural-looking
    ``gear="-1"`` with ``forcerange="-400 0"`` spelling yields exactly zero force.
    Keeping the limit means the 0..400 spelling is the one that survives.
    """
    model, _ = load_combined(msk, "STRIDE_L2")
    for side in ("r", "l"):
        aid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, f"{STRIDE}cable_{side}")
        assert aid >= 0
        assert model.actuator_forcelimited[aid] == 1
        np.testing.assert_allclose(model.actuator_forcerange[aid], [0.0, 400.0])
        assert model.actuator_gear[aid][0] == pytest.approx(-1.0)


@needs_myo_sim
@pytest.mark.parametrize("msk", ["myolegs26", "myofullbody"])
def test_cable_shortens_into_plantarflexion(msk):
    """Pulling the cable plantarflexes, and the moment arm is physically sized.

    Sign check: cable length must *increase* with ankle angle, so tension produces
    a negative (plantarflexing) generalized torque.  Getting this backwards is
    easy and silent -- the device still runs, it just assists the wrong phase.
    """
    model, data = load_combined(msk, "STRIDE_L2")
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "ankle_angle_r")
    tid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_TENDON, STRIDE + "cable_r")

    qs = np.linspace(*model.jnt_range[jid], 17)
    lengths = []
    for q in qs:
        data.qpos[:] = model.qpos0
        data.qvel[:] = 0
        data.qpos[_qadr(model, "ankle_angle_r")] = q
        ag = _poly(_AG, q)
        data.qpos[_qadr(model, STRIDE + "shank_r__link_ag_r")] = ag
        for joint, coef in _CHAIN.items():
            data.qpos[_qadr(model, STRIDE + joint)] = _poly(coef, ag)
        mj.mj_forward(model, data)
        lengths.append(data.ten_length[tid])

    lengths = np.array(lengths)
    assert np.all(np.diff(lengths) > 0), "cable does not shorten toward plantarflexion"
    arm = np.abs(np.gradient(lengths, qs))
    assert 0.05 < arm.min() and arm.max() < 0.15, f"moment arm {arm.min():.3f}..{arm.max():.3f} m/rad"


# ----------------------------------------------------------------------
# static config resolution (no myo_sim needed)
# ----------------------------------------------------------------------


def test_stride_mounts_identically_on_every_lineage(models_dir):
    """No attachment carries a per-MSK pose offset.

    The device is a rigid frame strapped flat to the shank, so it must sit the same
    way on every model.  Canting it per lineage to chase a differing ankle axis
    swings the actuator 27 mm laterally at the top and the plate 19 mm the other
    way at the bottom -- visibly crooked.  Where the axes disagree the *joint* axis
    is declared instead; see the next test.
    """
    config = DeviceConfig.from_yaml(str(models_dir / "STRIDE" / "L2config.yaml"))
    for msk in (None, "myolegs22", "myolegs26", "myolegs", "myofullbody"):
        for att in config.resolve_attachments(msk):
            assert att.pos is None, (msk, att.device_body)
            assert att.quat is None, (msk, att.device_body)


def test_stride_declares_a_sagittal_ankle_on_the_80_muscle_lineage(models_dir):
    """The 80-muscle lineage gets a sagittal ankle axis; the range is intersected.

    A rigid single-plane linkage cannot be coaxial with the anatomical talocrural
    axis (11.7 deg out of sagittal), and the mismatch drifts the plate off the sole
    by ~8 deg over the ROM.  Declaring the axis is what the hardware physically
    does; re-posing the device is not.

    The range stays the intersection of the model's own range with the coupling fit
    window [-0.888851, 0.349], so plantarflexion is never *widened* past the
    anatomical limit even though the axis was redefined.
    """
    config = DeviceConfig.from_yaml(str(models_dir / "STRIDE" / "L2config.yaml"))

    default = {j.name: j for j in config.resolve_joint_overrides()}
    assert default["ankle_angle_r"].range == [-0.888851, 0.349]
    assert default["ankle_angle_r"].axis is None  # 26/22-muscle is already sagittal

    for msk in ("myolegs", "myofullbody"):
        overrides = {j.name: j for j in config.resolve_joint_overrides(msk)}
        assert overrides["ankle_angle_r"].axis == [0, 0, 1], msk
        assert overrides["ankle_angle_r"].pos == [0, -0.4, 0], msk
        assert overrides["ankle_angle_r"].range == [-0.698132, 0.349], msk
        assert overrides["mtp_angle_r"].range == [0.2, 0.5], msk


@needs_myo_sim
@pytest.mark.parametrize("msk", ["myolegs22", "myolegs26", "myolegs", "myofullbody"])
def test_stride_geometry_matches_across_lineages(msk):
    """The device's geometry lands in the same place in the tibia frame on every MSK.

    This is the guard that catches a mount drifting on one lineage only.  Compared
    against ``myolegs26`` as the reference, since that is the lineage the CAD pose
    was authored and visually verified against.
    """
    reference, device = load_combined("myolegs26", "STRIDE_L2")
    mj.mj_forward(reference, device)
    model, data = load_combined(msk, "STRIDE_L2")
    mj.mj_forward(model, data)

    def centroid(m, d, geom_name):
        gid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, geom_name)
        mid = m.geom_dataid[gid]
        verts = m.mesh_vert[m.mesh_vertadr[mid] : m.mesh_vertadr[mid] + m.mesh_vertnum[mid]].astype(float)
        world = (d.geom_xmat[gid].reshape(3, 3) @ verts.T).T + d.geom_xpos[gid]
        tib = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, "tibia_r")
        return ((d.xmat[tib].reshape(3, 3).T @ (world - d.xpos[tib]).T).T).mean(axis=0)

    for geom in ("shank_r_actuator_geom", "link_ag_r_main_geom", "foot_r_main_geom"):
        expected = centroid(reference, device, STRIDE + geom)
        actual = centroid(model, data, STRIDE + geom)
        np.testing.assert_allclose(actual, expected, atol=1e-9, err_msg=f"{msk}: {geom} moved")


@pytest.mark.parametrize("device,config_name", [("NEUankle", "L1config.yaml"), ("STRIDE", "L2config.yaml")])
def test_new_configs_validate_against_their_device_xml(models_dir, device, config_name):
    """Every name the new configs reference resolves in the device XML."""
    from assist_sim.validate import _collect_names

    config = DeviceConfig.from_yaml(str(models_dir / device / config_name))
    names = _collect_names(str(config.model_xml_path))
    for att in config.attachments:
        assert att.device_body in names.bodies, att.device_body
    for msk in ("myolegs", "myofullbody"):
        for att in config.resolve_attachments(msk):
            assert att.device_body in names.bodies, (msk, att.device_body)
    for sensor in config.sensors:
        if sensor.target_kind == "site":
            assert sensor.target in names.sites, sensor.target
