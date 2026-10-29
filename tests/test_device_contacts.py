"""Device contacts: no part touches its own side, and no contact pushes hard at a keyframe.

The composer gives every colliding device geom the collision bits of its side
(``ModelCombiner._set_device_contact_bits``). The parts of one side never collide, left and
right parts collide, and every part collides with the MSK and terrain geoms as before. Before
this rule, the parts of one side of ``DephyExoBoot_L1``, ``Humotech_L1``, ``Anatomics_L1`` and
``OpenExo_L1`` sat inside each other on ``myolegs`` and ``myofullbody``, and the solver pushed
them apart at 3-21 kN at every keyframe. Nothing failed: the models compiled and stepped.

Three checks per MSK x device combination:

- No pair of device geoms that the rule keeps apart is a collision candidate. This check is
  structural. It reads the contype and conaffinity bits, the weld and parent filters, the
  excludes and the explicit pairs, so it does not depend on a pose.
- At each keyframe, the contacts between device geoms and MSK geoms push with
  ``MAX_FORCE_N`` or less in total and go no deeper than ``MAX_PENETRATION_M``.
- At each keyframe, the contacts between two device geoms (left against right) obey the
  same limits.

``MAX_FORCE_N`` comes from the measured distribution (2026-10-29, the 52 combinations at the
five keyframes, after the rule). Between device and MSK, 247 of the 260 cases give 0 N. The
other cases are 7.7-9.8 N (a thumb that rests on the ``OpenSourceLeg_KA_L1`` socket, which
fails the depth limit), then 87 N and more, up to 50 kN. Between left and right parts, the
smallest force is 38 N. 25 N admits seating contact and fails every case that loads a joint.
The depth limit is the one that ``test_device_contact_overlap.py`` applied at ``stand``. This
module checks it at every keyframe and for every device, so it replaces that module, which
also missed the two ``OpenSourceLeg`` devices because their prefix is not their registry key.

The known failures are strict xfails, so a fix makes the test fail until its entry goes.

A module-scoped fixture compiles each combination once for all the checks, and pytest groups
the tests by combination, so only one model is in memory at a time.
"""

from __future__ import annotations

import mujoco as mj
import numpy as np
import pytest

from assist_sim import load_combined, registry

from .conftest import needs_myo_sim
from .test_smoke_combinations import EXPECTED

pytestmark = needs_myo_sim

MAX_FORCE_N = 25.0
MAX_PENETRATION_M = 0.002
KEYFRAMES = ("stand", "walk_left", "walk_right", "squat", "lunge")
RIGHT_SUFFIXES = ("_r", "_R", "_dx")
LEFT_SUFFIXES = ("_l", "_L", "_sx")
LEGS_80 = ("myolegs", "myofullbody")

# The squat of the 80-muscle lineage puts the heels 93 mm apart (168 mm at stand), so the
# left and right shoes, boots and exo parts cross.
_SQUAT_FEET = "80-muscle squat: heels 93 mm apart, so the left and right foot parts cross"

# (msk, device, keyframe) -> reason, for contacts between a device geom and an MSK geom.
KNOWN_DEVICE_MSK: dict[tuple[str, str, str], str] = {
    **{(msk, "STRIDE_L2", "squat"): "STRIDE link_ag 37 mm inside calcn (50 kN)" for msk in LEGS_80},
    **{
        (msk, "STRIDE_L2", "lunge"): "STRIDE link_ag_r 37 mm inside calcn_r, r_fore_upper inside tibia_r (27 kN)"
        for msk in LEGS_80
    },
    **{(msk, "STRIDE_L2", kf): "STRIDE shank pushes 87 N on calcn" for msk in LEGS_80 for kf in ("walk_left", "walk_right")},
    **{(msk, "Anatomics_L1", "squat"): f"Anatomics right exo 12 mm inside tibia_l (116 N); {_SQUAT_FEET}" for msk in LEGS_80},
    **{
        ("myofullbody", "OpenSourceLeg_KA_L1", kf): "the right thumb rests 6.6 mm inside the OSL_KA socket (10 N)"
        for kf in ("walk_right", "lunge")
    },
    ("myolegs26", "UTAnkleExo_L2", "lunge"): "misplaced free-rooted UT exo (A-S4): part2_dx 7 mm inside toes_r (165 N)",
}

# (msk, device, keyframe) -> reason, for contacts between two device geoms.
KNOWN_DEVICE_DEVICE: dict[tuple[str, str, str], str] = {
    **{
        (msk, "HMEDI_L1", kf): "the HMEDI femur flaps sit 5.7 mm inside each other"
        for msk in LEGS_80
        for kf in ("stand", "squat")
    },
    **{(msk, "Humotech_L1", "stand"): "the Humotech heel rings sit 5.4 mm inside each other (38 N)" for msk in LEGS_80},
    **{
        (msk, dev, "squat"): _SQUAT_FEET
        for msk in LEGS_80
        for dev in ("Anatomics_L1", "DephyExoBoot_L1", "Humotech_L1", "STRIDE_L2")
    },
}


def _name_side(name: str) -> str | None:
    if name.endswith(RIGHT_SUFFIXES):
        return "right"
    if name.endswith(LEFT_SUFFIXES):
        return "left"
    return None


def _device_sides(model: mj.MjModel, prefix: str) -> dict[int, str | None]:
    """Device body id -> "left", "right" or None (unsided), read from the body names.

    The first non-device ancestor decides. If its name has no side suffix (pelvis, torso,
    world), the device body or its nearest device ancestor with a suffix decides.
    """
    sides: dict[int, str | None] = {}
    for body in range(1, model.nbody):
        if not model.body(body).name.startswith(prefix):
            continue
        chain, anchor = [], body
        while anchor != 0 and model.body(anchor).name.startswith(prefix):
            chain.append(model.body(anchor).name)
            anchor = int(model.body_parentid[anchor])
        side = _name_side(model.body(anchor).name) if anchor != 0 else None
        sides[body] = side or next((s for s in map(_name_side, chain) if s), None)
    return sides


@pytest.fixture(scope="module", params=list(EXPECTED), ids=str)
def combined(request):
    msk_key, device_key = request.param
    model, data = load_combined(msk_key, device_key)
    prefix = registry._DEVICE_NAMES.get(device_key, device_key) + "_"
    sides = _device_sides(model, prefix)
    assert sides, f"no body in {msk_key} x {device_key} starts with {prefix!r}"
    return msk_key, device_key, model, data, sides


def _apart(sides: dict[int, str | None], b1: int, b2: int) -> bool:
    """True when the side rule keeps two device bodies apart (one side, or unsided)."""
    return sides[b1] is None or sides[b2] is None or sides[b1] == sides[b2]


def _forbidden_candidates(model: mj.MjModel, sides: dict[int, str | None]) -> list[tuple[str, str]]:
    """Device geom pairs that MuJoCo can test, although the side rule keeps them apart.

    This mirrors the MuJoCo collision filter: the contype / conaffinity bits, no test inside
    one weld group or between a weld group and its parent (unless filterparent is off), the
    body excludes, and the explicit pairs, which ignore all of these.
    """
    filterparent = not (model.opt.disableflags & mj.mjtDisableBit.mjDSBL_FILTERPARENT)
    excluded = {(int(s) >> 16, int(s) & 0xFFFF) for s in model.exclude_signature}
    weld, parent = model.body_weldid, model.body_parentid
    contype, conaffinity, body_of = model.geom_contype, model.geom_conaffinity, model.geom_bodyid
    geoms = [g for g in range(model.ngeom) if int(body_of[g]) in sides and (contype[g] or conaffinity[g])]

    def label(g1: int, g2: int) -> tuple[str, str]:
        return model.geom(g1).name or model.body(body_of[g1]).name, model.geom(g2).name or model.body(body_of[g2]).name

    out = []
    for i, g1 in enumerate(geoms):
        for g2 in geoms[i + 1 :]:
            b1, b2 = int(body_of[g1]), int(body_of[g2])
            if not _apart(sides, b1, b2) or not (contype[g1] & conaffinity[g2] or contype[g2] & conaffinity[g1]):
                continue
            w1, w2 = int(weld[b1]), int(weld[b2])
            if w1 == w2:
                continue
            if filterparent and w1 and w2 and (w1 == weld[parent[w2]] or w2 == weld[parent[w1]]):
                continue
            if (min(b1, b2), max(b1, b2)) in excluded:
                continue
            out.append(label(g1, g2))
    for p in range(model.npair):
        g1, g2 = int(model.pair_geom1[p]), int(model.pair_geom2[p])
        b1, b2 = int(body_of[g1]), int(body_of[g2])
        if b1 in sides and b2 in sides and _apart(sides, b1, b2):
            out.append(label(g1, g2))
    return out


def test_no_same_side_device_collision(combined):
    msk_key, device_key, model, _, sides = combined
    forbidden = _forbidden_candidates(model, sides)
    assert not forbidden, (
        f"{msk_key} x {device_key}: {len(forbidden)} device geom pairs on one side (or unsided) can "
        f"collide, for example {forbidden[:5]}"
    )


def _device_contact_load(model, data, sides, kind: str) -> tuple[float, float, dict[str, float]]:
    """Total normal force (N), deepest penetration (m) and force per body pair for one kind.

    *kind* is ``"device_msk"`` (a device geom against an MSK geom) or ``"device_device"``.
    Contacts with the world body are not counted.
    """
    total, depth, per_pair = 0.0, 0.0, {}
    force = np.zeros(6)
    for i in range(data.ncon):
        contact = data.contact[i]
        b1, b2 = (int(model.geom_bodyid[g]) for g in contact.geom)
        if b1 == 0 or b2 == 0 or (b1 not in sides and b2 not in sides):
            continue
        if ("device_device" if b1 in sides and b2 in sides else "device_msk") != kind:
            continue
        mj.mj_contactForce(model, data, i, force)
        total += float(force[0])
        depth = max(depth, -float(contact.dist))
        pair = " | ".join(sorted((model.body(b1).name, model.body(b2).name)))
        per_pair[pair] = per_pair.get(pair, 0.0) + float(force[0])
    return total, depth, per_pair


def _check_contacts(combined, keyframe: str, kind: str, known: dict, request) -> None:
    msk_key, device_key, model, data, sides = combined
    reason = known.get((msk_key, device_key, keyframe))
    if reason:
        request.applymarker(pytest.mark.xfail(strict=True, reason=reason))
    key = mj.mj_name2id(model, mj.mjtObj.mjOBJ_KEY, keyframe)
    assert key >= 0, f"{msk_key} x {device_key} has no '{keyframe}' keyframe"
    mj.mj_resetDataKeyframe(model, data, key)
    mj.mj_forward(model, data)

    total, depth, per_pair = _device_contact_load(model, data, sides, kind)
    worst = {pair: round(f, 1) for pair, f in sorted(per_pair.items(), key=lambda kv: -kv[1])[:4]}
    assert total <= MAX_FORCE_N and depth <= MAX_PENETRATION_M, (
        f"{msk_key} x {device_key} at '{keyframe}': {kind} contacts push {total:.1f} N "
        f"(limit {MAX_FORCE_N} N) and reach {depth * 1000:.1f} mm deep "
        f"(limit {MAX_PENETRATION_M * 1000:.0f} mm). Largest body pairs (N): {worst}"
    )


@pytest.mark.parametrize("keyframe", KEYFRAMES)
def test_device_msk_contact_force(combined, keyframe, request):
    _check_contacts(combined, keyframe, "device_msk", KNOWN_DEVICE_MSK, request)


@pytest.mark.parametrize("keyframe", KEYFRAMES)
def test_device_device_contact_force(combined, keyframe, request):
    _check_contacts(combined, keyframe, "device_device", KNOWN_DEVICE_DEVICE, request)
