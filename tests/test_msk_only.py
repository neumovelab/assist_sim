"""The device-less pathway: ``load_msk`` hands a bare MSK to a downstream consumer.

Guards the two things that make it more than a thin wrapper: that skipping the whole
combine pipeline still yields the same baseline a combined build starts from, and that
the export is model-only and reloadable.
"""

from __future__ import annotations

import mujoco as mj
import pytest

from assist_sim import load_msk

pytest.importorskip("myo_sim", reason="MSK resolution needs myo_sim")

MSK = "myolegs26"


def test_load_msk_returns_compiled_baseline():
    model, data = load_msk(MSK)
    assert isinstance(model, mj.MjModel)
    assert isinstance(data, mj.MjData)
    assert model.nq > 0 and model.nu > 0


def test_load_msk_matches_the_baseline_a_combined_build_starts_from():
    """The device-less model is the same human the combine path is handed.

    Compared against the registry resolver rather than a hard-coded tuple so this
    tracks myo_sim rather than pinning it.
    """
    from assist_sim.registry import _resolve_msk

    expected = _resolve_msk(MSK).compile()
    model, _ = load_msk(MSK)
    assert (model.nq, model.nv, model.nu, model.nbody) == (
        expected.nq,
        expected.nv,
        expected.nu,
        expected.nbody,
    )
    assert model.body_mass.sum() == pytest.approx(expected.body_mass.sum())


def test_export_is_model_only_and_reloads(tmp_path):
    out = tmp_path / "msk.xml"
    model, _ = load_msk(MSK, export_xml=str(out))
    assert out.exists()

    reloaded = mj.MjModel.from_xml_path(str(out))
    assert (reloaded.nq, reloaded.nu, reloaded.nbody) == (model.nq, model.nu, model.nbody)
    assert reloaded.body_mass.sum() == pytest.approx(model.body_mass.sum())

    # Model-only means terrain is stripped -- no ground plane to stand on, since
    # downstream consumers layer the scene themselves.  The composed MSK's default
    # skybox is deliberately *not* covered by that: combined exports keep it too,
    # so asserting its absence here would pin behaviour the combined path lacks.
    assert not any(reloaded.geom_type[g] == mj.mjtGeom.mjGEOM_PLANE for g in range(reloaded.ngeom))


def test_no_device_prefixed_names_survive():
    """Nothing device-shaped leaks in: every body belongs to the human."""
    model, _ = load_msk(MSK)
    names = [mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i) or "" for i in range(1, model.nbody)]
    assert names
    for stem in ("exo_", "hippo_", "osl_", "tb_", "_sole", "_upper"):
        assert not any(stem in n for n in names), f"device-shaped body {stem!r} in MSK-only build"


def test_cache_round_trip(tmp_path):
    cache = tmp_path / "cache"
    first, _ = load_msk(MSK, cache_dir=str(cache))
    assert list(cache.glob("*.xml")), "nothing cached"
    second, _ = load_msk(MSK, cache_dir=str(cache))  # served from cache
    assert (second.nq, second.nu, second.nbody) == (first.nq, first.nu, first.nbody)
    assert second.body_mass.sum() == pytest.approx(first.body_mass.sum())
