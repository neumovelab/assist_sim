"""High-level loading utilities: a drop-in replacement for path resolution.

These wrap the autodiscovery registry so callers can work in terms of
``(msk_key, device_key)`` instead of file paths::

    from assist_sim import load_combined

    model, data = load_combined("myolegs26", "DephyExoBoot_L1")
"""

from __future__ import annotations

import shutil
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union

import mujoco as mj

from . import registry


def _package_token(module) -> str:
    """``version + newest source mtime`` for a package, for use in a cache key.

    The version alone is not enough.  Both assist_sim and myo_sim are normally installed
    editable during development, where the version is whatever was recorded at install time
    and does not move when the code changes -- so a cache keyed on version alone happily
    serves a model built by yesterday's pipeline.  (The previous ``_myosim_token`` claimed to
    fall back to an mtime for exactly this case, but the fallback was unreachable: myo_sim
    always defines ``__version__``.)

    Folding in the newest ``*.py`` mtime under the package makes an edit invalidate the
    cache.  Measured at a few milliseconds for both packages, against cache hits worth
    hundreds of milliseconds, so it is worth paying on every lookup.
    """
    version = str(getattr(module, "__version__", "unknown"))
    try:
        root = Path(module.__file__).resolve().parent
        newest = max((p.stat().st_mtime_ns for p in root.rglob("*.py")), default=0)
    except Exception:
        newest = 0
    return f"{version}+src{newest}"


def _myosim_token() -> str:
    """Cache-key token for the composed-MSK source (myo_sim); ``"absent"`` if missing."""
    try:
        import myo_sim
    except Exception:
        return "absent"
    return _package_token(myo_sim)


def _assist_sim_token() -> str:
    """Cache-key token for this package: release version plus local source state."""
    from . import __version__

    import assist_sim

    return f"{__version__}|{_package_token(assist_sim)}"


def resolve_model_path(msk_key: str, device_key: str) -> Tuple["mj.MjSpec", Path]:
    """Resolve a combination to ``(human_spec, device_config_path)``.

    The first element is a freshly-composed, model-only ``MjSpec``, **not** a path: an MSK
    registry key has no XML on disk, myo_sim composes it in memory.  The name predates
    that change and is kept because it is public API; the in-memory spec is the
    equivalent of what the path used to point at.

    Composing is not free -- this builds the whole MSK -- so call it when you want the
    spec, not merely to check that a pair resolves.  Use
    :func:`validate_combination` for the latter.

    Raises ``ValueError`` (with suggestions) if either key is unknown or the
    pair is incompatible.
    """
    return registry.resolve(msk_key, device_key)


def load_combined(
    msk_key: str,
    device_key: str,
    export_xml: Optional[str] = None,
    cache_dir: Optional[Union[str, Path]] = None,
    planar_root: bool = False,
) -> Tuple[mj.MjModel, mj.MjData]:
    """Resolve ``(msk_key, device_key)`` and return the combined model.

    The MSK is composed on demand by ``myo_sim`` (see
    :func:`assist_sim.registry.resolve`) and handed to the pipeline as a live
    ``MjSpec``; ``msk_key`` is forwarded so per-MSK config overrides apply.

    ``cache_dir`` opts into the on-disk cache.  A hit skips the compose *and* the combine,
    reloading the exported XML instead.  Whether that is faster depends on the model, because
    the reload cost is dominated by XML text parsing (measured, best of five, this machine):

    ==============  ==============  =============  =========
    MSK             full combine    cached reload  speedup
    ==============  ==============  =============  =========
    ``myolegs22``   0.47 s          0.05 s         9.0x
    ``myolegs26``   0.38 s          0.16 s         2.4x
    ``myolegs``     0.47 s          0.23 s         2.0x
    ``myofullbody`` 0.65 s          1.01 s         **0.6x**
    ==============  ==============  =============  =========

    ``myofullbody`` exports 0.6 MB of MJCF (418 actuators, 108 meshes) and parsing that costs
    more than composing it from scratch, so caching it is a pessimisation.  Leave
    ``cache_dir`` unset for that model.
    """
    from .combine import ModelCombiner  # lazy import avoids a circular import
    from .config import DeviceConfig

    if cache_dir is None:
        human_spec, device_config_path = registry.resolve(msk_key, device_key)
        config = DeviceConfig.from_yaml(str(device_config_path))
        return ModelCombiner().combine(human_spec, config, export_xml=export_xml, msk_key=msk_key, planar_root=planar_root)

    # Validate the pair and locate the device config *without* composing: the compose is the
    # expensive half, and on a hit it would be built only to be discarded.  (It used to run
    # before the lookup, which is why a hit measured the same as a miss -- 1.8 s either way
    # for myofullbody.)
    from . import cache as _cache

    device_config_path = registry.resolve_device_config(msk_key, device_key)
    config = DeviceConfig.from_yaml(str(device_config_path))

    cache_dir = Path(cache_dir)
    paths = _cache.input_paths_composed(str(device_config_path), str(config.model_xml_path))
    version = f"{_assist_sim_token()}+myosim-{_myosim_token()}"
    key = _cache.compute_key(paths, version, f"{msk_key}+planar" if planar_root else msk_key)

    hit = _cache.try_load(cache_dir, key)
    if hit is not None:
        if export_xml:
            shutil.copyfile(_cache.cached_xml_path(cache_dir, key), export_xml)
        return hit

    # Miss: compose now, build into the staging name, then publish atomically.
    human_spec = registry._resolve_msk(msk_key)
    cache_dir.mkdir(parents=True, exist_ok=True)
    staged = _cache.staging_xml_path(cache_dir, key)
    model, data = ModelCombiner().combine(human_spec, config, export_xml=str(staged), msk_key=msk_key, planar_root=planar_root)
    cached_xml = str(_cache.commit(cache_dir, key, staged))
    _cache.write_meta(
        cache_dir,
        key,
        {
            "assist_sim": _assist_sim_token(),
            "msk_key": msk_key,
            "device_key": device_key,
            "planar_root": planar_root,
            "myo_sim": _myosim_token(),
            "inputs": [str(p) for p in paths],
        },
    )
    if export_xml and export_xml != cached_xml:
        shutil.copyfile(cached_xml, export_xml)
    return model, data


def load_msk(
    msk_key: str,
    export_xml: Optional[str] = None,
    cache_dir: Optional[Union[str, Path]] = None,
) -> Tuple[mj.MjModel, mj.MjData]:
    """Resolve ``msk_key`` alone and return the baseline model, with no device.

    The device-less counterpart to :func:`load_combined`, for handing a bare MSK
    to a downstream consumer.  Almost all of the combine pipeline is device work
    -- surgery, attachment, actuators, tendons, equalities, contacts, sensors --
    so this skips straight from the resolved spec to a compile.  Keyframes need
    none of the decompose/rebuild that :meth:`ModelCombiner.combine` performs,
    because with no surgery the qpos/dof layout never changes.

    The export carries no ground plane, hfield or floor material, because the
    myosuite scene is stripped at resolve time.  It is not scene-free, though:
    every export gains a soft headlight and a neutral gradient skybox, so a
    caller wanting a specific backdrop must replace them.

    Args:
        msk_key: MSK registry key, e.g. ``"myolegs26"``.
        export_xml: If provided, save the compiled model XML to this path.
        cache_dir: Optional directory enabling local caching, as per
            :func:`load_combined`.

    Returns:
        Tuple of (MjModel, MjData) ready for simulation.
    """

    def _build(target_xml: Optional[str]) -> Tuple[mj.MjModel, mj.MjData]:
        spec = registry._resolve_msk(msk_key)
        model = spec.compile()
        if target_xml:
            from .utils import export_combined_xml

            mesh_dirs = [(Path(spec.modelfiledir), getattr(spec, "meshdir", "") or "")]
            export_combined_xml(spec, target_xml, mesh_dirs=mesh_dirs)
        return model, mj.MjData(model)

    if cache_dir is None:
        return _build(export_xml)

    # As in load_combined: compose only on a miss.  The key needs no input files -- an MSK has
    # no source on disk -- so it is entirely (msk_key, assist_sim token, myo_sim token), and
    # both tokens now move when their package's source does.
    from . import cache as _cache

    cache_dir = Path(cache_dir)
    version = f"{_assist_sim_token()}+myosim-{_myosim_token()}"
    key = _cache.compute_key(_cache.input_paths_msk(), version, msk_key)

    hit = _cache.try_load(cache_dir, key)
    if hit is not None:
        if export_xml:
            shutil.copyfile(_cache.cached_xml_path(cache_dir, key), export_xml)
        return hit

    cache_dir.mkdir(parents=True, exist_ok=True)
    staged = _cache.staging_xml_path(cache_dir, key)
    model, data = _build(str(staged))
    cached_xml = str(_cache.commit(cache_dir, key, staged))
    _cache.write_meta(
        cache_dir,
        key,
        {
            "assist_sim": _assist_sim_token(),
            "msk_key": msk_key,
            "device_key": None,
            "myo_sim": _myosim_token(),
            "inputs": [],
        },
    )
    if export_xml and export_xml != cached_xml:
        shutil.copyfile(cached_xml, export_xml)
    return model, data


def get_available_combinations() -> Dict[str, List[str]]:
    """Return ``{msk_key: [device_key, ...]}`` of discoverable combinations."""
    return registry.get_available_combinations()


def validate_combination(msk_key: str, device_key: str) -> bool:
    """Return True if the pair resolves and is compatible; else False."""
    return registry.validate_combination(msk_key, device_key)
