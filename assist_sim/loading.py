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


def _myosim_token() -> str:
    """A version token for the composed-MSK source (myo_sim), for cache keys.

    Prefers ``myo_sim.__version__``; falls back to the package file's mtime so a
    local editable-install edit still invalidates cached composed XMLs.
    """
    try:
        import myo_sim

        version = getattr(myo_sim, "__version__", None)
        if version:
            return str(version)
        return str(Path(myo_sim.__file__).stat().st_mtime_ns)
    except Exception:
        return "unknown"


def resolve_model_path(msk_key: str, device_key: str) -> Tuple[Path, Path]:
    """Resolve a combination to ``(human_xml, device_config)`` paths.

    Raises ``ValueError`` (with suggestions) if either key is unknown or the
    pair is incompatible.
    """
    return registry.resolve(msk_key, device_key)


def load_combined(
    msk_key: str,
    device_key: str,
    export_xml: Optional[str] = None,
    cache_dir: Optional[Union[str, Path]] = None,
) -> Tuple[mj.MjModel, mj.MjData]:
    """Resolve ``(msk_key, device_key)`` and return the combined model.

    The MSK is composed on demand by ``myo_sim`` (see
    :func:`assist_sim.registry.resolve`) and handed to the pipeline as a live
    ``MjSpec``; ``msk_key`` is forwarded so per-MSK config overrides apply.
    """
    from .combine import ModelCombiner  # lazy import avoids a circular import
    from .config import DeviceConfig

    human_spec, device_config_path = registry.resolve(msk_key, device_key)
    config = DeviceConfig.from_yaml(str(device_config_path))

    if cache_dir is None:
        return ModelCombiner().combine(human_spec, config, export_xml=export_xml, msk_key=msk_key)

    # Composed MSKs now round-trip through export_combined_xml, so caching works.
    # The MSK has no source file on disk; its identity is the (msk_key, myo_sim
    # version) pair, folded into the cache key alongside the device files.
    from . import __version__
    from . import cache as _cache

    cache_dir = Path(cache_dir)
    paths = _cache.input_paths_composed(str(device_config_path), str(config.model_xml_path))
    version = f"{__version__}+myosim-{_myosim_token()}"
    key = _cache.compute_key(paths, version, msk_key)

    hit = _cache.try_load(cache_dir, key)
    if hit is not None:
        if export_xml:
            shutil.copyfile(_cache.cached_xml_path(cache_dir, key), export_xml)
        return hit

    cache_dir.mkdir(parents=True, exist_ok=True)
    cached_xml = str(_cache.cached_xml_path(cache_dir, key))
    model, data = ModelCombiner().combine(human_spec, config, export_xml=cached_xml, msk_key=msk_key)
    _cache.write_meta(
        cache_dir,
        key,
        {
            "assist_sim_version": __version__,
            "msk_key": msk_key,
            "device_key": device_key,
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
    spec = registry._resolve_msk(msk_key)

    def _build(target_xml: Optional[str]) -> Tuple[mj.MjModel, mj.MjData]:
        model = spec.compile()
        if target_xml:
            from .utils import export_combined_xml

            mesh_dirs = [(Path(spec.modelfiledir), getattr(spec, "meshdir", "") or "")]
            export_combined_xml(spec, target_xml, mesh_dirs=mesh_dirs)
        return model, mj.MjData(model)

    if cache_dir is None:
        return _build(export_xml)

    from . import __version__
    from . import cache as _cache

    cache_dir = Path(cache_dir)
    version = f"{__version__}+myosim-{_myosim_token()}"
    key = _cache.compute_key(_cache.input_paths_msk(), version, msk_key)

    hit = _cache.try_load(cache_dir, key)
    if hit is not None:
        if export_xml:
            shutil.copyfile(_cache.cached_xml_path(cache_dir, key), export_xml)
        return hit

    cache_dir.mkdir(parents=True, exist_ok=True)
    cached_xml = str(_cache.cached_xml_path(cache_dir, key))
    model, data = _build(cached_xml)
    _cache.write_meta(
        cache_dir,
        key,
        {
            "assist_sim_version": __version__,
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
