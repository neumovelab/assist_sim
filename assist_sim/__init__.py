"""assist_sim — programmatic combination of musculoskeletal models with assistive devices.

Provides a single function to programmatically combine a musculoskeletal
baseline model (sourced from the ``myo_sim`` package) with a device model
defined locally under ``models/``, using the MuJoCo mjSpec API.

Usage::

    from assist_sim import load_combined_model

    model, data = load_combined_model(
        human_xml="external/myo_sim/leg/myoLeg22_2D.xml",
        device_config="models/DephyExoBoot/L1config.yaml",
    )

Or, by registry key (preferred once myo_sim is installed)::

    from assist_sim import load_combined

    model, data = load_combined(msk="myoLeg22_2D", device="DephyExoBoot_L1")
"""

import shutil
from pathlib import Path
from typing import Optional, Tuple, Union

import mujoco as mj

from .combine import ModelCombiner  # noqa: F401  (accessible, not re-exported)
from .config import DeviceConfig
from .loading import (  # noqa: F401
    get_available_combinations,
    load_combined,
    resolve_model_path,
    validate_combination,
)

# Bump whenever a pipeline change affects compiled-model output; the cache
# key includes this so stale cached XMLs are invalidated automatically.
__version__ = "0.1.0"

# Keep the public surface small: load_combined_model is the documented path.
# ModelCombiner stays importable from assist_sim.combine for advanced callers
# but is intentionally not part of the star-export surface.
__all__ = [
    "load_combined_model",
    "DeviceConfig",
    "load_combined",
    "resolve_model_path",
    "get_available_combinations",
    "validate_combination",
]


def load_combined_model(
    human_xml: str,
    device_config: str,
    export_xml: Optional[str] = None,
    msk_key: Optional[str] = None,
    keep_temp: bool = False,
    cache_dir: Optional[Union[str, Path]] = None,
) -> Tuple[mj.MjModel, mj.MjData]:
    """Combine a musculoskeletal model with a device and return the compiled result.

    This is a drop-in replacement for loading a pre-combined XML file.
    The baseline human model is never modified on disk.

    Args:
        human_xml: Path to the baseline musculoskeletal model XML (typically
            resolved from the ``myo_sim`` package).
        device_config: Path to the device's config.yaml file (under ``models/``).
        export_xml: Optional path to save the combined model as XML.
        msk_key: Optional MSK key for per-MSK config overrides.
        keep_temp: If True, leave the preprocess temp files on disk.
        cache_dir: Optional directory enabling local caching. When set, a
            combined model whose inputs are unchanged is loaded from disk
            instead of recompiled. Off by default.

    Returns:
        Tuple of (MjModel, MjData) ready for simulation.
    """
    config = DeviceConfig.from_yaml(device_config)

    if cache_dir is not None:
        from . import cache as _cache

        cache_dir = Path(cache_dir)
        paths = _cache.input_paths(
            human_xml, device_config, str(config.model_xml_path)
        )
        key = _cache.compute_key(paths, __version__, msk_key)

        hit = _cache.try_load(cache_dir, key)
        if hit is not None:
            if export_xml:
                shutil.copyfile(_cache.cached_xml_path(cache_dir, key), export_xml)
            return hit

        cache_dir.mkdir(parents=True, exist_ok=True)
        cached_xml = str(_cache.cached_xml_path(cache_dir, key))
        model, data = ModelCombiner().combine(
            human_xml,
            config,
            export_xml=cached_xml,
            msk_key=msk_key,
            keep_temp=keep_temp,
        )
        _cache.write_meta(
            cache_dir,
            key,
            {
                "assist_sim_version": __version__,
                "human_xml": str(Path(human_xml).resolve()),
                "device_config": str(Path(device_config).resolve()),
                "msk_key": msk_key,
                "inputs": [str(p) for p in paths],
            },
        )
        if export_xml and export_xml != cached_xml:
            shutil.copyfile(cached_xml, export_xml)
        return model, data

    return ModelCombiner().combine(
        human_xml,
        config,
        export_xml=export_xml,
        msk_key=msk_key,
        keep_temp=keep_temp,
    )
