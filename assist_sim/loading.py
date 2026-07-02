"""High-level loading utilities: a drop-in replacement for path resolution.

These wrap the autodiscovery registry so callers can work in terms of
``(msk_key, device_key)`` instead of file paths::

    from assist_sim import load_combined

    model, data = load_combined("myolegs26", "DephyExoBoot_L1")
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union

import mujoco as mj

from . import registry


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
    if cache_dir is not None:
        raise NotImplementedError(
            "cache_dir is not supported for composed (registry-key) MSKs yet -- the combined "
            "torso'd models don't round-trip through XML. Use load_combined_model with an explicit "
            "human_xml path if you need caching."
        )

    from .combine import ModelCombiner  # lazy import avoids a circular import
    from .config import DeviceConfig

    human_spec, device_config_path = registry.resolve(msk_key, device_key)
    config = DeviceConfig.from_yaml(str(device_config_path))
    return ModelCombiner().combine(human_spec, config, export_xml=export_xml, msk_key=msk_key)


def get_available_combinations() -> Dict[str, List[str]]:
    """Return ``{msk_key: [device_key, ...]}`` of discoverable combinations."""
    return registry.get_available_combinations()


def validate_combination(msk_key: str, device_key: str) -> bool:
    """Return True if the pair resolves and is compatible; else False."""
    return registry.validate_combination(msk_key, device_key)
