"""High-level loading utilities: a drop-in replacement for path resolution.

These wrap the autodiscovery registry so callers can work in terms of
``(msk_key, device_key)`` instead of file paths::

    from assist_sim import load_combined

    model, data = load_combined("myoLeg22_2D", "DephyExoBoot_L1")
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

    The ``msk_key`` is also forwarded so per-MSK config overrides apply.
    """
    human_xml, device_config = registry.resolve(msk_key, device_key)
    from . import load_combined_model  # lazy import avoids a circular import

    return load_combined_model(
        str(human_xml),
        str(device_config),
        export_xml=export_xml,
        msk_key=msk_key,
        cache_dir=cache_dir,
    )


def get_available_combinations() -> Dict[str, List[str]]:
    """Return ``{msk_key: [device_key, ...]}`` of discoverable combinations."""
    return registry.get_available_combinations()


def validate_combination(msk_key: str, device_key: str) -> bool:
    """Return True if the pair resolves and is compatible; else False."""
    return registry.validate_combination(msk_key, device_key)
