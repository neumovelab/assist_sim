"""MSK + device registry.

Two distinct discovery models:

- **MSK models** are an explicit, curated set.  ``myo_sim`` upstream contains
  many MSK variants; only the ones listed in :data:`_COMPATIBLE_MSK_KEYS` are
  pipeline-compatible.  Paths are resolved via :mod:`importlib.resources` so
  the MSK files travel inside the ``myo_sim`` wheel rather than on a known
  filesystem location.
- **Device configs** are autodiscovered by scanning ``models/<dir>/*config.yaml``
  in this repository.  Adding a new device dir with a config file makes it
  available next import; no registry edit required.

If ``myo_sim`` is not installed (or doesn't yet ship the listed MSK files),
:func:`resolve` raises :class:`ImportError` / :class:`ValueError` with a
pointer to the install instructions.
"""

from __future__ import annotations

from importlib.resources import files as _files
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

from .errors import closest_matches

REPO_ROOT = Path(__file__).resolve().parent.parent
MODELS_ROOT = REPO_ROOT / "models"

# ----------------------------------------------------------------------
# MSK registry (explicit, resolved via myo_sim)
# ----------------------------------------------------------------------

# Maps registry keys to (myo_sim subpackage, filename) tuples.
# The eventual myo_sim wheel exposes these files via importlib.resources;
# we hold these as the authoritative list of pipeline-compatible MSKs.
_COMPATIBLE_MSK_KEYS: Dict[str, Tuple[str, str]] = {
    "myoLeg22_2D": ("myo_sim.leg", "myoLeg22_2D.xml"),
    "myoLeg26_3D": ("myo_sim.leg", "myoLeg26_3D.xml"),
    "myoLeg80":    ("myo_sim.leg", "myolegs.xml"),
}


# Populated on first access.
MSK_MODELS: Dict[str, Path] = {}


def _resolve_msk(key: str) -> Path:
    """Resolve an MSK key to an absolute filesystem path via myo_sim package."""
    if key not in _COMPATIBLE_MSK_KEYS:
        suggestions = closest_matches(key, _COMPATIBLE_MSK_KEYS)
        hint = (
            f" Did you mean {', '.join(repr(s) for s in suggestions)}?"
            if suggestions
            else ""
        )
        raise ValueError(
            f"Unknown MSK model '{key}'. "
            f"Available: {sorted(_COMPATIBLE_MSK_KEYS)}.{hint}"
        )

    pkg, filename = _COMPATIBLE_MSK_KEYS[key]
    try:
        resource = _files(pkg).joinpath(filename)
    except (ModuleNotFoundError, ImportError) as exc:
        raise ImportError(
            f"The MSK model '{key}' lives in the myo_sim package, which is "
            f"not installed (looked for {pkg}.{filename}). "
            f"Install it with `pip install myo_sim` once published, or follow "
            f"the install instructions in the project README."
        ) from exc

    path = Path(str(resource))
    if not path.exists():
        raise FileNotFoundError(
            f"MSK model file missing inside myo_sim: {pkg}/{filename} "
            f"(resolved to {path}). The installed myo_sim version may not "
            f"include this MSK yet."
        )
    return path


def _populate_msk_cache() -> None:
    """Best-effort fill of MSK_MODELS for introspection.  Tolerates missing myo_sim."""
    MSK_MODELS.clear()
    for key in _COMPATIBLE_MSK_KEYS:
        try:
            MSK_MODELS[key] = _resolve_msk(key)
        except (ImportError, FileNotFoundError, ValueError):
            # myo_sim absent or file missing -- skip silently; resolve() raises
            # the informative error when this key is actually requested.
            continue


# ----------------------------------------------------------------------
# Device registry
# ----------------------------------------------------------------------

DEVICE_CONFIGS: Dict[str, Path] = {}

# device key -> device.name; and alias (device.name) -> primary key.
_DEVICE_NAMES: Dict[str, str] = {}
_DEVICE_ALIASES: Dict[str, str] = {}
_COMPATIBLE_MSK: Dict[str, Optional[List[str]]] = {}


def _device_key(config_path: Path) -> str:
    """Derive a device registry key from its config filename.

    ``models/DephyExoBoot/L1config.yaml`` -> ``DephyExoBoot_L1``.
    """
    parent = config_path.parent.name
    stem = config_path.stem  # e.g. "L1config", "A_L1config"
    base = stem[: -len("config")] if stem.endswith("config") else stem
    base = base.rstrip("_")
    return f"{parent}_{base}" if base else parent


def _scan_devices(models_root: Path) -> None:
    DEVICE_CONFIGS.clear()
    _DEVICE_NAMES.clear()
    _DEVICE_ALIASES.clear()
    _COMPATIBLE_MSK.clear()

    if not models_root.exists():
        return

    for sub in sorted(p for p in models_root.iterdir() if p.is_dir()):
        for config_path in sorted(sub.glob("*config.yaml")):
            key = _device_key(config_path)
            DEVICE_CONFIGS[key] = config_path
            try:
                raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
                device = raw.get("device", {})
                name = device.get("name")
                compat = device.get("compatible_msk")
            except Exception:  # noqa: BLE001 - discovery must not crash
                name, compat = None, None
            _COMPATIBLE_MSK[key] = compat
            if name:
                _DEVICE_NAMES[key] = name
                if name not in DEVICE_CONFIGS:
                    _DEVICE_ALIASES[name] = key


def refresh() -> None:
    """Re-scan ``models/`` and re-resolve MSK paths via myo_sim."""
    _scan_devices(MODELS_ROOT)
    _populate_msk_cache()


# ----------------------------------------------------------------------
# Resolution + queries
# ----------------------------------------------------------------------

def _resolve_device_key(device_key: str) -> str:
    if device_key in DEVICE_CONFIGS:
        return device_key
    if device_key in _DEVICE_ALIASES:
        return _DEVICE_ALIASES[device_key]
    candidates = list(DEVICE_CONFIGS) + list(_DEVICE_ALIASES)
    suggestions = closest_matches(device_key, candidates)
    hint = (
        f" Did you mean {', '.join(repr(s) for s in suggestions)}?"
        if suggestions
        else ""
    )
    raise ValueError(
        f"Unknown device '{device_key}'. Available: {sorted(DEVICE_CONFIGS)}.{hint}"
    )


def _compatible(device_key: str, msk_key: str) -> bool:
    compat = _COMPATIBLE_MSK.get(device_key)
    return compat is None or msk_key in compat


def resolve(msk_key: str, device_key: str) -> Tuple[Path, Path]:
    """Resolve ``(msk_key, device_key)`` to ``(human_xml, device_config)`` paths.

    Raises:
        ValueError: if either key is unknown or the pair is incompatible.
        ImportError: if the resolved MSK requires myo_sim but it isn't installed.
        FileNotFoundError: if myo_sim is installed but doesn't include this MSK.
    """
    msk_path = _resolve_msk(msk_key)
    key = _resolve_device_key(device_key)
    if not _compatible(key, msk_key):
        raise ValueError(
            f"Device '{device_key}' is not compatible with MSK '{msk_key}'. "
            f"Compatible MSKs: {_COMPATIBLE_MSK.get(key)}"
        )
    return msk_path, DEVICE_CONFIGS[key]


def get_available_combinations() -> Dict[str, List[str]]:
    """Return ``{msk_key: [device_key, ...]}`` honoring compatibility.

    Only includes MSKs whose files are actually resolvable through the
    installed myo_sim package.  Missing MSKs are silently omitted; call
    :func:`resolve` directly to surface the underlying error.
    """
    result: Dict[str, List[str]] = {}
    for msk_key in sorted(MSK_MODELS):
        devices = [
            dk for dk in sorted(DEVICE_CONFIGS) if _compatible(dk, msk_key)
        ]
        result[msk_key] = devices
    return result


def validate_combination(msk_key: str, device_key: str) -> bool:
    """Return True if the pair resolves and is compatible; else False."""
    try:
        resolve(msk_key, device_key)
        return True
    except (ValueError, ImportError, FileNotFoundError):
        return False


# Populate at import.
refresh()
