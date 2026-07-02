"""MSK + device registry.

Two distinct discovery models:

- **MSK models** are an explicit, curated set (:data:`_COMPATIBLE_MSK_KEYS`),
  keyed to mirror the ``myo_sim`` model names.  ``myo_sim`` composes its leg
  models at runtime, so each key is resolved by calling
  ``myo_sim.build_spec(<model>)`` and stripping the bundled myosuite scene,
  returning an editable ``MjSpec`` that the pipeline mutates in place (surgery
  via ``spec.delete``) -- it is never serialized to XML.  ``myolegs26``
  (legs-only) and ``myolegs`` (80-muscle, passive torso) are buildable;
  ``myolegs22`` has no source yet (a planned 26->22 reduction).
- **Device configs** are autodiscovered by scanning ``models/<dir>/*config.yaml``
  in this repository.  Adding a new device dir with a config file makes it
  available next import; no registry edit required.

If ``myo_sim`` is not installed (or is too old to build a requested MSK),
:func:`resolve` raises :class:`ImportError` / :class:`ValueError` with a
pointer to the cause -- it never warns.
"""

from __future__ import annotations

import importlib.util
from importlib.resources import files as _files
from pathlib import Path
from typing import TYPE_CHECKING, Dict, List, NamedTuple, Optional, Tuple

import yaml

from .errors import closest_matches

if TYPE_CHECKING:
    import mujoco

REPO_ROOT = Path(__file__).resolve().parent.parent

# Device configs + meshes ship inside the package (so they're available after
# a wheel install, not just editable installs).  Resolve via importlib.resources
# so this works identically in editable, wheel, and zipped distributions.
MODELS_ROOT = Path(str(_files("assist_sim").joinpath("models")))

# ----------------------------------------------------------------------
# MSK registry (explicit, composed at runtime by myo_sim)
# ----------------------------------------------------------------------
# On myo_sim's mm_refactor branch the leg models are *composed* at runtime --
# there is no static XML on disk.  assist_sim obtains an editable MjSpec via
# ``myo_sim.build_spec(<model>)``, strips the bundled myosuite scene (outputs are
# model-only), and hands the live spec to the combination pipeline, which edits
# it in place (surgery via ``spec.delete``) and never serializes it.


class _MskSource(NamedTuple):
    """Binds an assist_sim MSK key to a myo_sim composed model.

    ``myo_sim_model`` is the ``myo_sim.build_spec`` name, or ``None`` when no
    source exists yet (planned work).  ``min_mujoco`` is the lowest MuJoCo
    version that can *build* it -- the passive-torso models need ``MjSpec.delete``
    (3.3.4+).  ``note`` explains a gated/planned state in the error the caller sees.
    """

    myo_sim_model: Optional[str]
    min_mujoco: Tuple[int, int, int]
    note: str


# Curated, not autodiscovered.  Keys are assist_sim-facing aliases; values bind
# them to the myo_sim composed models that back them.
_COMPATIBLE_MSK_KEYS: Dict[str, _MskSource] = {
    "myolegs22": _MskSource(
        None,
        (3, 3, 3),
        "myolegs22 will be derived from myolegs26 via a 26->22 mjspec reduction, which is not implemented yet",
    ),
    "myolegs26": _MskSource("myolegs26", (3, 3, 3), ""),
    "myolegs": _MskSource(
        "myolegs",
        (3, 3, 4),
        "the 80-muscle model's passive-torso conversion uses MjSpec.delete, which needs mujoco>=3.3.4",
    ),
}

_HAS_MYO_SIM = importlib.util.find_spec("myo_sim") is not None


def _mujoco_version() -> Tuple[int, ...]:
    """Installed MuJoCo version as an int tuple, e.g. ``(3, 3, 4)``."""
    import mujoco

    parts = tuple(int(p) for p in mujoco.__version__.split(".")[:3])
    return parts + (0,) * (3 - len(parts))


def _resolve_msk(key: str) -> "mujoco.MjSpec":
    """Compose an MSK key into a fresh, model-only ``MjSpec`` via myo_sim.

    Calls ``myo_sim.build_spec(<model>)`` and strips the bundled myosuite scene,
    returning an editable spec.  A fresh spec is built per call because the
    combination pipeline mutates it in place (surgery via ``spec.delete``).
    The result is never serialized -- torso-composed models don't round-trip
    through ``to_xml`` -- so the pipeline works on the live spec.

    Raises -- never warns:

    - ``ValueError`` for an unknown key (with a "did you mean" suggestion) or a
      key whose myo_sim source does not exist yet (planned work).
    - ``ImportError`` when myo_sim isn't installed, the installed MuJoCo is too
      old to build the model, or the build itself fails.
    """
    if key not in _COMPATIBLE_MSK_KEYS:
        suggestions = closest_matches(key, _COMPATIBLE_MSK_KEYS)
        hint = f" Did you mean {', '.join(repr(s) for s in suggestions)}?" if suggestions else ""
        raise ValueError(f"Unknown MSK model '{key}'. Available: {sorted(_COMPATIBLE_MSK_KEYS)}.{hint}")

    src = _COMPATIBLE_MSK_KEYS[key]
    if src.myo_sim_model is None:
        raise ValueError(f"MSK model '{key}' is not available yet: {src.note}.")

    try:
        import myo_sim
    except ImportError as exc:
        raise ImportError(
            f"The MSK model '{key}' is composed by the myo_sim package, which is "
            f"not installed. Install it with `pip install myo_sim` once published, "
            f"or follow the install instructions in the project README."
        ) from exc

    if _mujoco_version() < src.min_mujoco:
        import mujoco

        raise ImportError(f"MSK model '{key}' requires {src.note}; installed mujoco is {mujoco.__version__}.")

    from .utils import strip_myosuite_scene_spec

    try:
        spec = myo_sim.build_spec(src.myo_sim_model)
    except Exception as exc:  # noqa: BLE001 - surface any build failure as a clear ImportError
        raise ImportError(
            f"Failed to compose MSK model '{key}' via myo_sim.build_spec({src.myo_sim_model!r}): {type(exc).__name__}: {exc}"
        ) from exc
    strip_myosuite_scene_spec(spec)
    return spec


def _msk_available(key: str) -> bool:
    """Cheap check (no build) of whether :func:`_resolve_msk` would succeed.

    Used by :func:`get_available_combinations` so introspection never triggers a
    model compile.  Confirms the key has a myo_sim source, myo_sim is installed,
    the installed MuJoCo is new enough, and myo_sim knows the composed model.
    """
    src = _COMPATIBLE_MSK_KEYS.get(key)
    if src is None or src.myo_sim_model is None or not _HAS_MYO_SIM:
        return False
    if _mujoco_version() < src.min_mujoco:
        return False
    try:
        import myo_sim
    except ImportError:
        return False
    return src.myo_sim_model in getattr(myo_sim, "_COMPOSED_MODELS", frozenset())


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
    """Re-scan ``models/`` for device configs.

    MSK models are composed lazily by :func:`_resolve_msk` on request, so there
    is nothing MSK-side to populate here.
    """
    _scan_devices(MODELS_ROOT)


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
    hint = f" Did you mean {', '.join(repr(s) for s in suggestions)}?" if suggestions else ""
    raise ValueError(f"Unknown device '{device_key}'. Available: {sorted(DEVICE_CONFIGS)}.{hint}")


def _compatible(device_key: str, msk_key: str) -> bool:
    compat = _COMPATIBLE_MSK.get(device_key)
    return compat is None or msk_key in compat


def resolve(msk_key: str, device_key: str) -> Tuple["mujoco.MjSpec", Path]:
    """Resolve ``(msk_key, device_key)`` to ``(human_spec, device_config_path)``.

    The human MSK is returned as a freshly-composed, model-only ``MjSpec`` (see
    :func:`_resolve_msk`); the device config is a filesystem path.  Compatibility
    is checked before composing, so an incompatible pair fails fast.

    Raises:
        ValueError: if either key is unknown, the pair is incompatible, or the
            MSK has no myo_sim source yet.
        ImportError: if the MSK requires myo_sim but it isn't installed, the
            installed MuJoCo is too old, or the build fails.
    """
    key = _resolve_device_key(device_key)
    if not _compatible(key, msk_key):
        raise ValueError(
            f"Device '{device_key}' is not compatible with MSK '{msk_key}'. Compatible MSKs: {_COMPATIBLE_MSK.get(key)}"
        )
    human_spec = _resolve_msk(msk_key)
    return human_spec, DEVICE_CONFIGS[key]


def get_available_combinations() -> Dict[str, List[str]]:
    """Return ``{msk_key: [device_key, ...]}`` honoring compatibility.

    Only includes MSKs that are actually buildable in this environment (myo_sim
    installed, MuJoCo new enough, model known to myo_sim).  Gated/planned MSKs
    are silently omitted; call :func:`resolve` directly to surface the
    underlying error.  Uses a cheap availability check -- never composes a model.
    """
    result: Dict[str, List[str]] = {}
    for msk_key in sorted(_COMPATIBLE_MSK_KEYS):
        if not _msk_available(msk_key):
            continue
        devices = [dk for dk in sorted(DEVICE_CONFIGS) if _compatible(dk, msk_key)]
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
