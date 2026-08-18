"""MSK + device registry.

Two distinct discovery models:

- **MSK models** are an explicit, curated set (:data:`_COMPATIBLE_MSK_KEYS`),
  keyed to mirror the ``myo_sim`` model names.  ``myo_sim`` composes its leg
  models at runtime, so each key is resolved by calling
  ``myo_sim.load_spec(<model>)`` and stripping the bundled myosuite scene,
  returning an editable ``MjSpec`` that the pipeline mutates in place (surgery
  via ``spec.delete``) -- it is never serialized to XML.  ``myolegs26``
  (26-muscle, torso'd) and ``myolegs`` (80-muscle, passive torso) are buildable;
  ``myolegs22`` (planar 22-muscle) is derived from ``myolegs26`` by an in-spec
  26->22 reduction (:func:`assist_sim.reduce_legs.reduce_myolegs26_to_22`).
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
# In myo_sim the leg models are *composed* at runtime -- there is no static XML
# on disk.  assist_sim obtains an editable MjSpec via
# ``myo_sim.load_spec(<model>)``, strips the bundled myosuite scene (outputs are
# model-only), and hands the live spec to the combination pipeline, which edits
# it in place (surgery via ``spec.delete``) and never serializes it.


class _MskSource(NamedTuple):
    """Binds an assist_sim MSK key to a myo_sim composed model.

    ``myo_sim_model`` is the ``myo_sim.load_spec`` name, or ``None`` when no
    source exists yet (planned work).  ``min_mujoco`` is the lowest MuJoCo
    version that can *build* it.  ``note`` explains a gated/planned state in the
    error the caller sees.  ``reduce_to_22`` marks a key whose composed spec is
    post-processed by the 26->22 planar reduction before the scene strip (only
    ``myolegs22`` today).
    """

    myo_sim_model: Optional[str]
    min_mujoco: Tuple[int, int, int]
    note: str
    reduce_to_22: bool = False


# The MuJoCo floor every current key needs: MjSpec.delete for the in-memory surgery.
# It equals the package floor (``mujoco>=3.4`` in pyproject.toml), so the version gate
# in _resolve_msk / _msk_available cannot trip on a resolvable install.  The mechanism
# is kept deliberately, for a future MSK that needs a *newer* MuJoCo than the package
# floor -- gating one key is then better than raising the floor for everyone.  (The
# earlier per-key 3.3.3 / 3.3.4 values predated the 3.4 floor and were unreachable:
# MjSpec.delete landed in 3.3.4, but nothing can install with a MuJoCo that old.)
_MIN_MUJOCO: Tuple[int, int, int] = (3, 4, 0)
_MIN_MUJOCO_NOTE = "the in-memory surgery uses MjSpec.delete, which needs mujoco>=3.4"

# Curated, not autodiscovered.  Keys are assist_sim-facing aliases; values bind
# them to the myo_sim composed models that back them.
_COMPATIBLE_MSK_KEYS: Dict[str, _MskSource] = {
    "myolegs22": _MskSource("myolegs26", _MIN_MUJOCO, _MIN_MUJOCO_NOTE, reduce_to_22=True),
    "myolegs26": _MskSource("myolegs26", _MIN_MUJOCO, _MIN_MUJOCO_NOTE),
    "myolegs": _MskSource("myolegs", _MIN_MUJOCO, _MIN_MUJOCO_NOTE),
    "myofullbody": _MskSource("myofullbody", _MIN_MUJOCO, _MIN_MUJOCO_NOTE),
}

_HAS_MYO_SIM = importlib.util.find_spec("myo_sim") is not None


def _mujoco_version() -> Tuple[int, ...]:
    """Installed MuJoCo version as an int tuple, e.g. ``(3, 3, 4)``."""
    import mujoco

    parts = tuple(int(p) for p in mujoco.__version__.split(".")[:3])
    return parts + (0,) * (3 - len(parts))


def _resolve_msk(key: str) -> "mujoco.MjSpec":
    """Compose an MSK key into a fresh, model-only ``MjSpec`` via myo_sim.

    Calls ``myo_sim.load_spec(<model>)`` and strips the bundled myosuite scene,
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
            f"not installed. Install it with `pip install myo-sim`, "
            f"or follow the install instructions in the project README."
        ) from exc

    if _mujoco_version() < src.min_mujoco:
        import mujoco

        raise ImportError(f"MSK model '{key}' requires {src.note}; installed mujoco is {mujoco.__version__}.")

    from .utils import strip_myosuite_scene_spec

    try:
        spec = myo_sim.load_spec(src.myo_sim_model)
        if src.reduce_to_22:
            from .reduce_legs import reduce_myolegs26_to_22

            reduce_myolegs26_to_22(spec)
    except Exception as exc:  # noqa: BLE001 - surface any build failure as a clear ImportError
        raise ImportError(
            f"Failed to compose MSK model '{key}' via myo_sim.load_spec({src.myo_sim_model!r}): {type(exc).__name__}: {exc}"
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
    return src.myo_sim_model in _composed_models(myo_sim)


def _composed_models(myo_sim) -> frozenset:
    """The model names myo_sim can compose, read from its ``_COMPOSED_MODELS`` attribute.

    That attribute is *private* to myo_sim, so it is a cross-repo contract that an
    upstream rename can break.  Reading it through ``getattr(..., frozenset())`` made the
    break invisible: every key looked unavailable, :func:`get_available_combinations`
    returned ``{}``, and both the CLI and myoassist then reported "no combinations
    buildable -- is myo_sim installed?" while myo_sim was installed and working fine.

    Raise instead.  A missing attribute means an incompatible myo_sim, which is a
    different thing from an empty registry, and assist_sim errors rather than degrading.
    """
    models = getattr(myo_sim, "_COMPOSED_MODELS", None)
    if models is None:
        raise ImportError(
            f"The installed myo_sim ({getattr(myo_sim, '__version__', 'unknown')}) does not expose "
            "'_COMPOSED_MODELS', which assist_sim reads to discover the composable MSK models. "
            "The myo_sim interface has changed; install a compatible myo-sim (>=0.2.1)."
        )
    return frozenset(models)


# ----------------------------------------------------------------------
# Device registry
# ----------------------------------------------------------------------

DEVICE_CONFIGS: Dict[str, Path] = {}

# device key -> device.name; and alias (device.name) -> primary key.
_DEVICE_NAMES: Dict[str, str] = {}
_DEVICE_ALIASES: Dict[str, str] = {}
_COMPATIBLE_MSK: Dict[str, Optional[List[str]]] = {}

# device key -> the parse error its config raised during discovery.  Discovery runs at
# import time and must not crash (a broken config would otherwise make `import
# assist_sim` -- and so the CLI you would use to diagnose it -- fail).  So the error is
# recorded here and raised when someone actually resolves that device, instead of being
# swallowed: a malformed config used to register silently with compatible_msk=None, which
# reads as "compatible with every MSK".
_DEVICE_ERRORS: Dict[str, Exception] = {}


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
    _DEVICE_ERRORS.clear()

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
            except Exception as exc:  # noqa: BLE001 - discovery must not crash; see _DEVICE_ERRORS
                _DEVICE_ERRORS[key] = exc
                name, compat = None, None
            _COMPATIBLE_MSK[key] = compat
            if name:
                _DEVICE_NAMES[key] = name
                # ``device.name`` doubles as an alias and as the namespace prefix the combined
                # model uses, so two devices sharing one is not a cosmetic clash: the second
                # would silently lose its alias here, and both would prefix their elements
                # identically.  Record it as a discovery error rather than dropping it, so
                # resolving either device says so instead of one quietly winning.
                clash = _DEVICE_ALIASES.get(name)
                if clash is not None and clash != key:
                    err = ValueError(
                        f"devices '{clash}' and '{key}' both declare device.name '{name}', which is "
                        f"also the namespace prefix for their elements. Give each a unique name."
                    )
                    _DEVICE_ERRORS.setdefault(clash, err)
                    _DEVICE_ERRORS.setdefault(key, err)
                elif name not in DEVICE_CONFIGS:
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
    key = None
    if device_key in DEVICE_CONFIGS:
        key = device_key
    elif device_key in _DEVICE_ALIASES:
        key = _DEVICE_ALIASES[device_key]
    if key is not None:
        # Surface a config that failed to parse during discovery, at the point of use.
        exc = _DEVICE_ERRORS.get(key)
        if exc is not None:
            raise ValueError(
                f"Device '{key}' has an unreadable config at {DEVICE_CONFIGS[key]}: {type(exc).__name__}: {exc}"
            ) from exc
        return key
    candidates = list(DEVICE_CONFIGS) + list(_DEVICE_ALIASES)
    suggestions = closest_matches(device_key, candidates)
    hint = f" Did you mean {', '.join(repr(s) for s in suggestions)}?" if suggestions else ""
    raise ValueError(f"Unknown device '{device_key}'. Available: {sorted(DEVICE_CONFIGS)}.{hint}")


def _compatible(device_key: str, msk_key: str) -> bool:
    """Whether *device_key* declares compatibility with *msk_key*.

    ``compatible_msk`` absent (``None``) means "every MSK".  A bare string is rejected
    rather than treated as a sequence: ``msk_key in "myolegs26"`` is a substring test, so
    ``compatible_msk: myolegs26`` would also match ``myolegs``.
    """
    compat = _COMPATIBLE_MSK.get(device_key)
    if compat is None:
        return True
    if isinstance(compat, str):
        raise ValueError(
            f"Device '{device_key}' declares 'compatible_msk: {compat}' as a bare string; it must be a "
            f"list, e.g. [{compat}]. A string would be matched by substring, so 'myolegs' would "
            f"wrongly match 'myolegs26'."
        )
    return msk_key in compat


def resolve_device_config(msk_key: str, device_key: str) -> Path:
    """Validate ``(msk_key, device_key)`` and return the device config path.

    The cheap half of :func:`resolve`: it checks that both keys are known, that the device's
    config parsed during discovery, and that the pair is compatible -- **without** composing
    the MSK.  The caching path in :mod:`assist_sim.loading` needs exactly this, so that a
    cache hit does not pay for a compose it is about to throw away.
    """
    if msk_key not in _COMPATIBLE_MSK_KEYS:
        suggestions = closest_matches(msk_key, _COMPATIBLE_MSK_KEYS)
        hint = f" Did you mean {', '.join(repr(s) for s in suggestions)}?" if suggestions else ""
        raise ValueError(f"Unknown MSK model '{msk_key}'. Available: {sorted(_COMPATIBLE_MSK_KEYS)}.{hint}")
    key = _resolve_device_key(device_key)
    if not _compatible(key, msk_key):
        raise ValueError(
            f"Device '{device_key}' is not compatible with MSK '{msk_key}'. Compatible MSKs: {_COMPATIBLE_MSK.get(key)}"
        )
    return DEVICE_CONFIGS[key]


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
    config_path = resolve_device_config(msk_key, device_key)
    return _resolve_msk(msk_key), config_path


def get_available_combinations() -> Dict[str, List[str]]:
    """Return ``{msk_key: [device_key, ...]}`` honoring compatibility.

    Only includes MSKs that are actually buildable in this environment (myo_sim
    installed, MuJoCo new enough, model known to myo_sim).  Gated/planned MSKs
    are silently omitted; call :func:`resolve` directly to surface the
    underlying error.  Uses a cheap availability check -- never composes a model.

    Devices whose config failed to parse are omitted too, rather than listed as though
    they worked; :func:`resolve` raises for those with the parse error (see
    :data:`_DEVICE_ERRORS`).  An incompatible *myo_sim* is not degraded to an empty
    result -- :func:`_composed_models` raises for that.
    """
    result: Dict[str, List[str]] = {}
    usable = [dk for dk in sorted(DEVICE_CONFIGS) if dk not in _DEVICE_ERRORS]
    for msk_key in sorted(_COMPATIBLE_MSK_KEYS):
        if not _msk_available(msk_key):
            continue
        result[msk_key] = [dk for dk in usable if _compatible(dk, msk_key)]
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
