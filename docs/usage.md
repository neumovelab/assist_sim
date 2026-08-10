# Usage

This document gives the full public API. For the architectural background, see
[concepts.md](concepts.md).

## Python API

### `load_combined_model`: the low-level entry point

```python
from assist_sim import load_combined_model

model, data = load_combined_model(
    human_xml: str,                      # path to MSK XML (typically from myo_sim)
    device_config: str,                  # path to device's config.yaml
    export_xml: Optional[str] = None,    # if set, also writes the combined XML
    msk_key: Optional[str] = None,       # for per-MSK config overrides
    keep_temp: bool = False,             # leave preprocess temp files on disk
    cache_dir: Optional[Path] = None,    # opt-in local cache
) -> tuple[mj.MjModel, mj.MjData]
```

The function returns a compiled model and an `MjData` that starts from `qpos0`.
It does not change the baseline `human_xml` file on disk.

### `load_combined`: the registry-aware convenience function

```python
from assist_sim import load_combined

model, data = load_combined(
    msk_key: str,                        # e.g. "myolegs26"
    device_key: str,                     # e.g. "DephyExoBoot_L1"
    export_xml: Optional[str] = None,    # if set, also writes the combined XML
    cache_dir: str | Path | None = None,
) -> tuple[mj.MjModel, mj.MjData]
```

This function resolves both keys through the registry. `myo_sim` composes the
musculoskeletal (MSK) model on demand, and local autodiscovery finds the device
config. The function then runs the combination pipeline on the live spec. It
passes `msk_key` automatically, so that the resolver can apply the per-MSK
overrides. Use this entry point when `myo_sim` is installed.

### `load_msk`: the baseline MSK model, with no device

```python
from assist_sim import load_msk

model, data = load_msk(
    msk_key: str,                        # e.g. "myolegs26"
    export_xml: Optional[str] = None,
    cache_dir: str | Path | None = None,
) -> tuple[mj.MjModel, mj.MjData]
```

This function resolves and compiles the MSK model alone. Use it to give a
baseline model to a downstream consumer. It does not run the device steps of
the pipeline. It also does not decompose and rebuild the keyframes, because no
removal changes the qpos layout. Its export contains no terrain, like the
export of a combined model.

### `resolve_model_path`: resolve keys to a spec and a device-config path

```python
from assist_sim import resolve_model_path

human_spec, device_config = resolve_model_path(
    msk: str,            # e.g. "myolegs26"
    device: str,         # e.g. "DephyExoBoot_L1"
) -> tuple[mj.MjSpec, Path]
```

This function is a thin wrapper for `registry.resolve`. It returns the new
`MjSpec` for the MSK model, with the scene removed; `myo_sim` composes that
spec on demand. The function also returns the filesystem `Path` to the config
YAML file of the device. It does **not** compile, serialize, export, or return
the path of an XML file. It raises `ValueError` or `ImportError` for a key that
is unknown, incompatible, or impossible to build.

### `get_available_combinations`

```python
from assist_sim import get_available_combinations

combos = get_available_combinations()
# {'myolegs26': ['DephyExoBoot_L1', 'HMEDI_L1', ...], ...}
```

This function returns a dict of `msk_key -> [device_key, ...]`. It obeys the
optional `compatible_msk:` list of each device. It includes only the MSK models
that the installed `myo_sim` can resolve.

### `validate_combination`

```python
from assist_sim import validate_combination

assert validate_combination("myolegs26", "DephyExoBoot_L1")
```

This function returns `True` if the pair resolves and is compatible. It catches
the `ValueError`, `ImportError` and `FileNotFoundError` conditions.

### `DeviceConfig`

Construct a config directly in code, instead of a load from YAML:

```python
from assist_sim import DeviceConfig

config = DeviceConfig.from_yaml("models/HMEDI/L1config.yaml")
print(config.attachments)
print(config.resolve_attachments("myolegs"))   # per-MSK resolved
```

## CLI

```bash
python -m assist_sim combine <msk> <device> [-o PATH] [--cache-dir DIR]
python -m assist_sim list                       # all available combinations
python -m assist_sim --version
```

Examples:

```bash
# Combine + write combined XML
python -m assist_sim combine myolegs26 DephyExoBoot_L1 -o combined.xml

# List everything available
python -m assist_sim list

# Combine and cache (faster on subsequent runs)
python -m assist_sim combine myolegs OpenSourceLeg_KA_L1 --cache-dir ./.cache
```

## Registry

The registry has two parts:

- **MSK models** are an explicit selected set. See `_COMPATIBLE_MSK_KEYS` in
  `assist_sim/registry.py`. To add a new MSK model, edit that dict. Then
  confirm that the file exists in `myo_sim`.
- **Device configs**: the registry finds these automatically when it scans
  `models/*/*config.yaml`. If you add a new device directory with a
  `*config.yaml` file, the device is available at the next import. You do not
  edit the registry.

```python
from assist_sim.registry import (
    _COMPATIBLE_MSK_KEYS,     # {msk_key: _MskSource}  (curated MSK table)
    DEVICE_CONFIGS,           # {device_key: Path}
    resolve,                  # (msk, device) -> (human_spec: MjSpec, device_config: Path)
    refresh,                  # re-scan models/ for device configs
)
```

## The cache

The cache is optional. To enable it, specify `cache_dir=`:

```python
model, data = load_combined_model(
    human_xml=...,
    device_config=...,
    cache_dir=Path(".assist_sim_cache"),
)
```

**Cache key**: the SHA-1 hash of `(human_xml path, human_xml mtime,
device_config path, device_config mtime, device_model_xml path,
device_model_xml mtime, pipeline version, msk_key)`. Any change gives a cache
miss and a new compile.

`load_combined` and `load_msk` compose the MSK model in memory, so there is no
human XML file to fingerprint. Their key uses the device files, the `msk_key`,
and a `myo_sim` version token in place of the MSK file. For `load_msk` there
are no device files.

**On disk**:
- `<cache_dir>/<key>.xml`: the exported combined XML file
- `<cache_dir>/<key>.meta.json`: the input fingerprints, to help you debug
  stale entries

**There is no global cache.** The package does not write to `~/.cache/...`. To
clear the cache, use `rm -r <cache_dir>`.

**A new pipeline version makes all entries invalid.** The version constant is
`assist_sim.__version__`. Increase it in `assist_sim/__init__.py` when a change
in pipeline behavior changes the compiled output.

## Per-MSK overrides

A device YAML file can contain per-MSK variations for each section except
`actuators` and the legacy `keyframes`:

- `attachments`
- `equality`
- `joint_overrides`
- `keyframe_overrides`
- `body_removals`
- `geom_removals`
- `mesh_replacements`
- `actuator_removals`
- `tendon_removals`
- `tendon_modifications`
- `body_overrides`
- `actuator_overrides`
- `contact` (`pairs` + `excludes`)
- `sensors`
- `sensor_removals`

Schema:

```yaml
tendon_modifications:
  default:
    - name: hamstrings_r_tendon
      wraps: ...
  myolegs:
    - name: bflh_r_tendon
      wraps: ...
```

The resolver selects `default`, unless `msk_key` matches a specific entry. For
the sections that support this form, and for the dispatch procedure, see
[device-config-reference.md](device-config-reference.md#per-msk-overrides-summary).

## Errors

An unknown MSK key or device key raises `ValueError`. The error contains a "did
you mean ..." suggestion from `difflib.get_close_matches`. If `myo_sim` is not
installed, the code raises `ImportError`. An incompatible pair raises
`ValueError`, and the error lists the compatible MSK models. The
`compatible_msk:` list of the device sets the compatibility.

Config validation: if `combine()` finds a name that it cannot resolve in a
removal list, an attachment, or an override, it raises an error immediately.
The error gives the incorrect name, its section, and a suggestion. The function
always raises. It never gives a warning and continues.

For a separate check before the run, for example in a test:

```python
from assist_sim.validate import validate_config

issues = validate_config(human_xml, device_config)
assert not issues, issues
```
