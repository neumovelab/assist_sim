# Usage

The full public API surface. See [concepts.md](concepts.md) for the
architectural background.

## Python API

### `load_combined_model` — the low-level entrypoint

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

Returns a compiled model + an `MjData` initialized from `qpos0`. The
baseline `human_xml` is never modified on disk.

### `load_combined` — registry-aware convenience

```python
from assist_sim import load_combined

model, data = load_combined(
    msk: str,            # e.g. "myoLeg22_2D"
    device: str,         # e.g. "DephyExoBoot_L1"
    cache_dir: Path | None = None,
) -> tuple[mj.MjModel, mj.MjData]
```

Resolves both keys through the registry (MSK path via `myo_sim`, device
path via local autodiscovery) and calls `load_combined_model` under the
hood. Auto-passes `msk_key=msk` for per-MSK override resolution. The
preferred entrypoint once `myo_sim` is installed.

### `resolve_model_path` — compile + export, return XML path

```python
from assist_sim import resolve_model_path

path = resolve_model_path(
    msk: str,
    device: str,
    cache_dir: Path | None = None,
    export_dir: Path | None = None,
) -> str
```

For callers (e.g. RL training configs) that want a file path to a combined
XML. Compiles the combination, writes it to a cache or export directory,
and returns the absolute path.

### `get_available_combinations`

```python
from assist_sim import get_available_combinations

combos = get_available_combinations()
# {'myoLeg22_2D': ['DephyExoBoot_L1', 'HMEDI_L1', ...], ...}
```

Returns a dict of `msk_key -> [device_key, ...]` honoring each device's
optional `compatible_msk:` list. Only includes MSKs whose files are
resolvable through the installed `myo_sim`.

### `validate_combination`

```python
from assist_sim import validate_combination

assert validate_combination("myoLeg22_2D", "DephyExoBoot_L1")
```

Returns `True` if the pair resolves and is compatible. Catches the various
`ValueError` / `ImportError` / `FileNotFoundError` paths.

### `DeviceConfig`

Direct programmatic config construction (instead of loading from YAML):

```python
from assist_sim import DeviceConfig

config = DeviceConfig.from_yaml("models/HMEDI/L1config.yaml")
print(config.attachments)
print(config.resolve_attachments("myoLeg80"))   # per-MSK resolved
```

## CLI

```bash
python -m assist_sim compile <msk> <device> [--export PATH] [--cache DIR]
python -m assist_sim list                       # all available combinations
python -m assist_sim --version
```

Examples:

```bash
# Compile + write combined XML
python -m assist_sim compile myoLeg22_2D DephyExoBoot_L1 --export combined.xml

# List everything available
python -m assist_sim list

# Compile and cache (faster on subsequent runs)
python -m assist_sim compile myoLeg80 OpenSourceLeg_KA_L1 --cache ./.cache
```

## Registry

The registry has two halves:

- **MSK models** are an explicit curated set (see `_COMPATIBLE_MSK_KEYS` in
  `assist_sim/registry.py`). Adding a new MSK requires editing that dict
  and confirming the file exists in `myo_sim`.
- **Device configs** are autodiscovered by scanning `models/*/config.yaml`.
  Adding a new device dir with a `*config.yaml` makes it available next
  import — no registry edit needed.

```python
from assist_sim.registry import (
    MSK_MODELS,               # {msk_key: resolved Path}
    DEVICE_CONFIGS,           # {device_key: Path}
    resolve,                  # (msk, device) -> (msk_path, device_path)
    refresh,                  # re-scan models/ and re-resolve via myo_sim
)
```

## Caching

Caching is opt-in via `cache_dir=`:

```python
model, data = load_combined_model(
    human_xml=...,
    device_config=...,
    cache_dir=Path(".assist_sim_cache"),
)
```

**Cache key**: SHA-1 of `(human_xml path, human_xml mtime, device_config path,
device_config mtime, device_model_xml path, device_model_xml mtime,
pipeline version, msk_key)`. Any change → cache miss → fresh compile.

**On disk**:
- `<cache_dir>/<key>.xml` — the exported combined XML
- `<cache_dir>/<key>.meta.json` — input fingerprints for debugging stale entries

**No global cache.** No `~/.cache/...` magic. Users who want to evict can
`rm -r <cache_dir>`.

**Pipeline version bumps** invalidate all entries. The version constant
lives at `assist_sim.__version__`; bump it in `assist_sim/__init__.py`
whenever pipeline behavior changes affect compiled output.

## Per-MSK overrides

Device YAMLs can express per-MSK variations for any of:

- `attachments`
- `tendon_modifications`
- `keyframe_overrides`
- `actuator_removals`
- `tendon_removals`
- `mesh_replacements`

Schema:

```yaml
tendon_modifications:
  default:
    - name: gastroc_r_tendon
      wraps: ...
  myoLeg80:
    - name: gasmed_r_tendon
      wraps: ...
```

The resolver picks `default` unless `msk_key` matches a specific entry.
See [device-config-reference.md](device-config-reference.md#per-msk-overrides)
for which sections support this and how the dispatch works.

## Error handling

Unknown MSK / device keys raise `ValueError` with a "did you mean ..."
suggestion via `difflib.get_close_matches`. Missing `myo_sim` raises
`ImportError`. Incompatible pairs (per the device's `compatible_msk:` list)
raise `ValueError` listing the compatible MSKs.

Config validation: when `combine()` encounters an unresolved name in any
removal list, attachment, or override, it raises immediately with the bad
name, the section it came from, and a suggestion. No silent warnings.

For a separate pre-flight check (e.g. in tests):

```python
from assist_sim.validate import validate_config

issues = validate_config(human_xml, device_config)
assert not issues, issues
```
