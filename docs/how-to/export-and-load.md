# How To: Export and Load Combined Models

When you want a *file* on disk (not just an in-memory `MjModel`) — for
sharing, for inspection in `simulate.exe`, for consumption by a tool
that expects a path. This guide covers the export options and the
re-load story.

## Export from Python

```python
from assist_sim import load_combined_model

model, data = load_combined_model(
    human_xml="path/to/myo_sim/leg/myoLeg22_2D.xml",
    device_config="models/DephyExoBoot/L1config.yaml",
    export_xml="combined.xml",        # also write to disk
)
```

This compiles the model in memory AND writes a clean XML to
`combined.xml`. The compiled `model` and `data` objects are returned
unchanged from the no-export case.

## Export from the CLI

```bash
python -m assist_sim compile myoLeg22_2D DephyExoBoot_L1 --export combined.xml
```

Identical behavior to the Python form. Prints the resulting `(nq, nu,
nbody, nmesh)` for sanity.

## What's in the exported XML

The exported XML is **model-only**:

- ✓ Combined body hierarchy (MSK + device, with device prefix)
- ✓ Meshes (deduplicated, paths rewritten relative to the export
  file's directory)
- ✓ Joints, actuators, tendons (MSK's + device's)
- ✓ Keyframes (with overrides applied, pruned for any removed joints)
- ✓ Visual block (lighting from the MSK + a minimal fallback if absent)
- ✓ Skybox texture, texfloor texture, matfloor material (kept *only*
  because MuJoCo's renderer needs them present for the skybox to draw;
  no ground body / no geom references them)

What's **not** in the exported XML:

- ✗ Ground body / ground-plane geom (stripped)
- ✗ Terrain include directives (stripped)
- ✗ Contact pairs referencing removed terrain geoms (scrubbed)

Downstream consumers (`myoassist`, `myoassist.terrains`) layer the
scene on top.

## Re-loading an exported XML

```python
import mujoco as mj

model = mj.MjModel.from_xml_path("combined.xml")
data = mj.MjData(model)
```

This is just standard MuJoCo. The exported XML is a self-contained model
(modulo the mesh files, which are referenced by relative path from the
XML's directory).

## Re-loading after moving the XML

The exported XML uses relative paths to its mesh files. If you move
`combined.xml` somewhere else, you need to either:

1. Move the mesh directory tree along with it (preserving the relative
   layout), or
2. Rewrite the mesh paths in the XML, or
3. Re-export from the original config to the new location

Option 3 is the cleanest. The pipeline rewrites mesh paths during export
to be relative to whatever the `export_xml=` target is.

## Caching (faster repeat loads)

Opt-in via `cache_dir=`:

```python
model, data = load_combined_model(
    human_xml=...,
    device_config=...,
    cache_dir="./.assist_sim_cache",
)
```

First call: full pipeline → writes a cached XML + meta.json keyed on the
inputs.

Second call (unchanged inputs): cache hit → loads the cached XML
directly. Significantly faster.

Edit any input file → cache miss → fresh compile.

Cache files:

- `<cache_dir>/<sha1_key>.xml` — the combined XML
- `<cache_dir>/<sha1_key>.meta.json` — input paths + mtimes (for
  debugging which entry is which)

**Invalidation rules:**
- Any input file mtime change → cache miss
- `assist_sim.__version__` bump → all entries miss

**Eviction:** `rm -r <cache_dir>` whenever. No background cleanup, no
size cap. Caching is a single-user local optimization, not a service.

## Common workflows

### Iterate on a YAML; want fast feedback

```bash
# First run: full compile
python examples/quickstart.py myoLeg22_2D MyDevice_L1
# Edit models/MyDevice/L1config.yaml
# Re-run: cache miss (YAML mtime changed) → fresh compile
python examples/quickstart.py myoLeg22_2D MyDevice_L1
```

### Generate a combined XML for use in another tool

```bash
python -m assist_sim compile myoLeg22_2D DephyExoBoot_L1 \
    --export /path/to/other_project/models/combined.xml
```

The other tool loads the XML directly via `MjModel.from_xml_path`.

### Pre-warm a cache for a CI run

```python
from assist_sim import get_available_combinations, load_combined

for msk, devs in get_available_combinations().items():
    for dev in devs:
        load_combined(msk, dev, cache_dir="./.cache")
```

Cold one-time pass; subsequent CI invocations hit cache.

## See also

- [usage.md](../usage.md) — the full API
- [troubleshooting.md](../troubleshooting.md) — mesh-path / cache-stale issues
