# How To: Export and Load Combined Models

Sometimes you need a *file* on disk, not only an `MjModel` object in
memory. Examples are a file to share, a file to examine in `simulate.exe`,
and a file for a tool that needs a path. This guide covers the export
options and the reload options.

## Export from Python

Use the registry key, which composes the musculoskeletal (MSK) model
through `myo_sim`:

```python
from assist_sim import load_combined

model, data = load_combined(
    "myolegs26",
    "DephyExoBoot_L1",
    export_xml="combined.xml",        # also write to disk
)
```

Or use an MSK XML that you already have on disk:

```python
from assist_sim import load_combined_model

model, data = load_combined_model(
    human_xml="path/to/myolegs26.xml",
    device_config="assist_sim/models/DephyExoBoot/L1config.yaml",
    export_xml="combined.xml",
)
```

Both functions compile the model in memory AND write a clean XML file to
`combined.xml`. The `model` and `data` objects are the same as the objects
from a call with no export.

## Export from the command line

```bash
python -m assist_sim combine myolegs26 DephyExoBoot_L1 -o combined.xml
```

The behavior is the same as the Python form. The command prints the
resulting `(nq, nu, nbody, nmesh)` for a quick check.

## Export a baseline MSK model with no device

`load_msk` is the equivalent of `load_combined` for a model with no device.
It does not run the combination pipeline. There are no removals, no
attachment and no keyframe rebuild, because nothing changes the qpos
layout. `load_msk` goes directly from the composed spec to a compile:

```python
from assist_sim import load_msk

model, data = load_msk("myolegs26", export_xml="baseline.xml")
```

```bash
python -m assist_sim msk myolegs26 -o baseline.xml
```

`load_msk` accepts `cache_dir=` on the same terms as `load_combined`. Its
export uses the same writer, so all the text below also applies to it. Use
`load_msk` to give a bare MSK model to a downstream tool. Also use it to
get an XML file for the config validator, which needs a file on disk.

## Content of the exported XML

The export carries the model plus a minimal visual block. It is not a
scene, and it is not bare either:

- ✓ The combined body hierarchy (the MSK model and the device, with the
  device prefix)
- ✓ The meshes. The export removes the duplicates. It also makes the paths
  relative to the directory of the export file.
- ✓ The joints, actuators and tendons of the MSK model and of the device
- ✓ The keyframes. The export applies the overrides. It also removes the
  slots of any removed joint.
- ✓ A soft headlight plus a skybox texture. The export keeps the skybox of
  the model if the model has one, else it adds a neutral gradient.
  `_ensure_minimal_visual` adds the item that is absent, so the file
  renders correctly when you open it on its own. You can override both
  items. The `<headlight>` and the skybox of a downstream scene win when
  you put that scene on top.

The exported XML does **not** contain these items:

- ✗ The ground plane, backdrop, pedestal, logo, scene lights and cameras.
  `strip_myosuite_scene_spec` removes the bundled myosuite scene from the
  composed spec at resolve time, before the pipeline sees it. None of the
  scene therefore reaches the export.
- ✗ The `<headlight>` and `<global>` camera settings of the MSK model.
  `_strip_scene_visual` removes them, so they cannot merge attribute by
  attribute with the lighting of a downstream scene.
- ✗ The scene textures and materials that no geom refers to.
  `_strip_orphan_scene_assets` removes them. Their `scene/*.png` paths
  would otherwise be invalid and cause an error on the reload.

An exported model therefore has no ground. Downstream tools
(`myoassist`, `myoassist.terrains`) put the ground and the lighting on top.

`utils.py` contains a separate terrain remover, `_strip_terrain`. It
applies to an MSK model that pulls in a terrain XML through an `<include>`.
It runs only when you give `terrain_paths=` to `export_combined_xml`. No
bundled model needs it, because the composed MSK models carry no terrain
include, and their scene is already absent at resolve time. `_strip_terrain`
stays for callers that give terrain paths, but the bundled models do not
use it.

## Reload an exported XML

```python
import mujoco as mj

model = mj.MjModel.from_xml_path("combined.xml")
data = mj.MjData(model)
```

This is standard MuJoCo. The exported XML is a self-contained model, if you
keep the mesh files with it. The XML refers to the meshes by a path that is
relative to its own directory.

## Reload after you move the XML

The exported XML uses relative paths to its mesh files. If you move
`combined.xml` to a different location, do one of these three tasks:

1. Move the mesh directory tree with the XML. Keep the relative layout.
2. Change the mesh paths in the XML.
3. Export again from the original config to the new location.

Option 3 is the simplest. During the export, the pipeline makes the mesh
paths relative to the `export_xml=` target.

## Caching

The cache is optional and off by default. Set `cache_dir=` to use it. All three entry points
accept it:

```python
model, data = load_combined("myolegs26", "DephyExoBoot_L1", cache_dir="./.assist_sim_cache")
model, data = load_msk("myolegs26", cache_dir="./.assist_sim_cache")
model, data = load_combined_model(human_xml=..., device_config=..., cache_dir="./.assist_sim_cache")
```

The first call runs the full pipeline and writes a cached XML plus a `meta.json`. A later call
with the same inputs is a hit: it skips the MSK compose **and** the combine, and reloads the
exported XML instead. If you change any input, the next call misses and rebuilds.

### Whether it pays depends on the model

A hit costs one XML parse, and parsing is what dominates. So the benefit scales inversely with
the size of the exported model (best of five, one machine -- treat the ratios as the signal,
not the absolute times):

| MSK | miss | hit | |
|---|--:|--:|---|
| `myolegs22` | 0.64 s | 0.08 s | 8.2x faster |
| `myolegs26` | 0.52 s | 0.19 s | 2.8x faster |
| `myolegs` | 0.59 s | 0.27 s | 2.2x faster |
| `myofullbody` | 0.99 s | 1.11 s | **0.9x -- slower** |

`myofullbody` exports 0.6 MB of MJCF (418 actuators, 108 meshes), and parsing that costs more
than composing it from scratch. **Do not set `cache_dir` for `myofullbody`.** For the three leg
models, do.

### If you are training, use it

The model is composed far more often than once per run. `myoassist` builds one per
CMA-ES candidate and one per `SubprocVecEnv` worker, so a controller-optimization run at its
shipped `--popsize 32 --maxiter 1000` composes on the order of 32,000 models.

Uncached, the composed pipeline costs **13-15x more per environment** than the static model
files MyoAssist 0.1 shipped. Cached, it is back to parity:

| per environment, `myolegs22` | |
|---|--:|
| MyoAssist 0.1, static XML on disk | 0.037 s |
| composed, uncached | 0.691 s |
| composed, cached | 0.045 s |

Almost all of that remaining 0.045 s is the `MjModel.from_xml_string` the environment performs
either way -- the same work the old path did as `from_xml_path`. The cache read itself is about
0.2 ms.

From `myoassist`, the switch is one environment variable, which covers both the RL and the
controller-optimization paths because both compose through the same function:

```bash
export MYOASSIST_CACHE_DIR=~/.cache/myoassist
```

`compose_env_model(..., cache_dir=...)` takes it explicitly too, and overrides the variable.
That cache stores the *merged* model (human + device + terrain), and its key folds in the
source state of `assist_sim`, `myoassist_terrains` and the compose module itself, so editing
any of the three invalidates it.

The cache files are:

- `<cache_dir>/<sha1_key>.xml`: the combined XML
- `<cache_dir>/<sha1_key>.meta.json`: the input paths and their mtimes, to
  help you find which entry is which

**Invalidation rules:**
- A change to the mtime of any input file (device config, device XML) causes a miss.
- A change to either package's *source* causes a miss. The key folds in a token per
  package that is the release version **plus** the newest `*.py` mtime under it, for both
  `assist_sim` and `myo_sim`. Version alone was not enough: on an editable install it is
  fixed at install time, so a version-only key served models built by an older pipeline.
- A composed MSK model has no file on disk, so for `load_msk` the key is entirely
  `(msk_key, assist_sim token, myo_sim token)`.
- `planar_root=True` is a distinct key from `planar_root=False`.

Entries are written to a per-writer `<key>.<pid>.<rand>.partial` and published with an
atomic rename, so N processes racing a cold cache cannot leave a half-written file behind.
An entry that will not load is treated as a miss and rebuilt rather than raised.

**Eviction:** run `rm -r <cache_dir>` at any time. There is no background
cleanup and no size limit. The cache is a local optimization for one user.

## Usual workflows

### Iterate on a YAML file with fast feedback

```bash
# First run: full compile
python examples/quickstart.py myolegs26 MyDevice_L1
# Edit assist_sim/models/MyDevice/L1config.yaml
# Re-run: cache miss (YAML mtime changed); fresh compile
python examples/quickstart.py myolegs26 MyDevice_L1
```

### Generate a combined XML for a different tool

```bash
python -m assist_sim combine myolegs26 DephyExoBoot_L1 \
    -o /path/to/other_project/models/combined.xml
```

The other tool loads the XML directly with `MjModel.from_xml_path`.

## See also

- [usage.md](../usage.md): the full API
- [troubleshooting.md](../troubleshooting.md): problems with the mesh paths
  and with a stale cache
