# Troubleshooting

This page lists common errors and their fixes. If you find an error that is not
in this list, read the error message first. Every error message carries a
`did you mean ...` suggestion and the name of the YAML section that holds the
incorrect reference.

## Install / import

### `ImportError: ... myo_sim ... is not installed`

```
ImportError: The MSK model 'myolegs26' is composed by the myo_sim package,
which is not installed. Install it with `pip install myo_sim` ...
```

`myo_sim` composes the musculoskeletal (MSK) models. `assist_sim` gets them
through `myo_sim.build_spec(...)`. Install `myo_sim`:

```bash
pip install myo_sim   # once on PyPI
# or, interim:
pip install git+https://github.com/MyoHub/myo_sim.git@dev
```

### `ImportError: MSK model '...' requires ... mujoco>=3.3.4`

The pipeline does the model surgery in memory with `MjSpec.delete`. MuJoCo
3.3.4 is the first version that has `MjSpec.delete`. Your environment has an
older MuJoCo version. `assist_sim` pins `mujoco>=3.3.4`, but a shared
environment can pin an older version. To upgrade, run
`pip install "mujoco>=3.3.4"`.

### `ValueError: MSK model '...' is not available yet`

`assist_sim` raises this error when a registry key has no `myo_sim` source
model. All four keys (`myolegs22`, `myolegs26`, `myolegs`, `myofullbody`) have
a source model today, so the supplied registry does not raise this error.
`assist_sim` builds `myolegs22` when it reduces `myolegs26` in memory from 26
to 22 muscles. `myolegs22` is available today; it is not a planned model.

### `ModuleNotFoundError: No module named 'assist_sim'`

`assist_sim` is not on `sys.path`. Use one of these two fixes:
- Run `pip install -e .` from the root of the repository.
- Or run scripts that call `sys.path.insert(0, repo_root)`. The scripts in
  `examples/` do this.

## Config / resolution

### `ValueError: Unknown MSK model 'myoleg22'`

The MSK key has a spelling error. The error message includes a
`Did you mean 'myolegs22'?` suggestion. Use that suggestion. The keys are
case-sensitive, and they have the exact form in `_COMPATIBLE_MSK_KEYS`.

### `ValueError: Unknown device 'OSL'`

The device key has a spelling error. Try `OpenSourceLeg_A_L1` or the alias
`OSL_A`. The command `python -m assist_sim list` shows every key.

### `ValueError: Device 'X' is not compatible with MSK 'Y'`

The YAML of the device sets a `compatible_msk:` list, and `Y` is not in that
list. Use one of these two fixes:
- Select a compatible MSK model. The error message lists them.
- Or, if `Y` must be compatible, add `Y` to the list. You can also remove the
  `compatible_msk:` field from the device YAML.

### `ValueError: tendon_modifications references unknown tendon 'gastroc_r_tendon2'`

```
ValueError: tendon_modifications references unknown tendon 'gastroc_r_tendon2'.
Did you mean: 'gastroc_r_tendon', 'gastroc_l_tendon', 'vasti_r_tendon'?
```

The YAML names a tendon that the MSK model does not have. A common cause is a
`default:` block that you wrote for the 22-muscle or 26-muscle model and then
applied to the 80-muscle lineage. The 80-muscle lineage splits the lumped
muscles and uses different names, such as `gasmed_r_tendon` and
`gaslat_r_tendon` for `gastroc_r_tendon`. To fix this, write 80-muscle entries
under `myolegs:`. You can also add an empty `myolegs: []` override to disable
the block:

```yaml
tendon_modifications:
  default: [...]
  myolegs: []
```

`myofullbody` uses the same 80-muscle leg, so it usually takes the same block.
A YAML anchor keeps the two blocks equal.

### `ValueError: tendon_modifications references unknown site '...'`

Each wrap edit also resolves its **target**, and `replace_*` resolves its
`new_body`. All three raise the error in the same way:

```
ValueError: tendon_modifications references unknown site 'gastroc_r_med_gas_r-P9'.
Did you mean: 'gastroc_r_med_gas_r-P3', ...?
ValueError: tendon_modifications references unknown geom 'nope_wrap_r'. ...
ValueError: tendon_modifications.new_body references unknown body 'tibia_rr'.
Did you mean: 'tibia_r', 'tibia_l'?
```

The wrap site names and the geom names differ between the lineages, in the same
way as the tendon names. Therefore the fix is the same: use per-MSK entries, or
an empty override.

### `ValueError: wrap edit op 'drop_site' is no longer supported`

```
ValueError: wrap edit op 'drop_site' is no longer supported: dropping a wrap
needs an editable wrap list, which MjSpec does not expose. Re-anchor the wrap
with 'replace_site' / 'replace_geom', or remove the whole muscle with
'actuator_removals'
```

`drop_site` is retired. It raises an error instead of a skip. Therefore a
surgical edit that you believe is applied always has an effect, or it tells you
that it failed. The four active operations are `reposition_site`,
`replace_site`, `reposition_geom` and `replace_geom`.

### A re-anchored muscle starts outside its own length range

`tendon_modifications` changes the path of a muscle, but the compiler keeps the
authored `lengthrange` (`LRopt.useexisting=1`). The `lengthrange` of a
re-anchored muscle then describes the old path. Calculate the range again. Then
set it in the `actuator_overrides:` section:

```yaml
actuator_overrides:
  default:
    - name: "gastroc_r"
      lengthrange: [0.202513, 0.229413]
```

The other muscles keep their authored values. The name resolves bare first,
then with the device prefix. If neither name exists, the code raises an error
with a "did you mean" list.

### A muscle disappears that you expected surgery to keep

`spec.delete(body)` cascades. It removes the subtree and every element that
references the subtree. This includes each tendon with a wrap point on a
removed body. The surgery keeps some biarticular muscles, such as `hamstrings`
across a transfemoral amputation and `gastroc` across a transtibial amputation.
The cascade also removes these muscles if you do not re-anchor them first.

To fix this, add a `tendon_modifications:` block. The block must move every
wrap point at the cut or distal to the cut onto the residual bone. This block
runs **before** the removals. Therefore the tendon does not reference a removed
body when the cascade runs. Also move the `sidesite` of each wrap cylinder. If
one geom stays behind, the cascade removes the tendon, whatever number of sites
you moved.

### Order of operations inside a removal pass

`body_removals` → `geom_removals` → contact `<pair>` removal →
`actuator_removals` → `tendon_removals` → `sensor_removals`.

`body_removals` and `geom_removals` name the primary surgery objects, and they
**raise** an error for an unknown name. The other three sections remove an
element only if it is present, because the body cascade can remove it first.
Therefore an entry for an actuator that the cascade already removed causes no
error. Keep such an entry only as a record of your intent.

## Rendering / viewer

### An export carries a skybox you did not ask for

The exports are **not** scene-free. `_strip_scene_visual` removes the myosuite
`<headlight>` and `<global>` elements from the MSK model. Then
`_ensure_minimal_visual` **adds** a soft headlight, a `<scale>` block and a
neutral gradient skybox. Therefore a bare export renders correctly in a viewer.
This behavior applies to every export: combined models and `load_msk` output.

You can override both additions. A downstream scene that supplies its own
`<headlight>` or skybox has priority when you put it on top. If you want a
specific backdrop, replace the skybox in your wrapper XML.

### Model floats inside / outside the floor in the viewer

The `assist_sim` exports are **model-only** in the sense that matters here:
they have no ground body, no hfield and no floor. If you want a floor for the
simulation, add one with `myoassist.terrains`. You can also include a terrain
config in your wrapper XML.

The live pipeline has no step that removes terrain. The composed MSK models
have no terrain include. `registry._resolve_msk` removes the bundled myosuite
scene (floor plane, backdrop, pedestal, logo, scene lights and cameras) at
resolve time, before the model reaches `combine`. `utils._strip_terrain` still
exists, but no caller passes `terrain_paths`. Therefore it runs only in its own
test.

### The initial camera view is off center or incorrectly rotated

`examples/quickstart.py` selects the camera azimuth from `args.msk`. If you add
a new MSK model with a different world orientation, change the `azimuth` branch
in that file.

## Cache

### Stale combined output after a config edit

The cache key uses the mtime of each input file. Some editors restore the mtime
when they save a file. If you edited a config file and the mtime did not
change, force a new compile:

```bash
rm -r .assist_sim_cache/
```

## Tests

### All MSK-dependent tests skip

This is correct when `myo_sim` is not installed. The `needs_myo_sim` marker in
`tests/conftest.py` controls these tests. Install `myo_sim`. Then run the tests
again.

### Test failure: signature mismatch in `test_smoke_combinations.py`

The expected `(nq, nu, nbody, nmesh)` tuples are frozen. If your change to the
pipeline affects the compiled output, update the EXPECTED dict in the test file
with the new numbers. Then increase `assist_sim.__version__` to invalidate the
caches.

## Pipeline internals

### `ValueError: body_removals references unknown body 'X'`

The MSK model does not have the body name that you listed in `body_removals`.
A common cause is one config that you use on several MSK models with different
skeletons. `body_removals` accepts the per-MSK form, so select the bodies by
key:

```yaml
body_removals:
  default:
    - "tibia_r"
  myolegs: &r80_bodies
    - "tibia_r"
    - "patella_r"   # a sibling of tibia_r, so the cascade misses it
  myofullbody: *r80_bodies
```

Every entry that the cascade already removes is optional. Keep it only as a
record of your intent.

### Broken mesh paths in the exported XML

`assist_sim` rewrites the mesh paths relative to the export location, and it
removes the source `meshdir`. If your downstream tool does not find a mesh,
check the tool. It can read the export without a change, or it can apply its
own `meshdir`. The mesh paths in the export are valid *only* relative to the
directory of the export file.

## How to ask for help

If you cannot solve the problem, collect this information:

- The exact `load_combined_model` or `load_combined` call that you made
- The full error message, with the "did you mean" suggestion
- The YAML that you use, or a diff against the YAML in the repository
- The output of `python -m assist_sim --version`

Then open an issue at <https://github.com/neumovelab/assist_sim/issues>.
