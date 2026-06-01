# Troubleshooting

Common errors and how to fix them. If you hit something not in this list,
the error messages all carry a `did you mean ...` suggestion + the section
of the YAML the bad reference came from — read those first.

## Install / import

### `ImportError: ... myo_sim ... not installed`

```
ImportError: The MSK model 'myoLeg22_2D' lives in the myo_sim package,
which is not installed (looked for myo_sim.leg.myoLeg22_2D.xml).
```

`myo_sim` ships the MSK XML files; `assist_sim` resolves them via
`importlib.resources`. Install it:

```bash
pip install myo_sim   # once on PyPI
# or, interim:
pip install git+https://github.com/MyoHub/myo_sim.git@<tag>
```

### `FileNotFoundError: MSK model file missing inside myo_sim`

`myo_sim` is installed but doesn't include the specific MSK key you
requested. Likely the upstream version hasn't merged 22/26 yet. Try a
newer tag, or fall back to `myoLeg80` (which is upstream-ready).

### `ModuleNotFoundError: No module named 'assist_sim'`

`assist_sim` isn't on `sys.path`. Either:
- `pip install -e .` from the repo root
- Or run scripts that do `sys.path.insert(0, repo_root)` (the examples
  in `examples/` do this)

## Config / resolution

### `ValueError: Unknown MSK model 'myoleg22'`

Typo in the MSK key. The error includes a `Did you mean 'myoLeg22_2D'?`
suggestion — use it. Keys are case-sensitive and follow the exact form in
`_COMPATIBLE_MSK_KEYS`.

### `ValueError: Unknown device 'OSL'`

Typo in the device key. Try `OpenSourceLeg_A_L1` or the alias `OSL_A`.
`python -m assist_sim list` shows everything.

### `ValueError: Device 'X' is not compatible with MSK 'Y'`

The device's YAML declares a `compatible_msk:` list and `Y` isn't in it.
Either:
- Pick a compatible MSK (the error lists them)
- Or, if you intended `Y` to be compatible, remove the `compatible_msk:`
  field from the device YAML (or add `Y` to the list)

### `ValueError: tendon_modifications references unknown tendon 'gastroc_r_tendon'`

The YAML refers to a tendon that doesn't exist in the MSK you're combining
with. Common cause: a `default:` block authored for 22/26 was applied to
80 (which uses different tendon names like `gasmed_r_tendon` /
`gaslat_r_tendon`). Fix: add an empty `myoLeg80: []` per-MSK override to
opt out:

```yaml
tendon_modifications:
  default: [...]
  myoLeg80: []
```

Or author 80-specific entries that use the 80 names.

### Cascade-cleanup errors after `body_removals`

If a YAML's `actuator_removals` or `tendon_removals` mentions an actuator
the body removal *already* removed (cascade), you get an "unknown actuator"
error. The order of operations is: body removals → actuator removals →
tendon removals. So if the cascade already nuked it, drop the explicit
removal from the YAML.

## Rendering / viewer

### Pure white background in the viewer

The terrain strip removes the ground but keeps the unnamed skybox texture
+ a 2D texture + a material (intentionally — MuJoCo's renderer needs that
texture/material binding for the skybox to render at all). If you still
get pure white, check:

- Your terrain include is detected (filename starts with `terrain_config`)
- The terrain XML defines both a skybox texture *and* a 2D texture with a
  material — both must survive the strip for the skybox to render

### Model floats inside / outside the floor in the viewer

`assist_sim` exports are **model-only**: no ground body. If you want a
floor for simulation, layer one via `myoassist.terrains` or include a
terrain config explicitly in your wrapping XML.

### Initial camera view is wrong direction

`examples/quickstart.py` dispatches camera azimuth on `args.msk`. If you
add a new MSK with a different world orientation, tweak the `azimuth`
branch there.

### Shadows / reflections still visible

`quickstart.py` sets `model.vis.quality.shadowsize = 0` to disable shadows.
Reflections are off because no remaining material has `reflectance > 0`
after the terrain strip. If you want them back: toggle in the viewer's
right-side **Rendering** panel, or comment out the shadowsize line.

## Cache

### Stale combined output after editing a config

The cache keys on input mtimes. If you edited a config file but the
filesystem mtime didn't update (e.g. some editors restore mtime on save),
force a recompile:

```bash
rm -r .assist_sim_cache/
```

Or bump `assist_sim.__version__` in `assist_sim/__init__.py` — that
invalidates all entries globally.

## Tests

### MSK-dependent tests are all skipped

Expected when `myo_sim` isn't installed. The `needs_myo_sim` marker in
`tests/conftest.py` gates them. Install `myo_sim` and rerun.

### Test failure: signature mismatch in `test_smoke_combinations.py`

The expected `(nq, nu, nbody, nmesh)` tuples are frozen. If you changed
pipeline behavior in a way that affects compiled output, update the
EXPECTED dict in the test file to match the new numbers — and bump
`assist_sim.__version__` so caches invalidate.

## Pipeline internals

### "ValueError: Body 'X' not found in human model"

The body name you listed in `body_removals` doesn't exist in the MSK.
Common cause: same config used on multiple MSKs with different body
naming. Use per-MSK `body_removals` (planned schema; currently use a
single list that's a union or split into two configs).

### Mesh paths broken in exported XML

`assist_sim` rewrites mesh paths relative to the export location and
strips the source `meshdir`. If your downstream tool can't find a mesh,
check whether it's reading the export as-is or applying its own
`meshdir`. The export's mesh paths are *only* valid relative to the
file's own directory.

## Asking for help

If you're stuck, gather:

- The exact `load_combined_model` / `load_combined` call you made
- The full error including the "did you mean" suggestion
- The YAML you're using (or a diff against what's in the repo)
- `python -m assist_sim --version` output

Then open an issue at <https://github.com/NeuMove/assist_sim/issues>.
