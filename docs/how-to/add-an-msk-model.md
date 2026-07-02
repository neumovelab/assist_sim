# How To: Add a New MSK Model

MSK models live in `myo_sim`, not in this repo. Adding a new MSK is a
two-step process: contribute the MSK to `myo_sim`, then register it
here.

## Step 1 -- Contribute the MSK to `myo_sim`

Upstream the new MSK to `MyoHub/myo_sim` via PR. On the `mm_refactor` branch,
leg models are *composed at runtime* and exposed as editable specs, so a new
MSK means:

- Register a composed model (a `BuildStrategy` + `MODEL_REGISTRY` entry in
  `myo_sim/build/compose.py`), and
- Expose it via `myo_sim.build_spec(<name>)` (i.e. add it to
  `GENERATE_SPEC_BUILDERS` / `FRAGMENT_SPEC_BUILDERS`) so `assist_sim` can get
  an editable `MjSpec` for it. See `myolegs26` for a worked example.

For the *interim* development period where `myo_sim` isn't yet PyPI-published
with your MSK, install a fork or branch:

```bash
pip install git+https://github.com/<your-fork>/myo_sim.git@<branch>
```

`assist_sim` consumes whatever's installed -- it doesn't care whether the
source is upstream or a fork.

## Step 2 -- Register the MSK in `assist_sim`

Add an entry to `_COMPATIBLE_MSK_KEYS` in `assist_sim/registry.py`, binding the
key to the `myo_sim.build_spec` model name and the minimum MuJoCo version that
can build it:

```python
_COMPATIBLE_MSK_KEYS: Dict[str, _MskSource] = {
    ...
    "MyNewMSK": _MskSource("my_new_model", (3, 3, 3), ""),    # NEW
}
```

`_MskSource(myo_sim_model, min_mujoco, note)`: `myo_sim_model` is the
`build_spec` name (or `None` for a planned key with no source yet); `min_mujoco`
gates models that need newer MuJoCo (e.g. passive-torso conversions need
`(3, 3, 4)`); `note` explains a gated/planned state in the error the caller
sees. At resolve time `assist_sim` calls `build_spec`, serializes the returned
`MjSpec`, strips the bundled myosuite scene, and caches a model-only XML.

## Step 3 -- Verify resolution

```python
from assist_sim.registry import resolve

msk_path, _ = resolve("MyNewMSK", "DephyExoBoot_L1")
print(msk_path)        # a cached, model-only XML Path that exists
```

If you get an `ImportError`, either `myo_sim` isn't installed, the installed
MuJoCo is older than `min_mujoco`, or `build_spec` failed -- the message says
which. Confirm `myo_sim` knows the model:

```python
import myo_sim
print("my_new_model" in myo_sim._COMPOSED_MODELS)
```

## Step 4 -- Update devices for compatibility

If your new MSK has unique conventions (different body / tendon names,
different world orientation, different DOFs), devices that previously
worked on 22/26/80 may break on it. For each device YAML where this
matters, add a per-MSK override block. See
[modify-an-msk-config.md](modify-an-msk-config.md).

Two common patterns:

1. **MSK has different tendon names** → add `myolegs: []`-style
   opt-outs (or per-MSK alternate names) to `tendon_modifications`,
   `tendon_removals`, `actuator_removals`.
2. **MSK has a different parent body for an attachment** → add
   per-MSK `attachments` block (see HMEDI's `myolegs` handling for
   an example).

## Step 5 -- Add tests + docs

- Add the new MSK to the `EXPECTED` dict in
  `tests/test_smoke_combinations.py` for each device combination you
  expect to work. The `(nq, nu, nbody, nmesh)` tuples are frozen
  signatures -- get them by running a one-off probe and pasting in
  the actual values.
- Update [available-models.md](../available-models.md) with the new
  MSK and its compatibility row.
- If the MSK has notable structural differences worth documenting
  (different facing direction, no arms, freejoint root, etc.), add
  a paragraph to [available-models.md](../available-models.md#important-msk-differences).

## Step 6 -- Update quickstart's camera dispatch

The viewer's initial camera azimuth in `examples/quickstart.py` is
chosen per-MSK because each MSK faces a different direction in world
coords. If the new MSK has a unique orientation, add a branch:

```python
if args.msk == "MyNewMSK":
    viewer.cam.azimuth = ...        # tune from a sample <camera> pos
    viewer.cam.elevation = ...
elif args.msk == "myolegs":
    ...
else:
    ...
```

To derive the values from a `<camera pos=... xyaxes=.../>` element, use
the conversion in [usage.md](../usage.md) or just iterate by eye in the
viewer (the right-side panel shows the current camera state).

## See also

- [concepts.md](../concepts.md#naming-conventions) -- registry key conventions
- [available-models.md](../available-models.md) -- full MSK + device matrix
