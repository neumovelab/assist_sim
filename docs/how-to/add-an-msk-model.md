# How To: Add a New MSK Model

MSK models live in `myo_sim`, not in this repo. Adding a new MSK is a
two-step process: contribute the MSK to `myo_sim`, then register it
here.

## Step 1 -- Contribute the MSK to `myo_sim`

Upstream the new MSK XML to `MyoHub/myo_sim` via PR. Conventions in
`myo_sim` (per upstream):

- Lower-limb MSKs live under `myo_sim/leg/`
- File naming follows existing patterns (e.g. `myolegs.xml` for the
  primary 80-muscle, named variants for customized models)
- Mesh files referenced via relative paths inside the package

For the *interim* development period where `myo_sim` isn't yet
PyPI-published with your MSK, you can use a git+http install of a
fork or branch:

```bash
pip install git+https://github.com/<your-fork>/myo_sim.git@<branch>
```

`assist_sim` consumes whatever's installed -- it doesn't care whether
the source is upstream or a fork.

## Step 2 -- Register the MSK in `assist_sim`

Add an entry to `_COMPATIBLE_MSK_KEYS` in `assist_sim/registry.py`:

```python
_COMPATIBLE_MSK_KEYS: Dict[str, Tuple[str, str]] = {
    "myoLeg22_2D": ("myo_sim.leg", "myoLeg22_2D.xml"),
    "myoLeg26_3D": ("myo_sim.leg", "myoLeg26_3D.xml"),
    "myoLeg80":    ("myo_sim.leg", "myolegs.xml"),
    "MyNewMSK":    ("myo_sim.leg", "my_new_msk.xml"),    # NEW
}
```

Each entry maps a registry key to a `(subpackage, filename)` tuple.
`importlib.resources.files(subpackage).joinpath(filename)` resolves
the file at runtime.

## Step 3 -- Verify resolution

```python
from assist_sim.registry import resolve

msk_path, _ = resolve("MyNewMSK", "DephyExoBoot_L1")
print(msk_path)        # should print an absolute Path that exists
```

If you get `FileNotFoundError`, the file isn't where the registry
expects. Check the installed `myo_sim`'s layout:

```python
import importlib.resources
import myo_sim
print(list(importlib.resources.files("myo_sim.leg").iterdir()))
```

## Step 4 -- Update devices for compatibility

If your new MSK has unique conventions (different body / tendon names,
different world orientation, different DOFs), devices that previously
worked on 22/26/80 may break on it. For each device YAML where this
matters, add a per-MSK override block. See
[modify-an-msk-config.md](modify-an-msk-config.md).

Two common patterns:

1. **MSK has different tendon names** → add `myoLeg80: []`-style
   opt-outs (or per-MSK alternate names) to `tendon_modifications`,
   `tendon_removals`, `actuator_removals`.
2. **MSK has a different parent body for an attachment** → add
   per-MSK `attachments` block (see HMEDI's `myoLeg80` handling for
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
elif args.msk == "myoLeg80":
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
