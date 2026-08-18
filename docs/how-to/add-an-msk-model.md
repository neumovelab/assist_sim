# How To: Add a New MSK Model

The musculoskeletal (MSK) models are in `myo_sim`, not in this repository.
To add a new MSK model, do two steps. First, contribute the MSK model to
`myo_sim`. Then register the MSK model here.

## Step 1: Contribute the MSK model to `myo_sim`

Send the new MSK model to `MyoHub/myo_sim` in a pull request. On the `dev`
branch, `myo_sim` composes the leg models at run time and gives them as
editable specs. A new MSK model therefore needs two items:

- A composed model. Add a `BuildStrategy` and a `MODEL_REGISTRY` entry in
  `myo_sim/build/compose.py`.
- Access through `myo_sim.load_spec(<name>)`. Add the model to
  `GENERATE_SPEC_BUILDERS` or `FRAGMENT_SPEC_BUILDERS`, so that `assist_sim`
  can get an editable `MjSpec` for it. `myolegs26` is a worked example.

The published `myo_sim` package will not contain your new MSK model until it is merged
upstream, so install your fork or branch over it:

```bash
pip install git+https://github.com/<your-fork>/myo_sim.git@<branch>
```

`assist_sim` uses the installed `myo_sim`. The source can be the upstream
package or a fork.

## Step 2: Register the MSK model in `assist_sim`

Add an entry to `_COMPATIBLE_MSK_KEYS` in `assist_sim/registry.py`. The
entry binds the key to the `myo_sim.load_spec` model name and to the
minimum MuJoCo version that can build the model:

```python
_COMPATIBLE_MSK_KEYS: Dict[str, _MskSource] = {
    ...
    "MyNewMSK": _MskSource("my_new_model", (3, 3, 3), ""),    # NEW
}
```

The fields of `_MskSource(myo_sim_model, min_mujoco, note)` are:

- `myo_sim_model` is the `load_spec` name. Use `None` for a planned key
  that has no source yet.
- `min_mujoco` blocks a model that needs a newer MuJoCo. For example, the
  passive-torso conversions need `(3, 3, 4)`.
- `note` explains a blocked state or a planned state in the error that the
  caller sees.

`_MskSource` has an optional fourth field, `reduce_to_22`. This field marks
a key that comes from another key through the 26->22 planar reduction. Only
`myolegs22` uses it today.

At resolve time, `assist_sim` calls `load_spec`. It then removes the
bundled myosuite scene from the returned `MjSpec`. It gives that live spec
to the pipeline. The pipeline does not serialize the spec and does not
cache it at this point.

Torso-composed models do not round-trip through `to_xml`. Every edit
therefore occurs on the spec in memory. These edits are the removals
through `spec.delete`, the attachment and the overrides. `assist_sim`
builds a fresh spec on each call, because the pipeline changes the spec in
place.

## Step 3: Verify resolution

`registry.resolve` returns `(MjSpec, Path)`. The first element is a live
spec, not a path:

```python
from assist_sim.registry import resolve

human_spec, device_config_path = resolve("MyNewMSK", "DephyExoBoot_L1")
print(len(human_spec.bodies))       # a composed MjSpec, editable
print(human_spec.compile().nq)      # compile it to see the model
print(device_config_path)           # the device config.yaml Path
```

To get a compiled baseline with no device, use `load_msk`:

```python
from assist_sim import load_msk

model, data = load_msk("MyNewMSK")
print(model.nq, model.nu, model.nbody)
```

Or use the command line, which also writes the XML:

```bash
python -m assist_sim msk MyNewMSK -o mynewmsk.xml
```

An `ImportError` has three possible causes. `myo_sim` is not installed, the
installed MuJoCo is older than `min_mujoco`, or `load_spec` gave an error.
The message tells you which cause is correct. To confirm that `myo_sim`
knows the model:

```python
import myo_sim
print("my_new_model" in myo_sim._COMPOSED_MODELS)
```

## Step 4: Update the devices for compatibility

Your new MSK model can have unique conventions. Examples are different body
or tendon names, a different world orientation, or different degrees of
freedom (DOF). Devices that work on 22, 26 or 80 can then give an error on
the new model. For each device YAML file that this affects, add a per-MSK
override block. See [modify-an-msk-config.md](modify-an-msk-config.md).

Three usual patterns:

1. **The MSK model has different tendon names.** Add `myolegs: []` opt-outs,
   or per-MSK alternate names, to `tendon_modifications`, `tendon_removals`
   and `actuator_removals`.
2. **The MSK model has a different parent body for an attachment.** Add a
   per-MSK `attachments` block. The `myolegs` handling in HMEDI is an
   example.
3. **The MSK model is a target for an amputee device.** Add a per-MSK
   `tendon_modifications` block that names the wrap sites and the wrap
   geoms. Also add a matching `actuator_overrides` block with the
   re-derived `lengthrange` for each muscle that you re-anchor. Both blocks
   are per-MSK, because the muscle names and the geometry differ by lineage.

## Step 5: Add tests and docs

- Add the new MSK model to the `EXPECTED` dict in
  `tests/test_smoke_combinations.py`, for each device combination that you
  expect to work. The `(nq, nu, nbody, nmesh)` tuples are frozen signatures.
  Get them from a single probe run. Then put the actual values in the dict.
- Update [available-models.md](../available-models.md) with the new MSK
  model and its compatibility row.
- The MSK model can have structural differences that other users must know
  about. Examples are a different facing direction, no arms, or a freejoint
  root. Add a paragraph about them to
  [available-models.md](../available-models.md#important-msk-differences).

## Step 6: Update the camera dispatch in quickstart

Each MSK model faces a different direction in world coordinates. The
`examples/quickstart.py` script therefore selects the initial camera azimuth
of the viewer per MSK model. If the new MSK model has a unique orientation,
add a branch:

```python
if args.msk == "MyNewMSK":
    viewer.cam.azimuth = ...        # tune from a sample <camera> pos
    viewer.cam.elevation = ...
elif args.msk == "myolegs":
    ...
else:
    ...
```

To derive the values from a `<camera pos=... xyaxes=.../>` element, use the
conversion in [usage.md](../usage.md). As an alternative, adjust the values
in the viewer until the view is correct. The panel on the right shows the
current camera state.

## See also

- [concepts.md](../concepts.md#naming-conventions): the registry key
  conventions
- [available-models.md](../available-models.md): the full matrix of MSK
  models and devices
