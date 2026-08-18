# How To: Debug a Combined Model

The combined model can compile and still be incorrect. Examples are
geometry in the wrong position, a missing tendon, a joint that does not
actuate, or an incorrect view in the viewer. This guide tells you where to
look.

## Step 0: Read the error message

If the load gave an error, the error has a structure:

```
ValueError: tendon_modifications references unknown tendon 'gastroc_r_tendon'.
Did you mean: 'grac_r_tendon'?
```

The section name (`tendon_modifications`) tells you which YAML block to
fix. The `did you mean ...` suggestion compares your name with the real
names in the musculoskeletal (MSK) model and in the device. Usually one of
those names is the correct answer.

`tendon_modifications` runs before every removal. Its errors therefore come
before the errors of all other sections.

If the load is successful but the result is incorrect, continue.

## Step 1: Examine the compiled model in Python

```python
from assist_sim import load_combined
import mujoco as mj

model, data = load_combined("myolegs26", "MyDevice_L1")

# How big is the model?
print(f"nq={model.nq} nu={model.nu} nbody={model.nbody}")
print(f"nmesh={model.nmesh} ntendon={model.ntendon} nkey={model.nkey}")

# What bodies / actuators / tendons exist?
for i in range(model.nbody):
    print(mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i))

for i in range(model.nu):
    print(mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i))

for i in range(model.ntendon):
    print(mj.mj_id2name(model, mj.mjtObj.mjOBJ_TENDON, i))
```

A body, an actuator or a tendon that you expect can be absent. Then examine
the YAML section that must add it or keep it.

## Step 2: Export the combined XML

```python
from assist_sim import load_combined

load_combined("myolegs26", "MyDevice_L1", export_xml="combined.xml")
```

To get a baseline with no device, which is the start point of the removals:

```bash
python -m assist_sim msk myolegs26 -o baseline.xml
```

Then open `combined.xml` in an editor. The XML is the primary record of the
combination. Everything in `MjModel` comes from this file. The file also
contains the automatic additions of MuJoCo, such as the default textures.

Look for these items:

- **The body hierarchy.** Is the device body attached to the parent body
  that the YAML file names? The combined XML must contain, for example,
  `<body name="MyDevice_L1_my_part" ...>` inside the parent body.
- **The actuators.** The combined XML has every actuator from the device
  XML with the device prefix, plus everything in `actuators:` with no
  prefix.
- **The tendons.** The combined XML has the spatial tendons from the device
  XML with the prefix, plus the remaining tendons of the MSK model.
- **The keyframes.** `<key name="stand" qpos="..."/>` must have the authored
  MSK values plus any `keyframe_overrides`.

## Step 3: Examine the MSK spec inside the pipeline

`assist_sim` never writes the MSK model to disk. The MSK model is a live
`MjSpec` from `myo_sim`. `assist_sim` edits it in place, so there is no XML
file after the removals. Run the stages yourself when you need the state
between them:

```python
from assist_sim.combine import ModelCombiner
from assist_sim.config import DeviceConfig
from assist_sim.registry import _resolve_msk

spec = _resolve_msk("myolegs26")
config = DeviceConfig.from_yaml("assist_sim/models/MyDevice/L1config.yaml")

ModelCombiner._apply_tendon_modifications(spec, config, msk_key="myolegs26")
print(spec.tendon("gastroc_r_tendon"))    # present: re-anchored

ModelCombiner._apply_removals(spec, config, msk_key="myolegs26")
print(spec.tendon("gastroc_r_tendon"))    # None: taken by the cascade

model = spec.compile()          # the human, after surgery, before attachment
```

A muscle can be present after the first call and `None` after the second
call. The `spec.delete` cascade then removed the muscle, because one of its
wrap points was still on a removed body.

`assist_sim` does write the device XML to disk, two times: a full copy and
a copy with no meshes. To keep both copies, set `keep_temp=True`, which
`load_combined_model` and `ModelCombiner.combine` accept:

```python
load_combined_model(..., keep_temp=True)
```

The two files go to the system temp directory as
`<device_stem>__dev_full_*.xml` and `<device_stem>__dev_nomesh_*.xml`. They do
not go next to the device XML, because the package directory is read-only on a
container or a shared cluster node. Print the directory to find them:

```bash
python -c "import tempfile; print(tempfile.gettempdir())"
```

Use the two files when a device tendon, actuator or mesh is absent from the
combined model. Each copy carries absolute mesh paths, so it also loads on its
own in `simulate`.

## Step 4: Examine the model in the viewer

```bash
python examples/quickstart.py myolegs26 MyDevice_L1
```

The viewer starts in the paused state, so that you can turn the model.
Drag to rotate. Scroll to zoom. Use ctrl-drag to pan.

Look for these symptoms:

- **The device body is in the wrong position.** The `pos` or the `quat` is
  incorrect. Look at the device body in the device XML and at the
  attachment in the YAML file. Adjust the `pos` and the `quat` of the YAML
  attachment.
- **The mesh points sideways.** The frame of the device body comes from the
  frame of its parent. The parent can be the 80-muscle torso, which is
  rotated in relation to 22 and 26. In that case, use a per-MSK attachment
  with a `quat` that compensates.
- **A muscle is absent from the model.** This is the main symptom of a
  missed re-anchor. Its wrap points were on a removed body, so the
  `spec.delete` cascade also removed the tendon and its actuator. Re-anchor
  the muscle with `tendon_modifications`. If the muscle has no joint left to
  span, record the loss in `actuator_removals` and in `tendon_removals`.
- **A muscle is absent although every site moved.** A wrap cylinder stayed
  behind. A spatial tendon anchors on its wrap geoms and on its sites. One
  geom on a removed body therefore removes the whole tendon. Move the geom
  with `replace_geom`. Move its sidesite with `replace_site`. In the
  80-muscle leg the names are pairs, for example `SM_at_condyles_wrap_r`
  with `SM_at_condyles_site_semimem_r`.
- **The tendon is a straight line across the joint.** The muscle stayed in
  the model, but a moved wrap point is in the wrong position. Adjust the
  `pos` values in `tendon_modifications`.
- **The device floats free of the model.** The device body is not attached.
  Examine the `attachments` list in the YAML file. Confirm that the
  `device_body` name agrees with a top-level body in the device XML.

## Step 5: Check the frozen signatures

`tests/test_smoke_combinations.py` holds a frozen `(nq, nu, nbody, nmesh)`
tuple for each supported pair of MSK model and device. These tuples are the
reference values. Any change to the pipeline or to a config that moves one
of the four numbers stops this test first.

```bash
pytest tests/test_smoke_combinations.py -v -k "MyDevice"
```

A test failure prints the expected tuple and the actual tuple. Decide which
tuple is correct. Then correct the config, or update the frozen tuple in
the same commit as the change that moved it.

For the amputee path, `tests/test_tendon_reanchor.py` is the more specific
test. It pins the four wrap-edit ops, the muscle survival for each
amputation, the `ctrl` order, and the re-derived `lengthrange`:

```bash
pytest tests/test_tendon_reanchor.py -v
```

For a more detailed comparison, use `assist_sim.config.DeviceConfig` and
the live model in a script. Both give sufficient introspection for a
comparison element by element.

## Step 6: Use the validator

The validator makes a check before you compile the full model. The
validator reads XML, and a registry MSK model has no XML on disk. Export
one first:

```bash
python -m assist_sim msk myolegs26 -o myolegs26.xml
```

```python
from assist_sim.config import DeviceConfig
from assist_sim.validate import validate_config

issues = validate_config(
    human_xml="myolegs26.xml",
    config=DeviceConfig.from_yaml("assist_sim/models/MyDevice/L1config.yaml"),
)
for issue in issues:
    print(issue)
```

`validate_config` returns a list of unresolved references. These are names
in the YAML file that exist neither in the MSK model nor in the device XML.
An empty list shows no problems.

The validator has two limits. First, it checks the `default` block of each
per-MSK section, not the per-MSK blocks. To check an override block, assign
the resolved list first
(`config.tendon_modifications = config.resolve_tendon_modifications("myolegs")`).
Second, the validator makes a name check only. It never compiles, so it
cannot tell you that a re-anchored wrap point is in the wrong position.

## Quick reference: symptoms and causes

| Symptom | Likely cause |
|---|---|
| `ValueError: unknown body 'X' in body_removals` | A typo in the YAML file, or a per-MSK config used on the wrong MSK model |
| `ValueError: unknown tendon 'X' in tendon_modifications` | The same. Examine the per-MSK overrides. The section runs first, so this error comes before any removal |
| `ValueError: wrap edit op 'drop_site' is no longer supported` | `drop_site` is retired. Re-anchor with `replace_site` or `replace_geom`, or remove the muscle with `actuator_removals` |
| `ValueError: replace_site 'X' requires 'new_body'` | Both `replace_*` ops need `new_body`. All four ops need `pos` |
| A muscle is absent from the combined model | Its wrap points were on a removed body, and it is not re-anchored. Add `tendon_modifications` |
| A muscle is absent although every site moved | A wrap cylinder stayed on a removed body. Add `replace_geom` for it, plus `replace_site` for its sidesite |
| A muscle gives an implausible force after the re-anchor | Its authored `lengthrange` still describes the intact path. Add an `actuator_overrides` entry |
| The device body floats in position | The device body is not in `attachments`, or its name does not agree with the XML |
| The mesh orientation is wrong on one MSK model only | The parent body frame differs between MSK models. Use per-MSK `attachments` with `quat` |
| `nq` decreases more than expected after `body_removals` | The cascade removed wrap-site bodies, which have joints. This is correct. The pipeline also removes their qpos slots from the keyframes |
| The keyframe pose is all zeros except the pelvis | The keyframe pruning and restore code is incorrect (this was a real bug before). Make sure that the joint table covers ALL named joints |
| The model has no ground | This is correct. `assist_sim` removes the bundled myosuite scene at resolve time, so an export carries no floor. Put a scene (`myoassist.terrains`) on top |
