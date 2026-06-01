# How To: Debug a Combined Model

When the combined model compiles but something's wrong — geometry in the
wrong place, missing tendon, joint not actuating, viewer looks weird —
here's where to look.

## Step 0: read the error message

If `load_combined_model` raised, the error has structure:

```
ValueError: tendon_modifications references unknown tendon 'gastroc_r_tendon'.
Did you mean: 'grac_r_tendon'?
```

The section name (`tendon_modifications`) tells you which YAML block to
fix. The `did you mean ...` suggestion uses fuzzy matching against the
real names in the MSK + device — usually the right answer is one of those.

If `load_combined_model` succeeded but the result is wrong, continue.

## Step 1: inspect the compiled model in Python

```python
from assist_sim import load_combined
import mujoco as mj

model, data = load_combined("myoLeg22_2D", "MyDevice_L1")

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

If a body / actuator / tendon you expected is missing, check the YAML
section that should have added or preserved it.

## Step 2: export the combined XML and read it

```python
from assist_sim import load_combined_model
load_combined_model(
    human_xml="...",
    device_config="...",
    export_xml="combined.xml",
)
```

Then open `combined.xml` in an editor. The XML is the canonical view of
what got combined; everything you see in `MjModel` came from this file
(modulo MuJoCo's own auto-additions like default textures).

What to look for:

- **Body hierarchy** — is your device body attached where the YAML said?
  The combined XML should have e.g. `<body name="MyDevice_L1_my_part"
  ...>` nested under the parent body.
- **Actuators** — every actuator from the device XML (with the device
  prefix) plus everything in `actuators:` (without prefix).
- **Tendons** — spatial tendons from the device XML (with prefix) plus
  the MSK's surviving tendons.
- **Keyframes** — `<key name="stand" qpos="..."/>` should have the
  authored MSK values plus any `keyframe_overrides`.

## Step 3: keep the preprocess temp file

The pipeline writes a temp XML during preprocess (before the MjSpec
phase). Keep it for inspection:

```python
load_combined_model(..., keep_temp=True)
```

Look for files matching `<msk_name>__human_pp_*.xml` next to the source
MSK. This is the human XML *after* removals and cascades but *before*
device attachment. Useful when:

- The compiled `nbody` is wrong → check whether the body removal cascade
  removed too much or too little
- Tendons disappeared → check whether their wrap sites got auto-pruned
  unexpectedly

## Step 4: visually inspect in the viewer

```bash
python examples/quickstart.py myoLeg22_2D MyDevice_L1
```

The paused viewer lets you spin the model. Drag to rotate, scroll to
zoom, ctrl-drag to pan.

What to look for:

- **Device body in the wrong place** → the `pos`/`quat` on either the
  device body (in the device XML) or the YAML attachment is off.
  Iterate by editing the YAML attachment's `pos`/`quat`.
- **Mesh oriented sideways** → device body's frame inherits its parent's
  frame. If the parent is e.g. the 80-muscle torso (rotated relative to
  22/26), use per-MSK attachment with a compensating `quat`.
- **Tendon dangling in space** → a wrap site on a removed body that
  wasn't re-anchored. Add a `tendon_modifications` with
  `replace_site` / `reposition_site` in the YAML.
- **Device floats free of the model** → the device body wasn't actually
  attached. Check the YAML's `attachments` list and confirm the
  `device_body` name matches a top-level body in the device XML.

## Step 5: diff against a known-good output

If you have a legacy combined XML for the same MSK + device, diff the
exported XML against it. `tests/test_legacy_parity.py` does this
structurally (matching `nq`, `nu`, body tree, actuator triples, tendon
wraps); you can run it directly:

```bash
pytest tests/test_legacy_parity.py -v -k "MyDevice"
```

For finer-grained diffing, both `assist_sim.config.DeviceConfig` and
the live model expose enough introspection to compare element-by-element
in a script.

## Step 6: use the validator

For a pre-flight check that doesn't fully compile the model:

```python
from assist_sim.validate import validate_config

issues = validate_config(
    human_xml="path/to/myo_sim/leg/myoLeg22_2D.xml",
    config=DeviceConfig.from_yaml("models/MyDevice/L1config.yaml"),
)
for issue in issues:
    print(issue)
```

Returns a list of unresolved references (names in the YAML that don't
exist in either the MSK or the device XML). Empty list = clean.

## Common cause cheatsheet

| Symptom | Likely cause |
|---|---|
| `ValueError: unknown body 'X' in body_removals` | Typo in YAML, or per-MSK config used on the wrong MSK |
| `ValueError: unknown tendon 'X' in tendon_modifications` | Same; check per-MSK overrides |
| Tendon wrap site count smaller than expected | Auto-prune removed wraps on removed-body sites. Add `tendon_modifications` to re-anchor if you wanted them preserved |
| Device body floats in place | Forgot to list it in `attachments`, or device body's name doesn't match the XML |
| Mesh in wrong orientation on one MSK only | Parent body frame differs across MSKs. Use per-MSK `attachments` with `quat` |
| nq drops more than expected after body_removals | Cascade removed wrap-site bodies (they have joints). Expected; their qpos slots are pruned from keyframes |
| Keyframe pose looks all-zeros except pelvis | The keyframe pruning math + restore is broken (was a real bug previously); make sure the joint table covers ALL named joints |
| All-white background in viewer | The terrain strip dropped both 2D texture + material binding. Fixed in current pipeline; if it recurs, check `_strip_terrain` in `utils.py` |
