# How To: Use Custom Devices

`assist_sim` ships with seven bundled devices (Dephy, HMEDI, Humotech,
OpenExo, Tutorial, and the two OSL variants). If you're authoring your
own device -- for a new exoskeleton, a different prosthetic geometry, an
internal lab project -- this guide covers where it lives and how to use it.

## Directory layout

Whether bundled or user-authored, every device follows the same shape:

```
<DeviceName>/
├── L1config.yaml         # the YAML config
├── L1model.xml           # MuJoCo XML: bodies, geoms, meshes, optional tendons/actuators
└── mesh/
    ├── part_a.stl
    └── part_b.stl
```

See [how-to/add-a-device.md](add-a-device.md) for the schema reference
and authoring walkthrough.

## Three usage patterns

### Pattern A -- direct path (v0.1.0)

Pass an absolute path to `load_combined_model` for the device YAML:

```python
from assist_sim import load_combined_model

model, data = load_combined_model(
    human_xml=...,                                  # MSK path (from myo_sim)
    device_config="/home/me/projects/MyExo/L1config.yaml",
)
```

<!-- **Pros:** zero setup. Works immediately. No registry edits, no env vars,
no changes to the package. -->

**Note:** the custom device will not appear in
`get_available_combinations()`, the CLI's `assist-sim list`, or any
other discovery output. You have to know the path.

<!-- ### Pattern B -- env var (planned, not yet implemented)

> *Status: deferred. Documented as the intended UX once a user needs it.*

```bash
export ASSIST_SIM_DEVICE_DIRS="/home/me/projects/devices:/shared/lab/devices"
```

On import, `assist_sim.registry` would scan those directories alongside
the bundled `assist_sim/models/`. Custom devices would then resolve by
key the same way bundled ones do:

```python
from assist_sim import load_combined
model, data = load_combined("myoLeg22_2D", "MyExo_L1")   # works, found via env var
```

CLI listing would include them too. The semicolon/colon path separator
follows the platform convention (`os.pathsep`).

If you find yourself needing this pattern, open an issue -- implementation
is mechanical and we'll add it when there's real demand.

### Pattern C -- programmatic registration (planned, not yet implemented)

> *Status: deferred. Documented as the intended UX once a user needs it.*

```python
from assist_sim.registry import register_device_dir, refresh

register_device_dir("/home/me/projects/my_devices")
refresh()

# Now `MyExo_L1` (or whatever's in that dir) is discoverable.
from assist_sim import load_combined
model, data = load_combined("myoLeg22_2D", "MyExo_L1")
```

Same outcome as Pattern B but driven from Python instead of the
environment. Useful when one entrypoint wants different devices than
another (e.g. different training scripts loading different device sets),
or when env vars aren't reliable in your runtime (notebook environments,
CI runners, etc.).

Like B, this is deferred until a user asks. -->

## Naming considerations

- The **directory name** becomes the prefix of the registry key:
  `MyExo/L1config.yaml` → `MyExo_L1`. Use PascalCase by convention.
- The **`device.name`** field inside the YAML is registered as an
  alias and is also used as the namespace prefix on every imported
  body, site, mesh, joint, actuator, and tendon. Match it to the
  directory name + variant for consistency (`MyExo_L1`, not just
  `MyExo`).
- If two devices end up with the same registry key (bundled + custom),
  the bundled one wins. Pick a distinctive name to avoid collision --
  there's no built-in shadowing.

## Custom MSK models

`assist_sim` doesn't currently support user-authored MSK models.
The three pipeline-compatible MSK keys (`myoLeg22_2D`, `myoLeg26_3D`,
`myoLeg80`) are curated and resolved through the `myo_sim` package.

If you have a new MSK you want supported:

1. Upstream the XML to `myo_sim` (see
   [how-to/add-an-msk-model.md](add-an-msk-model.md)).
2. Once it ships in `myo_sim`, add an entry to `_COMPATIBLE_MSK_KEYS`
   in `assist_sim/registry.py`.

There's no "load this MSK XML from a path I provide" pattern
because the pipeline relies on per-MSK conventions (frame orientation,
joint naming, tendon/site naming) that are baked into the device YAMLs.
Adding an unknown MSK risks silent incompatibility with existing
devices, hence the curated registry. If you would like custom MSK + device models, directly adding the device to the MSK model xml following similar procedures and formatting is best practice.

## See also

- [how-to/add-a-device.md](add-a-device.md) -- full device-authoring
  walkthrough
- [device-config-reference.md](../device-config-reference.md) -- YAML
  schema reference
- [troubleshooting.md](../troubleshooting.md) -- diagnosing errors when
  a custom device doesn't compile
