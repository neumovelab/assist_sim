# How To: Use Custom Devices

`assist_sim` contains thirteen bundled devices: Anatomics, Dephy, HMEDI,
Hippo, Humotech, KFoot, NEUankle, OpenExo, STRIDE, UTAnkleExo, Tutorial,
and the two OSL variants. You can also write your own device, for example
for a new exoskeleton, for a different prosthetic geometry, or for an
internal lab project. This guide tells you where a custom device goes and
how to use it.

## Directory layout

Each device has the same shape, bundled or user-authored:

```
<DeviceName>/
├── L1config.yaml         # the YAML config
├── L1model.xml           # MuJoCo XML: bodies, geoms, meshes, optional tendons/actuators
└── mesh/
    ├── part_a.stl
    └── part_b.stl
```

See [how-to/add-a-device.md](add-a-device.md) for the schema reference and
the full procedure.

## Three usage patterns

### Pattern A: direct path (v0.1.0)

Give an absolute path to the device YAML file in `load_combined_model`:

```python
from assist_sim import load_combined_model

model, data = load_combined_model(
    human_xml="myolegs26.xml",                      # an MSK XML on disk
    device_config="/home/me/projects/MyExo/L1config.yaml",
    msk_key="myolegs26",                            # selects per-MSK blocks
)
```

`myo_sim` composes its musculoskeletal (MSK) models at run time and
supplies no XML for them. Write one out first if you need this form:

```bash
python -m assist_sim msk myolegs26 -o myolegs26.xml
```

<!-- **Pros:** zero setup. Works immediately. No registry edits, no env vars,
no changes to the package. -->

**Note:** the custom device does not appear in
`get_available_combinations()`, in `assist-sim list` on the command line,
or in any other discovery output. You must know the path.

<!-- ### Pattern B: env var (planned, not yet implemented)

> *Status: deferred. This text describes the intended user experience for
> the time when a user needs it.*

```bash
export ASSIST_SIM_DEVICE_DIRS="/home/me/projects/devices:/shared/lab/devices"
```

On import, `assist_sim.registry` would scan those directories and the
bundled `assist_sim/models/` directory. Custom devices would then resolve
by key in the same way as the bundled devices:

```python
from assist_sim import load_combined
model, data = load_combined("myolegs26", "MyExo_L1")   # works, found via env var
```

The command-line listing would also include them. The path separator, a
semicolon or a colon, follows the platform convention (`os.pathsep`).

If you need this pattern, open an issue. The implementation is mechanical,
and we will add it when there is real demand.

### Pattern C: programmatic registration (planned, not yet implemented)

> *Status: deferred. This text describes the intended user experience for
> the time when a user needs it.*

```python
from assist_sim.registry import register_device_dir, refresh

register_device_dir("/home/me/projects/my_devices")
refresh()

# Now `MyExo_L1` (or whatever's in that dir) is discoverable.
from assist_sim import load_combined
model, data = load_combined("myolegs26", "MyExo_L1")
```

The result is the same as Pattern B, but Python drives it in place of the
environment. Pattern C is useful when one entry point needs different
devices than another entry point. Two training scripts that load different
device sets are an example. It is also useful when env vars are not
reliable in your runtime, such as notebook environments and CI runners.

Like Pattern B, we defer this pattern until a user asks for it. -->

## Naming considerations

- The **directory name** becomes the prefix of the registry key:
  `MyExo/L1config.yaml` → `MyExo_L1`. Use PascalCase by convention.
- The **`device.name`** field inside the YAML file is an alias in the
  registry. It is also the namespace prefix on every imported body, site,
  mesh, joint, actuator and tendon. Make it the same as the directory name
  plus the variant, for consistency. Use `MyExo_L1`, not `MyExo`.
- If a custom device and a bundled device have the same registry key, the
  bundled device wins. There is no built-in shadow mechanism, so select a
  distinctive name to prevent a collision.

## Custom MSK models

`assist_sim` does not support user-authored MSK models at this time. The
pipeline supports four curated MSK keys: `myolegs22`, `myolegs26`,
`myolegs` and `myofullbody`. `assist_sim` resolves them through the
`myo_sim` package.

To get support for a new MSK model:

1. Send the XML upstream to `myo_sim` (see
   [how-to/add-an-msk-model.md](add-an-msk-model.md)).
2. When it is in a `myo_sim` release, add an entry to
   `_COMPATIBLE_MSK_KEYS` in `assist_sim/registry.py`.

There is no pattern to load an MSK XML from a path that you give. The
pipeline depends on per-MSK conventions, such as the frame orientation, the
joint names, and the tendon and site names. The device YAML files contain
these conventions. An unknown MSK model can therefore be incompatible with
the existing devices, and no message tells you. The registry has a curated
list for this reason.

For a custom MSK model with a custom device, add the device directly to the
MSK model XML. This is the best practice. Use the same procedures and the
same format.

## See also

- [how-to/add-a-device.md](add-a-device.md): the full procedure to write a
  device
- [device-config-reference.md](../device-config-reference.md): the YAML
  schema reference
- [troubleshooting.md](../troubleshooting.md): how to diagnose errors when
  a custom device does not compile
