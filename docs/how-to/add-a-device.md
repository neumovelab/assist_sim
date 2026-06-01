# How To: Add a New Device

Authoring a new device from scratch. The pipeline will autodiscover it as
long as it follows the directory layout below.

## Directory layout

```
models/
└── MyDevice/                       # directory name -> half of the registry key
    ├── L1config.yaml               # YAML config (see schema reference)
    ├── L1model.xml                 # MuJoCo XML: bodies, geoms, meshes
    └── mesh/                       # STL files referenced by L1model.xml
        ├── part_a.stl
        ├── part_b.stl
        └── ...
```

The registry key derives from the directory name + the config stem:
`MyDevice/L1config.yaml` → `MyDevice_L1`. Add additional variants by
authoring sibling configs (`A_L1config.yaml`, `KA_L1config.yaml`) — see
OpenSourceLeg for an example.

## Step 1 — Author `L1model.xml`

The device XML is a standalone MuJoCo XML that contains *only* the device's
physical description. It must be loadable by `MjSpec.from_file` on its own
(though it may not simulate meaningfully — bodies don't need to be
connected to a world here; they get grafted onto the MSK at attach time).

Minimum shape:

```xml
<mujoco model="MyDeviceL1">
    <compiler angle="radian"/>

    <default class="main">
        <!-- Optional: default classes the device uses -->
    </default>

    <asset>
        <mesh file="mesh/part_a.stl" name="part_a_geom"/>
        <!-- ...other meshes... -->
    </asset>

    <worldbody>
        <body name="my_device_part_a" pos="0 0 0">
            <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>
            <geom name="part_a_geom" mesh="part_a_geom" type="mesh" rgba="0.3 0.3 0.3 1"/>
            <site name="my_device_attach_marker" pos="0 0 0"/>
        </body>
        <!-- ...other top-level bodies... -->
    </worldbody>

    <!-- Optional: spatial tendons for cable-driven devices -->
    <tendon>
        <spatial name="cable_r" limited="true" range="0 2" width="0.005">
            <site site="cable_r_P1"/>
            <site site="cable_r_P2"/>
        </spatial>
    </tendon>

    <!-- Optional: tendon-transmission actuators -->
    <actuator>
        <general name="Exo_R" tendon="cable_r"
                 gaintype="fixed" gainprm="100 0 0"
                 biastype="none" biasprm="0 0 0"
                 dyntype="none" dynprm="1 0 0"
                 ctrllimited="true" ctrlrange="-1 0" gear="1.0"/>
    </actuator>
</mujoco>
```

**Conventions:**
- Each body that will independently attach to the MSK should be a
  *top-level* `<body>` (direct child of `<worldbody>`). The YAML's
  `attachments` list pulls them in by name.
- Sites can live nested inside bodies; they get prefixed with the device
  name on attach.
- For prosthetics, also include any *replacement meshes* (e.g. residual
  stump meshes) in `<asset>`. They don't need to be referenced by any geom
  in the device XML — they're loaded into the combined spec when the
  pipeline executes `mesh_replacements`.

## Step 2 — Author `L1config.yaml`

The YAML drives the combination. See
[device-config-reference.md](../device-config-reference.md) for the full
schema. Minimum:

```yaml
device:
  name: "MyDevice_L1"
  model_xml: "L1model.xml"

attachments:
  - device_body: "my_device_part_a"
    parent_body: "tibia_r"
```

For a real exoskeleton you'll typically also have:

```yaml
joint_overrides:
  - name: "ankle_angle_r"
    range: [-0.45, 0.349]

actuators:
  - name: "MyDevice_motor_r"
    joint: "ankle_angle_r"
    gaintype: "fixed"
    gainprm: [100, 0, 0]
    ctrlrange: [-1, 1]
    ctrllimited: true

keyframe_overrides:
  stand:
    pelvis_ty: 0.93
```

For a prosthetic, add:

```yaml
body_removals:
  - "talus_r"          # transtibial

mesh_replacements:
  default:
    - geom: "tibia_r_geom_1"
      mesh: "my_device_residual_stump"
  myoLeg80:
    - geom: "r_tibia"
      mesh: "my_device_residual_stump"

geom_removals:
  default:
    - "tibia_r_geom_2"   # drop the fibula geom (covered by stump mesh)
  myoLeg80:
    - "r_fibula"

actuator_removals:
  - "soleus_r"
  - "tibant_r"
  # ... etc

tendon_removals:
  - "soleus_r_tendon"
  - "tib_ant_r_tendon"
```

## Step 3 — Verify discovery

```python
from assist_sim.registry import DEVICE_CONFIGS, refresh
refresh()
print("MyDevice_L1" in DEVICE_CONFIGS)   # should be True
```

Or from the CLI:

```bash
python -m assist_sim list
```

## Step 4 — Compile + visually inspect

```bash
python examples/quickstart.py myoLeg22_2D MyDevice_L1
```

If it opens the viewer and shows the device attached, you're done. Common
issues at this step:

- **`unknown body 'my_device_part_a'`**: the body name in `attachments`
  doesn't match the top-level body in the device XML. Names are
  case-sensitive.
- **Device geometry in the wrong place**: the device body's `pos`/`quat`
  in the XML is interpreted in the parent body's frame. Use `pos`/`quat`
  on the attachment in the YAML to nudge.
- **MSK-specific issues on 80**: if 80 has different parent body names or
  needs a different attachment pose, use the per-MSK `attachments:`
  form (see HMEDI for an example).

## Step 5 — Add to tests + docs

Add the new device to the smoke regression in
`tests/test_smoke_combinations.py` (frozen `(nq, nu, nbody, nmesh)`
tuples). Update [available-models.md](../available-models.md) with a
description and the compatibility matrix.

If you also have a legacy combined XML (e.g. from a prior monolithic
authoring pipeline), drop a parity test into
`tests/test_legacy_parity.py` to confirm the pipeline output matches the
legacy structure.

## See also

- [device-config-reference.md](../device-config-reference.md) — full schema
- [how-to/debug-a-combined-model.md](debug-a-combined-model.md) — when
  things look wrong in the viewer
