# How To: Add a New Device

This guide tells you how to write a new device. The pipeline finds the
device automatically if the device obeys the directory layout below.

## Directory layout

```
assist_sim/models/                  # the scanned root (MODELS_ROOT)
└── MyDevice/                       # directory name -> half of the registry key
    ├── L1config.yaml               # YAML config (see schema reference)
    ├── L1model.xml                 # MuJoCo XML: bodies, geoms, meshes
    └── mesh/                       # STL files referenced by L1model.xml
        ├── part_a.stl
        ├── part_b.stl
        └── ...
```

The registry key comes from the directory name plus the config stem:
`MyDevice/L1config.yaml` → `MyDevice_L1`. To add more variants, write
sibling configs (`A_L1config.yaml`, `KA_L1config.yaml`). OpenSourceLeg is
an example.

## Step 1: Write `L1model.xml`

The device XML is a standalone MuJoCo XML file. It contains only the
physical description of the device. `MjSpec.from_file` must load it on its
own. The device XML does not have to simulate correctly. Its bodies do not
connect to a world here, because the pipeline attaches them to the
musculoskeletal (MSK) model at attach time.

The minimum content is:

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
- Each body that attaches to the MSK model on its own must be a *top-level*
  `<body>`. A top-level body is a direct child of `<worldbody>`. The
  `attachments` list in the YAML file selects these bodies by name.
- Sites can be inside bodies. The pipeline adds the device name as a prefix
  to each site at attach time.
- For a prosthetic, also put the *replacement meshes* in `<asset>`. The
  residual stump meshes are an example. No geom in the device XML has to
  refer to them. The pipeline loads them into the combined spec when it
  applies `mesh_replacements`.

## Step 2: Write `config.yaml`

The YAML file controls the combination. See
[device-config-reference.md](../device-config-reference.md) for the full
schema. The minimum content is:

```yaml
device:
  name: "MyDevice_L1"
  model_xml: "L1model.xml"

attachments:
  - device_body: "my_device_part_a"
    parent_body: "tibia_r"
```

An exoskeleton usually also needs these blocks:

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

A **free-floating mechanism** clamps to the leg at more than one point. A
parallel-linkage exoskeleton is an example, but a rigid strap-on is not.
Give each root body its own `<freejoint>` in the XML. Attach the body to
`world`. Then hold the body with `equality` constraints in place of rigid
re-parenting. See `UTAnkleExo` and the
[`equality` reference](../device-config-reference.md#equality).

```yaml
attachments:
  - device_body: "part3_r"       # top-level body with a <freejoint>
    parent_body: "world"
    pos: [-0.157, 0.035, -0.583]  # world pose so it sits on the leg
    quat: [0.121, 0, 0, 0.993]

equality:
  - type: "connect"
    device_body: "part3_r"
    parent_body: "calcn_r"
    anchor: [-0.071, 0.05, 0.005]  # in part3_r's local frame
```

Caution: `body_removals` uses `spec.delete`, which cascades. It removes the
subtree and every actuator, tendon and sensor that refers to the subtree.

For a prosthetic, add these blocks:

```yaml
body_removals:
  - "talus_r"          # transtibial; cascades to calcn_r and toes_r

mesh_replacements:
  default:
    - geom: "tibia_r_geom_1"
      mesh: "my_device_residual_stump"
  myolegs:
    - geom: "tibia_r"
      mesh: "my_device_residual_stump"

geom_removals:
  default:
    - "tibia_r_geom_2"   # drop the fibula geom (covered by stump mesh)
  myolegs:
    - "fibula_r"

actuator_removals:
  - "soleus_r"
  - "tibant_r"
  # ... etc

tendon_removals:
  - "soleus_r_tendon"
  - "tib_ant_r_tendon"
```

A real amputation keeps some biarticular muscles. You must therefore
re-anchor such a muscle to the residual bone. `tendon_modifications` runs
before the removals, so it can do this:

```yaml
tendon_modifications:
  default:
    - name: "gastroc_r_tendon"       # spans knee + ankle; the knee survives
      wraps:
        - replace_site: "gastroc_r_med_gas_r-P3"
          new_body: "tibia_r"        # residual tibia, at the cut plane
          pos: [-0.02631, -0.21890, 0.02034]

actuator_overrides:
  default:
    - name: "gastroc_r"
      lengthrange: [0.202513, 0.229413]   # re-derived for the shorter path
```

See [modify-an-msk-config.md](modify-an-msk-config.md#worked-example-osl_ka-transfemoral-re-anchoring)
for the four wrap-edit ops and the rules for the new point positions.

## Step 3: Verify discovery

```python
from assist_sim.registry import DEVICE_CONFIGS, refresh
refresh()
print("MyDevice_L1" in DEVICE_CONFIGS)   # should be True
```

Or use the command line:

```bash
python -m assist_sim list
```

## Step 4: Compile and examine the model

```bash
python examples/quickstart.py myolegs26 MyDevice_L1
```

If the viewer opens and shows the device attached, the device is correct.
These problems are usual at this step:

- **`unknown body 'my_device_part_a'`**: the body name in `attachments`
  does not agree with the top-level body in the device XML. Names are
  case-sensitive.
- **The device geometry is in the wrong position**: MuJoCo reads the `pos`
  and `quat` of the device body in the frame of the parent body. Set `pos`
  and `quat` on the attachment in the YAML file to adjust the position.
- **The device is incorrect on 80 only**: model 80 can have different
  parent body names, or it can need a different attachment pose. Use the
  per-MSK `attachments:` form. HMEDI is an example.

## Step 5: Add tests and docs

Add the new device to the `EXPECTED` dict in
`tests/test_smoke_combinations.py`. Add one frozen `(nq, nu, nbody, nmesh)`
tuple for each MSK model that you support. That dict protects the whole
pipeline, because it shows an unwanted change in the combined model. Get
the tuples from a single probe run. Then put the actual values in the dict:

```bash
python -m assist_sim combine myolegs26 MyDevice_L1
```

Update [available-models.md](../available-models.md) with a description and
the compatibility matrix.

For a prosthetic that re-anchors muscles, also add a case to
`tests/test_tendon_reanchor.py`. That test checks that the re-anchored
muscles stay in the model after the removal cascade. It also checks that
each muscle starts inside its `lengthrange`.

## See also

- [device-config-reference.md](../device-config-reference.md): the full
  schema
- [how-to/debug-a-combined-model.md](debug-a-combined-model.md): what to do
  when the viewer shows an incorrect model
