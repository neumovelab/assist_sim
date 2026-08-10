# Available Models

This document lists the musculoskeletal (MSK) models and the devices, and shows
which combinations are compatible.

## MSK models (composed by `myo_sim`)

`myo_sim` composes its leg models at run time. To resolve an MSK key,
`assist_sim` calls `myo_sim.build_spec(<model>)`, then removes the bundled
myosuite scene. See [concepts.md](concepts.md). The result is a live `MjSpec`,
never a file. The `assist_sim` key is the same as the `myo_sim` model name:

| Key (= `myo_sim` model) | Base `nq` (position coordinates) | Status |
|---|---|---|
| `myolegs22`   | 39  | **Available**. Planar, 22 muscles, sagittal-plane legs and passive torso; a 26→22 reduction of `myolegs26` |
| `myolegs26`   | 47  | **Available**. 26 muscles, passive torso and legs |
| `myolegs`     | 35  | **Available**. 80 muscles, passive torso |
| `myofullbody` | 129 | **Available**. Full body (torso muscles, arms and legs) |

`myolegs`, `myofullbody` and `myolegs22` need `mujoco>=3.3.4`, because they use
`MjSpec.delete` in memory. `myolegs22` reduces `myolegs26`. `myolegs26` builds
on `3.3.3`. An unknown key raises a clear error. There is no fallback.

### Important MSK notes (myolegs26)

- **Torso scaffold.** `myolegs26` is the equivalent of `myolegs` with 26
  muscles. It has a passive anatomical torso scaffold (spine, ribs and head,
  with no arms and no torso muscles) above the legs with 26 muscles. A device
  that attaches to the torso, for example the torso band of HMEDI, operates on
  it as it does on `myolegs`.
- **Free root.** The root is a `freejoint` on the torso scaffold, which is the
  myosuite convention. It is not a pair of `pelvis_tx`/`pelvis_ty` slide
  joints. The pipeline skips a device keyframe override that targets
  `pelvis_ty`, and gives no message, because that joint does not exist.
- **No keyframe.** The model loads at `qpos0`, which is the assembled standing
  pose. The upstream reference configuration contains that pose. Like
  `myolegs`, the model loads a small distance above the ground. There is no
  `stand` keyframe.

### Important MSK notes (myolegs22)

- **Planar root.** `myolegs26` uses a `freejoint`, but `myolegs22` replaces the
  root with three sagittal-plane DOFs: `pelvis_tx` (fore-aft), `pelvis_ty`
  (vertical) and `pelvis_tilt`. Thus a device keyframe override that targets
  `pelvis_ty` applies. `myolegs22` removes the frontal-plane hip DOFs
  (`hip_adduction` and `hip_rotation`) and the `abd` and `add` muscles, which
  gives 22 muscles instead of 26. It keeps the torso.
- **Keyframes.** The model has five keyframes: `stand`, `walk_left`,
  `walk_right`, `squat` and `lunge`. These keyframes remain after a device
  combination.

### Keyframes are asymmetric across the four MSK models

Only `myolegs22` has keyframes. The other three models compile with `nkey=0`:

| Key | Keyframes | `keyframe_overrides` |
|---|---|---|
| `myolegs22`   | 5 (`stand`, `walk_left`, `walk_right`, `squat`, `lunge`) | applies |
| `myolegs26`   | none | **no effect** |
| `myolegs`     | none | **no effect** |
| `myofullbody` | none | **no effect** |

`keyframe_overrides` changes keyframes that already exist. It never creates a
keyframe. Thus it has no effect on the three MSK models with no keyframes, and
it gives **no error and no warning**. A device YAML file can contain a
`keyframe_overrides:` block that operates only on `myolegs22`. Before you
conclude that the pipeline ignored a value, examine the per-MSK dispatch of the
block.

The three models with no keyframes load at `qpos0`, which is the assembled
standing pose from the upstream reference configuration.

## Device models

There are twelve device directories under `models/`. They give thirteen device
keys:

| Device key | Config | Type | Notes |
|---|---|---|---|
| `Anatomics_L1`        | `models/Anatomics/L1config.yaml` | Ankle exoskeleton | Bilateral instrumented soles and a right shank/foot frame; passive (welded, no actuators) |
| `STRIDE_L2`           | `models/STRIDE/L2config.yaml` | Cable-driven ankle exo (closed linkage) | **S**lack-**T**ensioning via **R**eel-**I**n **D**ifferential **E**lasticity Ankle Exoskeleton. Bilateral **Level 2**. A Watt six-bar behind each ankle (5 hinges per side, 1 net DOF) closes with `equality: joint` quartics. Includes the split shoe. 400 N Bowden cables `cable_r`/`_l`. `contype=2` filters the contact between the device parts. The device clamps the ankle range of motion (ROM) to the coupling fit window |
| `DephyExoBoot_L1`     | `models/DephyExoBoot/L1config.yaml` | Ankle exoskeleton | Bilateral; battery, Raspberry Pi and boot straps; ankle ROM override |
| `HMEDI_L1`            | `models/HMEDI/L1config.yaml` | Hip-flexion cable exo | Bilateral; spatial-tendon cables that `Exo_R`/`Exo_L` drive; the torso piece attaches to `pelvis` on the leg models that have a torso |
| `Hippo_L1`            | `models/Hippo/L1config.yaml` | Hip-flexion exoskeleton | Bilateral; pelvis backplate, hip shell, waistband and AK10-9 housing; thigh braces and cuffs on each femur (welded, visual); ideal fixed-gain hip actuators `Exo_R`/`Exo_L` on `hip_flexion_r`/`_l`; attaches to the pelvis and the femurs (no torso necessary) |
| `Humotech_L1`         | `models/Humotech/L1config.yaml` | Ankle exo with cables | Bilateral; pf/df (plantarflexion/dorsiflexion) cables (passive); joint-transmission actuators `Exo_R`/`Exo_L` |
| `OpenExo_L1`          | `models/OpenExo/L1config.yaml` | Ankle exo | Bilateral |
| `UTAnkleExo_L2`       | `models/UTAnkleExo/L2config.yaml` | Ankle exoskeleton (parallel linkage) | Bilateral; **free-rooted** (not rigid); `<connect>` equalities clamp it to calcn, talus and tibia; spring and cable actuation (`part2part3act_dx`/`_sx`) |
| `Tutorial_L1`         | `models/Tutorial/L1config.yaml` | Teaching device | A simple exoskeleton, to train new users |
| `KFoot_L1`            | `models/KFoot/L1config.yaml` | Transtibial prosthetic | Removes the talus and the parts below it on the right side; residual stump tibia mesh and residuum mass; passive spring-damper ankle (`df_`/`pf_ankle_angle_r`); restores the right-side sensors |
| `NEUankle_L1`         | `models/NEUankle/L1config.yaml` | Powered transtibial prosthetic | The same biological scope as KFoot and OSL_A, but an actuator *drives* the ankle: one hinge `neuankle_ankle_angle_r` with a 50 Nm joint-torque actuator; restores the right-side sensors |
| `OpenSourceLeg_A_L1`  | `models/OpenSourceLeg/A_L1config.yaml` | Transtibial prosthetic | Removes the talus and the parts below it on the right side; replaces the tibia mesh with a residual stump |
| `OpenSourceLeg_KA_L1` | `models/OpenSourceLeg/KA_L1config.yaml` | Transfemoral prosthetic | Removes the tibia and the parts below it on the right side; replaces the femur mesh with a residual stump |

The registry adds `OSL_A` and `OSL_KA` as aliases for the OSL keys. The
`device.name` field of the device YAML file gives these aliases.

`models/` also contains directories that are **not** registry devices. These
are the upper-body collaboration environments below. They have no
`*config.yaml` file. Thus autodiscovery does not find them as device keys, and
they do not appear in the compatibility matrix.

## Upper-body / collaboration environments

The **collaboration environments** are separate from the registry devices
above. Dedicated functions in `assist_sim/upper_body.py` build these upper-body
models; `load_combined` does not build them. Three of them are *composed
models*: a `myo_sim` human with collaborator hardware. MPL is a self-contained
collaborator *robot* with no human, and the code loads it directly.

These environments are **not modular**: each one is a single fixed environment,
not a combination of an MSK model and a device. They are also **not registry
devices**. Thus `list` and `get_available_combinations` do not show them, and
the compatibility matrix does not include them.

| Environment | Description | Builder call | Conversion doc |
|---|---|---|---|
| `Wheelchair`      | A seated human who propels a manual wheelchair | `build_wheelchair(arms=..., torso=...)` | [`models/Wheelchair/CONVERSION.md`](../assist_sim/models/Wheelchair/CONVERSION.md) (**available**) |
| `MPL`             | Standalone bimanual Modular Prosthetic Limb robot | `build_mpl()` | [`models/MPL/CONVERSION.md`](../assist_sim/models/MPL/CONVERSION.md) (**available**) |
| `AuxivoLiftsuit`  | Passive back-exosuit on the muscled `myotorso` | `build_auxivo_liftsuit()` | [`models/AuxivoLiftsuit/CONVERSION.md`](../assist_sim/models/AuxivoLiftsuit/CONVERSION.md) (**available**) |
| `bionic-bimanual` | MyoChallenge biological-arm + MPL-prosthesis manipulation task | `build_bionic_bimanual()` | [`models/MPL/CONVERSION.md`](../assist_sim/models/MPL/CONVERSION.md), bionic section (**available**) |

All four environments are ported and available now. The three composed
environments (`Wheelchair`, `AuxivoLiftsuit` and `bionic-bimanual`) also have a
`build_*_spec()` function. That function returns the `MjSpec` before the
compile. `export_upper_body_xml(spec, path)` serializes that spec to an
independent XML file that you can reload. For the build API, the policy for
mesh sources, the convention for keyframes and `CONVERSION.md`, the export
function, and the detail of each environment, see
[collaboration-environments.md](collaboration-environments.md).

## Compatibility matrix

✓ = tested with frozen smoke signatures. Every device operates with every MSK
model, because all four models have the same passive torso scaffold. Three
devices set `compatible_msk`: `KFoot_L1`, `NEUankle_L1` and `STRIDE_L2`. Each
of them lists all four models, so the list excludes nothing.

| Device | myolegs22 | myolegs26 | myolegs | myofullbody |
|---|:-:|:-:|:-:|:-:|
| `Anatomics_L1`        | ✓ | ✓ | ✓ | ✓ |
| `NEUankle_L1`         | ✓ | ✓ | ✓ | ✓ |
| `DephyExoBoot_L1`     | ✓ | ✓ | ✓ | ✓ |
| `KFoot_L1`            | ✓ | ✓ | ✓ | ✓ |
| `OpenSourceLeg_A_L1`  | ✓ | ✓ | ✓ | ✓ |
| `OpenSourceLeg_KA_L1` | ✓ | ✓ | ✓ | ✓ |
| `STRIDE_L2`           | ✓ | ✓ | ✓ | ✓ |
| `Humotech_L1`         | ✓ | ✓ | ✓ | ✓ |
| `OpenExo_L1`          | ✓ | ✓ | ✓ | ✓ |
| `UTAnkleExo_L2`       | ✓ | ✓ | ✓ | ✓ |
| `Tutorial_L1`         | ✓ | ✓ | ✓ | ✓ |
| `HMEDI_L1`            | ✓ | ✓ | ✓ | ✓ |
| `Hippo_L1`            | ✓ | ✓ | ✓ | ✓ |

## Verify the combinations locally

```bash
python -m assist_sim list
```

The command returns the live `{msk: [device, ...]}` dict. It obeys the
`compatible_msk` filter of each device. It also removes the MSK models that the
installed `myo_sim` cannot resolve.

```python
from assist_sim import get_available_combinations
print(get_available_combinations())
```

## Add to the matrix

- **A new device**: add a new directory under `models/` with `L1config.yaml`,
  `L1model.xml` and the meshes. Autodiscovery finds it. See
  [how-to/add-a-device.md](how-to/add-a-device.md).
- **A new MSK model**: add an entry to `_COMPATIBLE_MSK_KEYS` in
  `assist_sim/registry.py`. The entry connects the key to a
  `myo_sim.build_spec` model name and to its minimum MuJoCo version. See
  [how-to/add-an-msk-model.md](how-to/add-an-msk-model.md).
