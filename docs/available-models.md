# Available Models

Inventory of what's compatible with what.

## MSK models (composed by `myo_sim`)

`myo_sim` composes its leg models at runtime, so each MSK key is resolved by
calling `myo_sim.build_spec(<model>)` and stripping the bundled myosuite scene
(see [concepts.md](concepts.md)) -- returning a live `MjSpec`, never a file. The
assist_sim key matches the `myo_sim` model name:

| Key (= `myo_sim` model) | Base DOFs | Status |
|---|---|---|
| `myolegs26` | 47 | **Available** — 26-muscle, passive torso + legs |
| `myolegs`   | 35 | **Available** — 80-muscle, passive torso |
| `myolegs22` | —  | Planned — a 26→22 mjspec reduction of `myolegs26`, not built yet |

Both buildable models require `mujoco>=3.3.4` (in-memory `MjSpec.delete`). Resolving
a gated/planned key raises a clear error (never a silent fallback).

### Important MSK notes (myolegs26)

- **Torso'd.** `myolegs26` is the 26-muscle counterpart to `myolegs`: a passive
  anatomical torso scaffold (spine, ribs, head; no arms, no torso muscles) over
  the 26-muscle legs. Torso-targeting devices (e.g. HMEDI's torso band) work on
  it, just like on `myolegs`.
- **Free-root base.** The root is a `freejoint` (myosuite convention) on the
  torso scaffold, not `pelvis_tx`/`pelvis_ty` slide joints. Device keyframe
  overrides that target `pelvis_ty` are silently skipped (the joint doesn't exist).
- **No keyframe.** The model loads at `qpos0`, which is the assembled standing
  pose (baked into the reference configuration upstream). Like `myolegs`, it
  loads floating slightly above the ground — there is no `stand` keyframe.

## Device models

Ten device directories under `models/`, contributing eleven device keys:

| Device key | Config | Type | Notes |
|---|---|---|---|
| `Anatomics_L1`        | `models/Anatomics/L1config.yaml` | Ankle exoskeleton | Bilateral instrumented soles + right shank/foot frame; passive (welded, no actuators) |
| `DephyExoBoot_L1`     | `models/DephyExoBoot/L1config.yaml` | Ankle exoskeleton | Bilateral; battery + Raspberry Pi + boot strapping; ankle ROM override |
| `HMEDI_L1`            | `models/HMEDI/L1config.yaml` | Hip-flexion cable exo | Bilateral; spatial-tendon cables driven by `Exo_R`/`Exo_L`; torso piece attached to `pelvis` on both torso'd leg models |
| `Hippo_L1`            | `models/Hippo/L1config.yaml` | Hip-flexion exoskeleton | Bilateral; pelvis backplate + hip shell + waistband + AK10-9 housing, thigh braces/cuffs on each femur (welded, visual); ideal fixed-gain hip actuators `Exo_R`/`Exo_L` on `hip_flexion_r`/`_l`; mounts on pelvis + femurs (no torso needed) |
| `Humotech_L1`         | `models/Humotech/L1config.yaml` | Ankle exo with cables | Bilateral; pf/df cables (passive); joint-transmission `Exo_R`/`Exo_L` |
| `OpenExo_L1`          | `models/OpenExo/L1config.yaml` | Ankle exo | Bilateral |
| `UTAnkleExo_L2`       | `models/UTAnkleExo/L2config.yaml` | Ankle exoskeleton (parallel linkage) | Bilateral; **free-rooted** (non-rigid), clamped to calcn/talus/tibia via `<connect>` equalities; spring + cable-actuated (`part2part3act_dx`/`_sx`) |
| `Tutorial_L1`         | `models/Tutorial/L1config.yaml` | Teaching device | Stripped-down exo for onboarding |
| `KFoot_L1`            | `models/KFoot/L1config.yaml` | Transtibial prosthetic | Removes talus + below on the right side; residual stump tibia mesh; passive spring-damper ankle (`df_`/`pf_ankle_angle_r`) |
| `OpenSourceLeg_A_L1`  | `models/OpenSourceLeg/A_L1config.yaml` | Transtibial prosthetic | Removes talus + below on the right side; replaces tibia mesh with residual stump |
| `OpenSourceLeg_KA_L1` | `models/OpenSourceLeg/KA_L1config.yaml` | Transfemoral prosthetic | Removes tibia + below on the right side; replaces femur mesh with residual stump |

`OSL_A` and `OSL_KA` are registered as aliases for the OSL keys (via the
device YAML's `device.name`).

## Compatibility matrix

✓ = tested (frozen smoke signatures); — = not yet buildable (planned MSK).

| Device | myolegs26 | myolegs | myolegs22 |
|---|:-:|:-:|:-:|
| `Anatomics_L1`        | ✓ | ✓ | — |
| `DephyExoBoot_L1`     | ✓ | ✓ | — |
| `KFoot_L1`            | ✓ | ✓ | — |
| `OpenSourceLeg_A_L1`  | ✓ | ✓ | — |
| `OpenSourceLeg_KA_L1` | ✓ | ✓ | — |
| `Humotech_L1`         | ✓ | ✓ | — |
| `OpenExo_L1`          | ✓ | ✓ | — |
| `UTAnkleExo_L2`       | ✓ | ✓ | — |
| `Tutorial_L1`         | ✓ | ✓ | — |
| `HMEDI_L1`            | ✓ | ✓ | — |
| `Hippo_L1`            | ✓ | ✓ | — |

The `myolegs22` column activates when the 26→22 mjspec reduction lands.

## Verifying combinations locally

```bash
python -m assist_sim list
```

Returns the live `{msk: [device, ...]}` dict honoring each device's
`compatible_msk` filter, filtered further by whichever MSKs are
resolvable in the installed `myo_sim`.

```python
from assist_sim import get_available_combinations
print(get_available_combinations())
```

## Adding to the matrix

- **A new device**: drop a new dir under `models/` with `L1config.yaml`
  + `L1model.xml` + meshes. Picked up automatically. See
  [how-to/add-a-device.md](how-to/add-a-device.md).
- **A new MSK**: add an entry to `_COMPATIBLE_MSK_KEYS` in
  `assist_sim/registry.py` binding the key to a `myo_sim.build_spec` model
  name (and its minimum MuJoCo version). See
  [how-to/add-an-msk-model.md](how-to/add-an-msk-model.md).
