# Available Models

Inventory of what's compatible with what.

## MSK models (composed by `myo_sim`)

`myo_sim` composes its leg models at runtime, so each MSK key is resolved by
calling `myo_sim.build_spec(<model>)` and stripping the bundled myosuite scene
(see [concepts.md](concepts.md)) -- returning a live `MjSpec`, never a file. The
assist_sim key matches the `myo_sim` model name:

| Key (= `myo_sim` model) | Base DOFs | Status |
|---|---|---|
| `myolegs22`   | 39  | **Available** — planar 22-muscle, sagittal-plane legs + passive torso; a 26→22 reduction of `myolegs26` |
| `myolegs26`   | 47  | **Available** — 26-muscle, passive torso + legs |
| `myolegs`     | 35  | **Available** — 80-muscle, passive torso |
| `myofullbody` | 129 | **Available** — full-body (torso muscles + arms + legs) |

`myolegs`, `myofullbody`, and `myolegs22` (which reduces `myolegs26`) require
`mujoco>=3.3.4` (in-memory `MjSpec.delete`); `myolegs26` builds on `3.3.3`.
Resolving an unknown key raises a clear error (never a silent fallback).

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

### Important MSK notes (myolegs22)

- **Planar root.** Unlike `myolegs26`'s `freejoint`, `myolegs22` replaces the
  root with three sagittal-plane DOFs — `pelvis_tx` (fore-aft), `pelvis_ty`
  (vertical), `pelvis_tilt` — so device keyframe overrides targeting `pelvis_ty`
  apply. The frontal-plane hip DOFs (`hip_adduction`/`hip_rotation`) and the
  `abd`/`add` muscles are removed (26 → 22 muscles); the torso is kept.
- **Keyframes.** Ships five keyframes — `stand`, `walk_left`, `walk_right`,
  `squat`, `lunge` — that survive device combination.

## Device models

Ten device directories under `models/`, contributing eleven device keys:

| Device key | Config | Type | Notes |
|---|---|---|---|
| `Anatomics_L1`        | `models/Anatomics/L1config.yaml` | Ankle exoskeleton | Bilateral instrumented soles + right shank/foot frame; passive (welded, no actuators) |
| `DephyExoBoot_L1`     | `models/DephyExoBoot/L1config.yaml` | Ankle exoskeleton | Bilateral; battery + Raspberry Pi + boot strapping; ankle ROM override |
| `HMEDI_L1`            | `models/HMEDI/L1config.yaml` | Hip-flexion cable exo | Bilateral; spatial-tendon cables driven by `Exo_R`/`Exo_L`; torso piece attached to `pelvis` on the torso'd leg models |
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

`models/` also contains directories that are **not** registry devices —
the upper-body collaboration environments below. They have no `*config.yaml`,
so they are not autodiscovered as device keys and do not appear in the
compatibility matrix.

## Upper-body / collaboration environments

Separate from the registry devices above are the **collaboration
environments**: single *composed models* (a `myo_sim` human + collaborator
hardware) built by dedicated functions in `assist_sim/upper_body.py`, not by
`load_combined`. They are **not modular** — each is one fixed environment, not
an MSK × device pairing — and are **not registry devices**, so they are absent
from `list` / `get_available_combinations` and the compatibility matrix.

| Environment | Description | Builder call | Conversion doc |
|---|---|---|---|
| `Wheelchair`     | Seated human propelling a manual wheelchair | `build_wheelchair(arms=..., torso=...)` | [`models/Wheelchair/CONVERSION.md`](../assist_sim/models/Wheelchair/CONVERSION.md) — **available** |
| `MPL`            | Modular Prosthetic Limb — a robotic prosthetic arm/hand | `build_mpl(...)` | `models/MPL/CONVERSION.md` — *forthcoming* |
| `AuxivoLiftsuit` | Passive torso back-exosuit | `build_auxivo_liftsuit(...)` | `models/AuxivoLiftsuit/CONVERSION.md` — *forthcoming* |

Only the Wheelchair is fully ported today; MPL and AuxivoLiftsuit are being
added in parallel. See
[collaboration-environments.md](collaboration-environments.md) for the build
API, the mesh-sourcing policy, the keyframe / `CONVERSION.md` convention, and
the Wheelchair worked example.

## Compatibility matrix

✓ = tested (frozen smoke signatures). Every device works with every MSK model:
all four share the passive torso scaffold, so no device pins its
`compatible_msk`.

| Device | myolegs22 | myolegs26 | myolegs | myofullbody |
|---|:-:|:-:|:-:|:-:|
| `Anatomics_L1`        | ✓ | ✓ | ✓ | ✓ |
| `DephyExoBoot_L1`     | ✓ | ✓ | ✓ | ✓ |
| `KFoot_L1`            | ✓ | ✓ | ✓ | ✓ |
| `OpenSourceLeg_A_L1`  | ✓ | ✓ | ✓ | ✓ |
| `OpenSourceLeg_KA_L1` | ✓ | ✓ | ✓ | ✓ |
| `Humotech_L1`         | ✓ | ✓ | ✓ | ✓ |
| `OpenExo_L1`          | ✓ | ✓ | ✓ | ✓ |
| `UTAnkleExo_L2`       | ✓ | ✓ | ✓ | ✓ |
| `Tutorial_L1`         | ✓ | ✓ | ✓ | ✓ |
| `HMEDI_L1`            | ✓ | ✓ | ✓ | ✓ |
| `Hippo_L1`            | ✓ | ✓ | ✓ | ✓ |

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
