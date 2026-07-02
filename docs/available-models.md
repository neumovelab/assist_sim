# Available Models

Inventory of what's compatible with what.

## MSK models (composed by `myo_sim`)

`myo_sim` composes its leg models at runtime, so each MSK key is resolved by
calling `myo_sim.build_spec(<model>)`, serializing to XML, and stripping the
bundled myosuite scene (see [concepts.md](concepts.md)). The assist_sim key
matches the `myo_sim` model name. Three are registered; which are buildable
depends on the installed MuJoCo:

| Key (= `myo_sim` model) | Base DOFs | Status |
|---|---|---|
| `myolegs26` | 47 | **Available** (Phase 1) — 26-muscle, legs-only |
| `myolegs`   | —  | Phase 2 — 80-muscle; needs `mujoco>=3.3.4` (passive-torso conversion uses `MjSpec.delete`) |
| `myolegs22` | —  | Planned — a 26→22 mjspec reduction of `myolegs26`, not built yet |

`myolegs26` is the only MSK buildable on the pinned `mujoco==3.3.3`. Resolving
a gated/planned key raises a clear error (never a silent fallback).

### Important MSK notes (myolegs26)

- **Legs-only.** `myolegs26` is intentionally trunk-less — no HAT, torso, arms,
  or head. Devices whose attachments target a `torso` body (e.g. HMEDI's torso
  band) need a torso'd MSK (`myolegs`, Phase 2).
- **Free-root base.** The root is a `freejoint` (myosuite convention), not
  `pelvis_tx`/`pelvis_ty` slide joints. Device keyframe overrides that target
  `pelvis_ty` are silently skipped (the joint doesn't exist); the standing
  height comes from the base `stand` keyframe's free-root height.
- **Ships a `stand` keyframe** (fully at-rest, feet on the pedestal). The scene
  is stripped by assist_sim, but the keyframe's authored joint values are
  preserved by name through the pipeline.

## Device models

Six device directories under `models/`, contributing seven device keys:

| Device key | Config | Type | Notes |
|---|---|---|---|
| `DephyExoBoot_L1`     | `models/DephyExoBoot/L1config.yaml` | Ankle exoskeleton | Bilateral; battery + Raspberry Pi + boot strapping; ankle ROM override |
| `HMEDI_L1`            | `models/HMEDI/L1config.yaml` | Hip-flexion cable exo | Bilateral; spatial-tendon cables driven by `Exo_R`/`Exo_L`; torso re-parented on myolegs |
| `Humotech_L1`         | `models/Humotech/L1config.yaml` | Ankle exo with cables | Bilateral; pf/df cables (passive); joint-transmission `Exo_R`/`Exo_L` |
| `OpenExo_L1`          | `models/OpenExo/L1config.yaml` | Ankle exo | Bilateral |
| `Tutorial_L1`         | `models/Tutorial/L1config.yaml` | Teaching device | Stripped-down exo for onboarding |
| `OpenSourceLeg_A_L1`  | `models/OpenSourceLeg/A_L1config.yaml` | Transtibial prosthetic | Removes talus + below on the right side; replaces tibia mesh with residual stump |
| `OpenSourceLeg_KA_L1` | `models/OpenSourceLeg/KA_L1config.yaml` | Transfemoral prosthetic | Removes tibia + below on the right side; replaces femur mesh with residual stump |

`OSL_A` and `OSL_KA` are registered as aliases for the OSL keys (via the
device YAML's `device.name`).

## Compatibility matrix

✓ = tested (frozen smoke signatures); — = not yet buildable (Phase 2 / planned);
n/a = device needs a torso'd MSK.

| Device | myolegs26 | myolegs | myolegs22 |
|---|:-:|:-:|:-:|
| `DephyExoBoot_L1`     | ✓ | — | — |
| `OpenSourceLeg_A_L1`  | ✓ | — | — |
| `OpenSourceLeg_KA_L1` | ✓ | — | — |
| `Humotech_L1`         | ✓ | — | — |
| `OpenExo_L1`          | ✓ | — | — |
| `Tutorial_L1`         | ✓ | — | — |
| `HMEDI_L1`            | n/a (needs torso) | — | — |

Phase 1 wires `myolegs26`. The `myolegs` and `myolegs22` columns activate
in Phase 2 (mujoco 3.3.4 bump) and when the 26→22 reduction lands, respectively.

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
