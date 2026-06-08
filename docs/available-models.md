# Available Models

Inventory of what's compatible with what.

## MSK models (via `myo_sim`)

Three MSK keys are pipeline-compatible:

| Key | Source (in `myo_sim`) | DOFs | Notes |
|---|---|---|---|
| `myoLeg22_2D` | `myo_sim/leg/myoLeg22_2D.xml` | 53 | 22-muscle 2D, custom lower-limb-focused variant |
| `myoLeg26_3D` | `myo_sim/leg/myoLeg26_3D.xml` | 60 | 26-muscle 3D, custom lower-limb-focused variant |
| `myoLeg80`    | `myo_sim/leg/myolegs.xml`     | 35 | Upstream 80-muscle MyoLeg with simplified torso, no arm articulation |

### Important MSK differences

- **22/26** face world +X; **80** faces world −Y. Device attachments use
  body frames so this rarely matters at the YAML level, but quickstart's
  initial camera azimuth differs per-MSK.
- **80 has no arm articulation** (single low-poly torso mesh, no shoulder
  / elbow / wrist joints); 22/26 carry a full HAT + arm chain.
- **80 uses a `freejoint`** as the root; 22/26 have slide joints
  (`pelvis_tx`, `pelvis_ty`, `pelvis_tilt`). Keyframes that override
  `pelvis_ty` in 22/26 are not applied to 80 (the joint doesn't exist).
- **Tendon and site naming differs** between the lower-limb-focused 22/26
  and the full-anatomy 80. Per-MSK config overrides handle this.

## Device models

Six device directories under `models/`, contributing seven device keys:

| Device key | Config | Type | Notes |
|---|---|---|---|
| `DephyExoBoot_L1`     | `models/DephyExoBoot/L1config.yaml` | Ankle exoskeleton | Bilateral; battery + Raspberry Pi + boot strapping; ankle ROM override |
| `HMEDI_L1`            | `models/HMEDI/L1config.yaml` | Hip-flexion cable exo | Bilateral; spatial-tendon cables driven by `Exo_R`/`Exo_L`; torso re-parented on myoLeg80 |
| `Humotech_L1`         | `models/Humotech/L1config.yaml` | Ankle exo with cables | Bilateral; pf/df cables (passive); joint-transmission `Exo_R`/`Exo_L` |
| `OpenExo_L1`          | `models/OpenExo/L1config.yaml` | Ankle exo | Bilateral |
| `Tutorial_L1`         | `models/Tutorial/L1config.yaml` | Teaching device | Stripped-down exo for onboarding |
| `OpenSourceLeg_A_L1`  | `models/OpenSourceLeg/A_L1config.yaml` | Transtibial prosthetic | Removes talus + below on the right side; replaces tibia mesh with residual stump |
| `OpenSourceLeg_KA_L1` | `models/OpenSourceLeg/KA_L1config.yaml` | Transfemoral prosthetic | Removes tibia + below on the right side; replaces femur mesh with residual stump |

`OSL_A` and `OSL_KA` are registered as aliases for the OSL keys (via the
device YAML's `device.name`).

## Compatibility matrix

✓ = tested.

| Device | myoLeg22_2D | myoLeg26_3D | myoLeg80 |
|---|:-:|:-:|:-:|
| `DephyExoBoot_L1`     | ✓ | ✓ | ✓ |
| `HMEDI_L1`            | ✓ | ✓ | ✓ |
| `Humotech_L1`         | ✓ | ✓ | ✓ |
| `OpenExo_L1`          | ✓ | ✓ | ✓ |
| `Tutorial_L1`         | ✓ | ✓ | ✓ |
| `OpenSourceLeg_A_L1`  | ✓ | ✓ | ✓ |
| `OpenSourceLeg_KA_L1` | ✓ | ✓ | ✓ |

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
  `assist_sim/registry.py` pointing at the new file in `myo_sim`. See
  [how-to/add-an-msk-model.md](how-to/add-an-msk-model.md).
