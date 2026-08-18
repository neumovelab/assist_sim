# assist_sim

`assist_sim` combines musculoskeletal (MSK) models with assistive devices for
MuJoCo simulation. It is the middle layer between [myo_sim][myo_sim], which
supplies the MSK models, and downstream training frameworks such as
[myoassist][myoassist].

`assist_sim` takes a baseline MSK model and a device that YAML describes. It
applies the prosthetic surgery (body removals, tendon edits, mesh swaps). Then
it attaches the device. It returns a compiled `MjModel` that you can simulate.
`assist_sim` keeps the baseline MSK model on disk unchanged.

## Quickstart

```python
from assist_sim import load_combined
model, data = load_combined("myolegs26", "DephyExoBoot_L1")
# `model` and `data` are ready for mj.mj_step / mj.viewer
```

This composes the MSK model on demand through `myo_sim`, which you must install
first. It reads the device config from the bundled `assist_sim/models/`
directory. For full control of the paths, if you already have a baseline MSK XML
file on disk:

```python
from assist_sim import load_combined_model

model, data = load_combined_model(
    human_xml="path/to/myolegs26.xml",
    device_config="path/to/your/Device/L1config.yaml",
)
```

### Baseline MSK model, no device

`load_msk` is the counterpart to `load_combined` for a model with no device. Use
it when you give a bare MSK model to a downstream consumer:

```python
from assist_sim import load_msk

model, data = load_msk("myolegs26")
model, data = load_msk("myolegs26", export_xml="myolegs26.xml")
```

It skips the device half of the pipeline (surgery, attachment, actuators,
tendons, equalities, contacts, sensors). It goes directly from the resolved spec
to a compile. Without surgery, the qpos and DOF (degree of freedom) layout stays
the same. Therefore the keyframes also need no decomposition and no rebuild. It
takes the same optional `cache_dir=` argument as `load_combined`.

Or from the CLI:

```bash
python -m assist_sim combine myolegs26 DephyExoBoot_L1 -o combined.xml
python -m assist_sim msk myolegs26 -o out.xml      # baseline MSK, no device
python -m assist_sim list
python -m assist_sim validate myolegs26 DephyExoBoot_L1
```

To inspect any combination visually:

```bash
python examples/quickstart.py myolegs26 DephyExoBoot_L1
```

## Available Combinations

The MSK keys mirror the myo_sim model names. All four keys are connected and
tested: `myolegs22` (planar, 22 muscles, a runtime 26→22 reduction),
`myolegs26` (26 muscles, passive torso), `myolegs` (80 muscles, passive torso)
and `myofullbody` (full body). Every device works with every MSK model, because
all four MSK models carry the passive torso scaffold. Each pair also has a
frozen smoke signature.

| Device key            | myolegs22 | myolegs26 | myolegs | myofullbody |
|-----------------------|:-:|:-:|:-:|:-:|
| `Anatomics_L1`        | ✓ | ✓ | ✓ | ✓ |
| `DephyExoBoot_L1`     | ✓ | ✓ | ✓ | ✓ |
| `HMEDI_L1`            | ✓ | ✓ | ✓ | ✓ |
| `Hippo_L1`            | ✓ | ✓ | ✓ | ✓ |
| `Humotech_L1`         | ✓ | ✓ | ✓ | ✓ |
| `KFoot_L1`            | ✓ | ✓ | ✓ | ✓ |
| `NEUankle_L1`         | ✓ | ✓ | ✓ | ✓ |
| `OpenExo_L1`          | ✓ | ✓ | ✓ | ✓ |
| `OpenSourceLeg_A_L1`  | ✓ | ✓ | ✓ | ✓ |
| `OpenSourceLeg_KA_L1` | ✓ | ✓ | ✓ | ✓ |
| `STRIDE_L2`           | ✓ | ✓ | ✓ | ✓ |
| `Tutorial_L1`         | ✓ | ✓ | ✓ | ✓ |
| `UTAnkleExo_L2`       | ✓ | ✓ | ✓ | ✓ |

See [docs/available-models.md](docs/available-models.md) for a description of
each device and the tested combinations.

## Collaboration environments (upper-body)

`assist_sim` also ships **upper-body collaboration environments** next to the
modular lower-limb devices. Dedicated functions in `assist_sim/upper_body.py`
build them, not `load_combined`. These environments are **not** registry
devices, and they are **not** modular:

```python
from assist_sim.upper_body import (
    build_wheelchair,
    build_mpl,
    build_auxivo_liftsuit,
    build_bionic_bimanual,
)

model, data = build_wheelchair(arms="both", torso="passive")  # "both"|"right"|"left"; "passive"|"muscled"
model, data = build_mpl()               # standalone bimanual MPL robot (no human)
model, data = build_auxivo_liftsuit()   # passive back-exosuit on the muscled myotorso
model, data = build_bionic_bimanual()   # MyoChallenge arm + MPL-prosthesis manipulation task
```

All four environments are available today. The **Wheelchair** is a seated human
who propels a manual wheelchair. The **MPL** is a standalone bimanual Modular
Prosthetic Limb robot. The **AuxivoLiftsuit** is a passive back-exosuit on the
muscled `myotorso`. **bionic-bimanual** is the MyoChallenge manipulation task
that pairs a biological arm with an MPL prosthesis.

Each builder returns `(MjModel, MjData)`. The three composed environments also
give a `build_*_spec()` companion that returns the uncompiled `MjSpec`.
`export_upper_body_xml(spec, path)` serializes that spec to a standalone XML
file that you can reload. See
[docs/collaboration-environments.md](docs/collaboration-environments.md).

## Installation

```bash
# Clone
git clone https://github.com/neumovelab/assist_sim.git
cd assist_sim

# Editable install. myo-sim is a declared dependency, so this pulls it from PyPI too.
pip install -e .
```

Requirements: Python 3.10 to 3.13, MuJoCo `>=3.4,<3.12`. The floor is what `myo_sim`
requires (`MjSpec.delete`, the in-memory surgery, landed in 3.3.4). The ceiling is
measured, not defensive: 3.4 through 3.11 are verified, and MuJoCo keeps widening scalar
`MjSpec` fields and moving `MjData` arrays between releases.

For development:

```bash
pip install -r requirements-dev.txt
pytest
```

## Caching (use it if you are training)

Model building is opt-in cacheable, and the difference matters downstream. Pass `cache_dir=`
to `load_combined` / `load_msk` / `load_combined_model`, or from `myoassist` set one variable:

```bash
export MYOASSIST_CACHE_DIR=~/.cache/myoassist
```

`myoassist` composes a model per CMA-ES candidate and per `SubprocVecEnv` worker, so a
controller-optimization run composes tens of thousands of them. Uncached, the composed pipeline
costs 13-15x more per environment than the static model files MyoAssist 0.1 shipped; cached, it
is back to parity. A hit is 2-8x faster for the three leg models — but *slower* for
`myofullbody`, whose 0.6 MB export costs more to parse than to compose, so leave that one
uncached. Numbers and invalidation rules:
[docs/how-to/export-and-load.md](docs/how-to/export-and-load.md).

## Documentation

| Doc | What |
|---|---|
| [docs/getting-started.md](docs/getting-started.md) | Install, run the quickstart, first compiled model |
| [docs/concepts.md](docs/concepts.md) | Architecture: in-memory pipeline, naming, repo split |
| [docs/usage.md](docs/usage.md) | Full API: `load_combined_model`, the cache, the CLI, the registry |
| [docs/device-config-reference.md](docs/device-config-reference.md) | Every YAML field with examples |
| [docs/available-models.md](docs/available-models.md) | Devices, MSK models, and the tested combinations |
| [docs/collaboration-environments.md](docs/collaboration-environments.md) | Upper-body collaboration environments (wheelchair, MPL, liftsuit, bionic-bimanual) |
| [docs/troubleshooting.md](docs/troubleshooting.md) | Common errors and their fixes |
| [docs/how-to/](docs/how-to/) | Task-focused guides (add a device, use custom devices, modify a config, debug, export) |

## License

Apache 2.0. See [LICENSE](LICENSE).

## Citation

If you use this in academic work, cite the parent project [myoassist][myoassist]
(citation TBD).

[myo_sim]: https://github.com/MyoHub/myo_sim
[myoassist]: https://github.com/neumovelab/myoassist
