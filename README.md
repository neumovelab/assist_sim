# assist_sim

Programmatic combination of musculoskeletal (MSK) models with assistive devices
for MuJoCo simulation — the middle layer between [myo_sim][myo_sim] (which
ships the MSK models) and downstream training frameworks (e.g.
[myoassist][myoassist]).

`assist_sim` takes a baseline MSK and a YAML-described device, applies any
prosthetic surgery (body removals, tendon edits, mesh swaps), attaches the
device, and returns a compiled `MjModel` ready to simulate. The baseline MSK
on disk is never modified.

## Quickstart

```python
from assist_sim import load_combined
model, data = load_combined("myolegs26", "DephyExoBoot_L1")
# `model` and `data` are ready for mj.mj_step / mj.viewer
```

This composes the MSK on demand through `myo_sim` (must be installed) and the
device config from the bundled `assist_sim/models/`. For full path control (if
you already have a baseline MSK XML on disk):

```python
from assist_sim import load_combined_model

model, data = load_combined_model(
    human_xml="path/to/myolegs26.xml",
    device_config="path/to/your/Device/L1config.yaml",
)
```

Or from the CLI:

```bash
python -m assist_sim combine myolegs26 DephyExoBoot_L1 -o combined.xml
python -m assist_sim list
```

Visual inspection of any combination:

```bash
python examples/quickstart.py myolegs26 DephyExoBoot_L1
```

## Available Combinations

MSK keys mirror the myo_sim model names. All four are wired and tested:
`myolegs22` (planar 22-muscle, a runtime 26→22 reduction), `myolegs26`
(26-muscle, passive torso), `myolegs` (80-muscle, passive torso) and
`myofullbody` (full body). Every device works with every MSK — all four carry the
passive torso scaffold — and each pairing has a frozen smoke signature.

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

See [docs/available-models.md](docs/available-models.md) for descriptions of
each device + tested combinations.

## Collaboration environments (upper-body)

Alongside the modular lower-limb devices, `assist_sim` ships **upper-body
collaboration environments** built by dedicated functions in
`assist_sim/upper_body.py`, rather than by `load_combined`. These are **not**
registry devices and are **not** modular:

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

All four are available today: the **Wheelchair** (seated human propelling a
manual wheelchair), the **MPL** (a standalone bimanual Modular Prosthetic Limb
robot), the **AuxivoLiftsuit** (a passive back-exosuit on the muscled
`myotorso`), and **bionic-bimanual** (the MyoChallenge manipulation task pairing
a biological arm with an MPL prosthesis). Each builder returns
`(MjModel, MjData)`; the three composed environments also expose a
`build_*_spec()` companion returning the uncompiled `MjSpec`, which
`export_upper_body_xml(spec, path)` serializes to a standalone, reloadable XML.
See [docs/collaboration-environments.md](docs/collaboration-environments.md).

## Installation

```bash
# Clone
git clone https://github.com/NeuMove/assist_sim.git
cd assist_sim

# Editable install + myo_sim dependency
pip install -e .
pip install git+https://github.com/MyoHub/myo_sim.git   # interim, until on PyPI
```

Requirements: Python ≥ 3.10, MuJoCo ≥ 3.3.4 (for in-memory `MjSpec.delete`).

For development:

```bash
pip install -r requirements-dev.txt
pytest
```

## Documentation

| Doc | What |
|---|---|
| [docs/getting-started.md](docs/getting-started.md) | Install, run the quickstart, first compiled model |
| [docs/concepts.md](docs/concepts.md) | Architecture: in-memory pipeline, naming, repo split |
| [docs/usage.md](docs/usage.md) | Full API: `load_combined_model`, caching, CLI, registry |
| [docs/device-config-reference.md](docs/device-config-reference.md) | Every YAML field with examples |
| [docs/available-models.md](docs/available-models.md) | Devices + MSKs + which combinations are tested |
| [docs/collaboration-environments.md](docs/collaboration-environments.md) | Upper-body collaboration environments (wheelchair, MPL, liftsuit, bionic-bimanual) |
| [docs/troubleshooting.md](docs/troubleshooting.md) | Common errors and how to fix |
| [docs/how-to/](docs/how-to/) | Task-focused guides (add a device, use custom devices, modify a config, debug, export) |

## License

Apache 2.0 — see [LICENSE](LICENSE).

## Citation

If you use this in academic work, cite the parent project [myoassist][myoassist]
(citation TBD).

[myo_sim]: https://github.com/MyoHub/myo_sim
[myoassist]: https://github.com/neumovelab/myoassist
