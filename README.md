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
from assist_sim import load_combined_model

model, data = load_combined_model(
    human_xml="path/to/myo_sim/leg/myoLeg22_2D.xml",
    device_config="models/DephyExoBoot/L1config.yaml",
)
# `model` and `data` are ready for mj.mj_step / mj.viewer
```

Or by registry key (once `myo_sim` is installed):

```python
from assist_sim import load_combined
model, data = load_combined("myoLeg22_2D", "DephyExoBoot_L1")
```

Or from the CLI:

```bash
python -m assist_sim compile myoLeg22_2D DephyExoBoot_L1 --export combined.xml
python -m assist_sim list
```

Visual inspection of any combination:

```bash
python examples/quickstart.py myoLeg22_2D DephyExoBoot_L1
```

## Available Combinations

| Device key            | myoLeg22_2D | myoLeg26_3D | myoLeg80 |
|-----------------------|:-:|:-:|:-:|
| `DephyExoBoot_L1`     | ✓ | ✓ | ✓ |
| `HMEDI_L1`            | ✓ | ✓ | ✓ |
| `Humotech_L1`         | ✓ | ✓ | ✓ |
| `OpenExo_L1`          | ✓ | ✓ | ✓ |
| `Tutorial_L1`         | ✓ | ✓ | ✓ |
| `OpenSourceLeg_A_L1`  | ✓ | ✓ | ✓ |
| `OpenSourceLeg_KA_L1` | ✓ | ✓ | ✓ |

See [docs/available-models.md](docs/available-models.md) for descriptions of
each device + tested combinations.

## Installation

```bash
# Clone
git clone https://github.com/NeuMove/assist_sim.git
cd assist_sim

# Editable install + myo_sim dependency
pip install -e .
pip install git+https://github.com/MyoHub/myo_sim.git   # interim, until on PyPI
```

Requirements: Python ≥ 3.10, MuJoCo 3.3.3.

For development:

```bash
pip install -r requirements-dev.txt
pytest
```

## Documentation

| Doc | What |
|---|---|
| [docs/getting-started.md](docs/getting-started.md) | Install, run the quickstart, first compiled model |
| [docs/concepts.md](docs/concepts.md) | Architecture: two-phase pipeline, naming, repo split |
| [docs/usage.md](docs/usage.md) | Full API: `load_combined_model`, caching, CLI, registry |
| [docs/device-config-reference.md](docs/device-config-reference.md) | Every YAML field with examples |
| [docs/available-models.md](docs/available-models.md) | Devices + MSKs + which combinations are tested |
| [docs/troubleshooting.md](docs/troubleshooting.md) | Common errors and how to fix |
| [docs/how-to/](docs/how-to/) | Task-focused guides (add a device, modify a config, debug, export) |

## License

Apache 2.0 — see [LICENSE](LICENSE).

## Citation

If you use this in academic work, cite the parent project [myoassist][myoassist]
(citation TBD).

[myo_sim]: https://github.com/MyoHub/myo_sim
[myoassist]: https://github.com/neumovelab/myoassist
