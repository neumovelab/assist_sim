# Getting Started

This guide gives the installation steps, your first compiled model, and the
visual inspection. The procedure takes approximately 5 minutes if you have the
prerequisites.

## Prerequisites

- Python ≥ 3.10
- A MuJoCo ≥ 3.3.4 installation (`pip install "mujoco>=3.3.4"`). The pipeline
  uses `MjSpec.delete`, which is available in 3.3.4.
- `myo_sim`, for the baseline musculoskeletal (MSK) models. See the
  installation step below.

## Install

```bash
git clone https://github.com/neumovelab/assist_sim.git
cd assist_sim
pip install -e .
```

The editable installation gets `mujoco>=3.3.4`, `PyYAML` and `numpy` from
`pyproject.toml`. It does **not** install `myo_sim` automatically, because
`myo_sim` is published separately.

### myo_sim

`myo_sim` supplies the baseline MSK models. On the `dev` branch, `myo_sim`
*composes* the leg models at run time, and `assist_sim` gets them with
`myo_sim.build_spec(<model>)`. There are three installation options:

```bash
# (1) Once it is published to PyPI (preferred long-term):
pip install myo_sim

# (2) From a git branch in the meantime:
pip install git+https://github.com/MyoHub/myo_sim.git@dev

# (3) Editable, for local development on myo_sim itself:
git clone https://github.com/MyoHub/myo_sim.git
pip install -e ./myo_sim
```

Verify the installation:

```python
import myo_sim
print("myolegs26" in myo_sim._COMPOSED_MODELS)   # True
print(myo_sim.build_spec("myolegs26"))           # an editable MjSpec
```

If `myo_sim` is not installed, `assist_sim` imports correctly and most of the
API is available. But a call that resolves an MSK model, for example
`load_combined` or `registry.resolve`, raises an `ImportError`. That error
refers you to these installation instructions.

## First compiled model

```python
from assist_sim import load_combined

model, data = load_combined("myolegs26", "DephyExoBoot_L1")
print(f"nq={model.nq}  nu={model.nu}  nbody={model.nbody}")
# nq=47  nu=28  nbody=52
```

`model` is a standard MuJoCo `MjModel`. You can step it, render it, or inspect
it. You can also use it as the model of the environment in your training
framework.

## A baseline MSK with no device

`load_msk` gives an MSK model with no device. It does not run the device steps
of the pipeline. It compiles the resolved spec directly:

```python
from assist_sim import load_msk

model, data = load_msk("myolegs26")
print(f"nq={model.nq}  nu={model.nu}  nbody={model.nbody}")
# nq=47  nu=26  nbody=38
```

The command-line interface (CLI) does the same, and it also writes an export:

```bash
python -m assist_sim msk myolegs26 -o out.xml
```

The two forms accept the optional `cache_dir=` or `--cache-dir` argument. The
section below describes it.

## Visual inspection

The `examples/quickstart.py` script opens a paused MuJoCo viewer. The viewer
shows the first keyframe of the combined model. If the model has no keyframe,
the viewer shows `qpos0`. Only `myolegs22` has keyframes.

```bash
python examples/quickstart.py                                    # defaults: myolegs26 + DephyExoBoot_L1
python examples/quickstart.py myolegs26 OpenSourceLeg_KA_L1     # explicit pair
python examples/quickstart.py --list                             # list compatible MSK + device keys
```

The viewer opens in the paused state. Drag to rotate the view. Scroll to zoom.
Ctrl-drag to pan. Press **Enter** in the terminal to close the script. If you
close only the window, the script continues.

## Optional: the cache

If you load the same combination many times, enable the local cache:

```python
model, data = load_combined_model(
    human_xml=...,
    device_config=...,
    cache_dir="./.assist_sim_cache",
)
```

A second call with the same inputs does not run the pipeline. It loads the
cached XML file directly. The cache becomes invalid when an input modification
time changes, or when the pipeline version increases. For more data, see
[usage.md](usage.md).

## What next?

- [concepts.md](concepts.md): how the pipeline operates in memory
- [usage.md](usage.md): the full public API
- [how-to/add-a-device.md](how-to/add-a-device.md): how to author a new device
- [device-config-reference.md](device-config-reference.md): the YAML schema
  reference
