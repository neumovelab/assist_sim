# Getting Started

This guide walks through installation, your first compiled model, and visual
inspection. About 5 minutes end-to-end if the prerequisites are in place.

## Prerequisites

- Python ≥ 3.10
- A working MuJoCo ≥ 3.3.4 install (`pip install "mujoco>=3.3.4"`) -- the
  pipeline uses `MjSpec.delete`, which lands in 3.3.4
- `myo_sim` for the baseline MSK models (see install step below)

## Install

```bash
git clone https://github.com/NeuMove/assist_sim.git
cd assist_sim
pip install -e .
```

The editable install picks up `mujoco>=3.3.4`, `PyYAML`, `numpy` from
`pyproject.toml`. It does **not** auto-install `myo_sim` (which is published
separately).

### myo_sim

`myo_sim` provides the baseline MSK models. On the `dev` branch it
*composes* leg models at runtime, and `assist_sim` obtains them via
`myo_sim.build_spec(<model>)`. Three install options:

```bash
# (1) Once it's published to PyPI -- preferred long-term:
pip install myo_sim

# (2) From a git branch in the meantime:
pip install git+https://github.com/MyoHub/myo_sim.git@dev

# (3) Editable, for local development on myo_sim itself:
git clone https://github.com/MyoHub/myo_sim.git
pip install -e ./myo_sim
```

Verify:

```python
import myo_sim
print("myolegs26" in myo_sim._COMPOSED_MODELS)   # True
print(myo_sim.build_spec("myolegs26"))           # an editable MjSpec
```

If `myo_sim` is missing, `assist_sim` will still import and you can use most
of the API, but any call that resolves an MSK (e.g. `load_combined`,
`registry.resolve`) raises an `ImportError` pointing back at the install
instructions.

## First compiled model

```python
from assist_sim import load_combined

model, data = load_combined("myolegs26", "DephyExoBoot_L1")
print(f"nq={model.nq}  nu={model.nu}  nbody={model.nbody}")
# nq=47  nu=28  nbody=37
```

`model` is a standard MuJoCo `MjModel` -- step it, render it,
inspect it, use it as the env's model in your training framework.

## Visual inspection

The `examples/quickstart.py` script opens a paused MuJoCo viewer at the first
keyframe of the combined model:

```bash
python examples/quickstart.py                                    # defaults: myolegs26 + DephyExoBoot_L1
python examples/quickstart.py myolegs26 OpenSourceLeg_KA_L1     # explicit pair
python examples/quickstart.py --list                             # list compatible MSK + device keys
```

The viewer opens paused; drag to rotate, scroll to zoom, ctrl-drag to pan.
Press **Enter** in the terminal to close (closing the window alone won't end
the script).

## Optional: caching

If you reload the same combination repeatedly, opt in to local caching:

```python
model, data = load_combined_model(
    human_xml=...,
    device_config=...,
    cache_dir="./.assist_sim_cache",
)
```

A second call with unchanged inputs skips the full pipeline and loads the
cached XML directly. Cache invalidates on input mtime change or pipeline
version bump. See [usage.md](usage.md) for details.

## What next?

- [concepts.md](concepts.md) -- how the in-memory pipeline works
- [usage.md](usage.md) -- the full API surface
- [how-to/add-a-device.md](how-to/add-a-device.md) -- authoring a new device
- [device-config-reference.md](device-config-reference.md) -- YAML schema reference
