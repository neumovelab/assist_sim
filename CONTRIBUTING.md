# Contributing to `assist_sim`

This guide covers the development setup, the codebase layout, and the
conventions that keep contributions easy to review and merge.

## Setup

```bash
git clone https://github.com/neumovelab/assist_sim.git
cd assist_sim
pip install -e .
pip install -r requirements-dev.txt
pytest                # most tests skip without myo_sim
```

`myo_sim` composes the baseline musculoskeletal (MSK) models and is a hard runtime
dependency (`myo-sim>=0.2.1`), so `pip install -e .` already brings it in from PyPI and the
full suite runs:

```bash
pytest                # 307 collected: 305 pass, 2 skip (~3 min)
```

One skip is platform-gated: `test_staging_works_from_a_read_only_device_directory` needs POSIX
file modes, so it runs on Linux and macOS and skips on Windows.

If you deliberately install without it, most tests skip on the `needs_myo_sim` gate and three
modules (`test_msk_only.py`, `test_tendon_reanchor.py`, `test_upper_body_export.py`) skip
completely at import.

## Repo layout (orientation)

```
assist_sim/                       ← the importable package
├── __init__.py                   ← public API (load_combined_model, etc.)
├── __main__.py                   ← CLI (`python -m assist_sim`)
├── combine.py                    ← the pipeline (in-memory MjSpec surgery + attach)
├── preprocess.py                 ← device-XML prep + KeyframeData container
├── registry.py                   ← MSK + device key resolution
├── config.py                     ← DeviceConfig dataclass, strict YAML loader, per-MSK resolvers
├── utils.py                      ← XML export, myosuite-scene strip, default hoist, mesh dedup
├── validate.py                   ← standalone config validator
├── cache.py                      ← opt-in local cache (keys, atomic publish, recovery)
├── loading.py                    ← high-level load_combined / load_msk / resolve_model_path
├── reduce_legs.py                ← in-spec 26 to 22 planar reduction (produces myolegs22)
├── canonical_keyframes.py        ← fallback poses for an MSK that ships no keyframes
├── root_frame.py                 ← to_planar_root, the CO-only freejoint to pelvis-DOF reframe
├── upper_body.py                 ← composed upper-body envs (wheelchair, MPL, liftsuit, bionic)
├── errors.py                     ← error formatting helpers
└── models/                       ← bundled device configs + meshes
    ├── DephyExoBoot/             ← one folder per device
    ├── HMEDI/
    └── ...

tests/                            ← pytest suite
docs/                             ← user-facing documentation
examples/                         ← runnable example scripts
```

See [docs/concepts.md](docs/concepts.md) for the architectural overview and for
the position of `assist_sim` between `myo_sim` upstream and `myoassist`
downstream.

## How to add a device

This is the most common contribution. See
[docs/how-to/add-a-device.md](docs/how-to/add-a-device.md) for the full
walkthrough. In summary:

1. Create `assist_sim/models/MyDevice/` with `L1config.yaml`, `L1model.xml`,
   and a `mesh/` subdir.
2. The registry finds the device automatically at the next import. You do not
   have to change any code.
3. Add the device to the `EXPECTED` dict in
   `tests/test_smoke_combinations.py` for each MSK model it must work with.
4. Add a row to [docs/available-models.md](docs/available-models.md).
5. If the device behaves differently on different MSK models, use the per-MSK
   override schema (see
   [docs/how-to/modify-an-msk-config.md](docs/how-to/modify-an-msk-config.md)).

## How to add an MSK model

The MSK models live in `myo_sim`, not here. See
[docs/how-to/add-an-msk-model.md](docs/how-to/add-an-msk-model.md).
After `myo_sim` includes the MSK model, add an entry to
`_COMPATIBLE_MSK_KEYS` in `assist_sim/registry.py`. Then update the tests and
the docs.

## Pipeline changes

If you change the combination pipeline itself (`preprocess.py`, `combine.py`,
`utils.py`, etc.), bump `version` in `pyproject.toml` and add a `CHANGELOG.md` entry.
`assist_sim.__version__` is read from installed package metadata, so do **not** look for it
in `assist_sim/__init__.py`, and note that on an editable install it does not change until you
reinstall. The cache key folds in the newest source mtime as well as the version, so your
local edits invalidate cached exports even before you bump.

Also update the smoke regression tuples in
`tests/test_smoke_combinations.py` if your change affects a compiled
`(nq, nu, nbody, nmesh)` signature. To get the new tuples, run the suite once
with `pytest -v`. Then copy the actual values into the test file.

## Style

- **Errors over warnings.** An unresolved name reference in a YAML config
  raises `ValueError`. The error includes a "did you mean" suggestion from
  `difflib.get_close_matches`. Do not add `warnings.warn` calls, because they
  hide problems.
- **Make per-MSK overrides optional.** Add per-MSK schema support to a YAML
  section only when a real config needs it. If no config needs it, keep the
  section flat.
- **Keep the public surface minimal.** `assist_sim/__init__.py` exports the
  committed API. Internal helpers keep an underscore prefix.
- **In-memory surgery (`mujoco>=3.4,<3.12`).** The removals run on the live human
  `MjSpec` through `spec.delete`. `spec.delete` cascades to subtrees and to the
  elements that reference them. The pipeline removes contact `<pair>` elements
  manually. The pipeline keeps the human model in memory and does not serialize it
  mid-flow, because torso-composed models do not round-trip through a bare `to_xml`.
  Exporting is different: `utils.export_combined_xml` applies the default hoist/name fixups
  that make those models reload, and all four MSK keys round-trip through it.
- **Re-anchor before you remove.** `tendon_modifications` runs before every
  removal. Move a muscle that the surgery keeps onto the residual bone while its
  wrap sites still exist. After the removals run, the cascade already removed
  that muscle. A re-anchor changes the path of a muscle, so also set an
  `actuator_overrides` `lengthrange` for it. The compiler keeps the authored
  ranges.

## Tests

```bash
pytest                          # run everything
pytest tests/test_X.py -v       # one file, verbose
pytest -k smoke -v              # match by test name fragment
```

Tests with the `@needs_myo_sim` marker skip when `myo_sim` is not installed. Do
not work around the gate. If a test needs MSK model files, add the marker to it.

## Pull requests

Make your branch from `main`. Keep each commit focused. On each push and pull
request, CI (`.github/workflows/test.yml`) runs four jobs:

- `lint` — `ruff check` and `ruff format --check`, once.
- `test` — `pytest` on the pinned stack from `requirements.txt`: Python 3.10 to
  3.13 on Linux, and the two ends of that span on Windows and macOS.
- `mujoco-range` — `pytest` at both ends of the MuJoCo range that
  `pyproject.toml` declares, installed **without** the pin file. This is the only
  job that tests what a plain `pip install assist-sim` gives a user. If it fails
  on a new MuJoCo release, fix the incompatibility and then raise the ceiling in
  `pyproject.toml`. Do not relax the job.
- `package` — builds the wheel and verifies that it carries the device configs,
  the meshes and the `py.typed` marker.

## Questions

Open an issue at <https://github.com/neumovelab/assist_sim/issues>.
