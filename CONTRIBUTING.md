# Contributing to `assist_sim`

This guide covers the development setup, the
codebase layout, and the conventions that keep contributions easy to
review and merge.

## Setup

```bash
git clone https://github.com/neumovelab/assist_sim.git
cd assist_sim
pip install -e .
pip install -r requirements-dev.txt
pytest                # 50 pass without myo_sim; 24 skip
```

For full-suite testing you also need `myo_sim` (ships the baseline MSK
files):

```bash
pip install myo_sim                                          # when on PyPI
# or, interim:
pip install git+https://github.com/MyoHub/myo_sim.git@<tag>
pytest                # all 74 should pass
```

## Repo layout (orientation)

```
assist_sim/                       ← the importable package
├── __init__.py                   ← public API (load_combined_model, etc.)
├── __main__.py                   ← CLI (`python -m assist_sim`)
├── combine.py                    ← Phase 2 of pipeline (MjSpec attach pass)
├── preprocess.py                 ← Phase 1 of pipeline (ElementTree XML pass)
├── registry.py                   ← MSK + device key resolution
├── config.py                     ← DeviceConfig dataclass + per-MSK resolvers
├── utils.py                      ← XML export, terrain strip, mesh dedup
├── validate.py                   ← standalone config validator
├── cache.py                      ← opt-in local cache
├── loading.py                    ← high-level load_combined / resolve_model_path
├── errors.py                     ← error formatting helpers
└── models/                       ← bundled device configs + meshes
    ├── DephyExoBoot/             ← one folder per device
    ├── HMEDI/
    └── ...

tests/                            ← pytest suite
docs/                             ← user-facing documentation
examples/                         ← runnable example scripts
```

See [docs/concepts.md](docs/concepts.md) for the architectural overview
and how `assist_sim` fits with `myo_sim` upstream and `myoassist` downstream.

## Adding a device

The most common contribution. See
[docs/how-to/add-a-device.md](docs/how-to/add-a-device.md) for the full
walkthrough. TL;DR:

1. Create `assist_sim/models/MyDevice/` with `L1config.yaml`, `L1model.xml`,
   and a `mesh/` subdir.
2. The registry autodiscovers it on next import — no code changes needed.
3. Add the device to the `EXPECTED` dict in
   `tests/test_smoke_combinations.py` for each MSK it should work with.
4. Add a row to [docs/available-models.md](docs/available-models.md).
5. If the device behaves differently across MSKs, use the per-MSK
   override schema (see
   [docs/how-to/modify-an-msk-config.md](docs/how-to/modify-an-msk-config.md)).

## Adding an MSK

MSK models live in `myo_sim`, not here. See
[docs/how-to/add-an-msk-model.md](docs/how-to/add-an-msk-model.md).
After the MSK lands in `myo_sim`, add an entry to
`_COMPATIBLE_MSK_KEYS` in `assist_sim/registry.py` and update tests +
docs.

## Pipeline changes

If you're modifying the combination pipeline itself (`preprocess.py`,
`combine.py`, `utils.py`, etc.), bump `__version__` in
`assist_sim/__init__.py`. The cache key includes this string, so a bump
ensures stale cached exports are invalidated automatically.

Also update the smoke regression tuples in
`tests/test_smoke_combinations.py` if your change affects any compiled
`(nq, nu, nbody, nmesh)` signatures. Capture the new tuples by running
the suite once with `pytest -v` and pasting the actual values.

## Style

- **Errors over warnings.** Unresolved name references in YAML configs
  raise `ValueError` (with a "did you mean" suggestion via
  `difflib.get_close_matches`). Don't add `warnings.warn` calls — they
  hide problems.
- **Per-MSK overrides should be opt-in.** Add per-MSK schema support to
  a YAML section only when a real config needs it. Otherwise stay flat.
- **Public surface is minimal.** Things exported from `assist_sim/__init__.py`
  are committed-to. Internal helpers stay underscore-prefixed.
- **No `MjSpec.delete`.** The pipeline targets `mujoco==3.3.3` which
  doesn't expose it. Removals happen at the ElementTree level in Phase 1.

## Tests

```bash
pytest                          # run everything
pytest tests/test_X.py -v       # one file, verbose
pytest -k smoke -v              # match by test name fragment
```

Tests gated by `@needs_myo_sim` skip when `myo_sim` isn't installed.
Don't try to work around the gate — if a test needs MSK files, mark it.

## Pull requests

Branch off `main`. Keep commits focused. CI (`.github/workflows/test.yml`)
runs `pytest` against Python 3.10 / 3.11 / 3.12 + verifies the wheel
builds cleanly on push and PR.

## Questions

Open an issue at <https://github.com/neumovelab/assist_sim/issues>.
