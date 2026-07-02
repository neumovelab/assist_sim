# CLAUDE.md

`assist_sim` is the middle layer between [`myo_sim`](https://github.com/MyoHub/myo_sim) (ships the
baseline musculoskeletal models) and downstream training (e.g. `myoassist`): it takes a baseline MSK
plus a YAML-described assistive device, applies any prosthetic surgery, attaches the device, and
returns a compiled `MjModel`. The baseline MSK on disk is never modified.

Read `CONTRIBUTING.md` and `docs/concepts.md` before any substantial change.

## Quickstart (pip)

```bash
pip install -e .
pip install -r requirements-dev.txt
pytest                              # 63 pass, 1 skip with myo_sim (HMEDI needs myolegs / Phase 2)
ruff check . && ruff format --check .

python -m assist_sim list           # discoverable combinations (also: validate, combine)
python examples/quickstart.py myolegs26 DephyExoBoot_L1   # visual inspection
```

> Note: uv migration is planned for tooling parity with `myo_sim` (which uses `uv sync` /
> `uv run pytest -n auto`). Until then, assist_sim is pip-based.

## Architecture

Two-phase pipeline (the design exists because `mujoco==3.3.3` has no `MjSpec.delete`):

1. **Phase 1 — `preprocess.py`** (ElementTree): all removals + cascade cleanup on the human XML.
2. **Phase 2 — `combine.py`** (MjSpec): attach device bodies, edit attributes, add actuators,
   rebuild keyframes. Additive only.

`registry.py` resolves MSK keys (from `myo_sim`) and auto-discovers device configs by scanning
`assist_sim/models/*/L1config.yaml`. `config.py` holds the `DeviceConfig` dataclass + per-MSK
resolvers.

## Conventions (load-bearing)

- **Errors over warnings.** Unresolved YAML references raise `ValueError` with a "did you mean"
  suggestion. No `warnings.warn`.
- **No `MjSpec.delete`.** Targets `mujoco==3.3.3` (pinned, deliberate). All removals happen in
  Phase 1 at the ElementTree level.
- **Minimal public surface.** Only names exported from `assist_sim/__init__.py` are committed API;
  internals stay underscore-prefixed.
- **Devices autodiscover.** Drop a folder under `assist_sim/models/`; no code change needed.
- **Bump `__version__`** in `assist_sim/__init__.py` for pipeline changes — it's part of the cache
  key, so a bump invalidates stale exports. Update smoke tuples in
  `tests/test_smoke_combinations.py` if `(nq, nu, nbody, nmesh)` signatures change.

## myo_sim integration status

Baseline MSKs live in `myo_sim`, not here; assist_sim resolves them via `_COMPATIBLE_MSK_KEYS` in
`registry.py`. On myo_sim's `mm_refactor` branch, leg models are **composed at runtime** (no static
XML). **Phase 1 (done):** `_resolve_msk` calls `myo_sim.build_spec(<model>)`, serializes the
returned `MjSpec` to XML, strips the bundled myosuite scene (`utils._strip_myosuite_scene`), and
caches a model-only XML that feeds the existing preprocess+combine pipeline. assist_sim MSK keys
mirror the myo_sim model names. Only `myolegs26` (legs-only, 26-muscle) is buildable on the pinned
`mujoco==3.3.3`; `myolegs` (80-muscle, passive-torso, needs `MjSpec.delete`) is gated on
`mujoco>=3.3.4` and `myolegs22` awaits a 26→22 mjspec reduction — both raise a clear error when
resolved.

**Phase 2 (next):** bump the pin to `mujoco>=3.3.4`, switch Phase-1 removals to in-memory
`spec.delete()` (dropping the XML round-trip), which also unlocks `myolegs` and torso'd devices
(e.g. HMEDI). Background write-up: `myo_sim-leg-integration.md`.

## More detail

`CONTRIBUTING.md` (setup, adding a device/MSK, style, PRs) and `docs/` (architecture, usage,
device-config reference, troubleshooting, how-to guides).
