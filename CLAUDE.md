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
pytest                              # 50 pass, 24 skip without myo_sim
ruff check . && ruff format --check .

python -m assist_sim list           # discoverable combinations (also: validate, combine)
python examples/quickstart.py myoLeg22_2D DephyExoBoot_L1   # visual inspection
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
`registry.py`. On myo_sim's `mm_refactor_mjspec` branch, leg models are **composed at runtime**
(no static XML, no path accessor), which the current resolver does not yet handle. The proposed
fix (expose a leg `MjSpec` via myo_sim's `FRAGMENT_SPEC_BUILDERS`, serialize to XML, then run the
existing pipeline) is written up in `myo_sim-leg-integration.md` — pending agreement with myo_sim.

## More detail

`CONTRIBUTING.md` (setup, adding a device/MSK, style, PRs) and `docs/` (architecture, usage,
device-config reference, troubleshooting, how-to guides).
