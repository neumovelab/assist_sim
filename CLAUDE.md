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
pytest                              # 51 pass with myo_sim installed
ruff check . && ruff format --check .

python -m assist_sim list           # discoverable combinations (also: validate, combine)
python examples/quickstart.py myolegs26 DephyExoBoot_L1   # visual inspection
```

> Note: uv migration is planned for tooling parity with `myo_sim` (which uses `uv sync` /
> `uv run pytest -n auto`). Until then, assist_sim is pip-based.

## Architecture

Single-phase, fully in-memory pipeline (requires `mujoco>=3.3.4` for `MjSpec.delete`):

1. **Resolve** — `registry._resolve_msk` calls `myo_sim.build_spec(<model>)` and strips the
   bundled myosuite scene (`utils.strip_myosuite_scene_spec`), returning a live human `MjSpec`.
   The composed model is never serialized (torso-composed models don't round-trip through
   `to_xml`), so everything downstream edits the spec directly.
2. **Combine** (`combine.py`, MjSpec): surgery via `spec.delete` (body/geom/actuator/tendon
   removals — cascades subtrees + referencing elements; manual scrub of contact `<pair>`s), then
   attach device bodies, edit attributes, add actuators, and rebuild keyframes (decomposed by
   joint name before surgery, restored after the final compile).

`registry.py` resolves MSK keys (composed by `myo_sim`) and auto-discovers device configs by
scanning `assist_sim/models/*/L1config.yaml`. `config.py` holds the `DeviceConfig` dataclass +
per-MSK resolvers. `preprocess.py` is now just device-XML prep (`prepare_device_xml`) + the
`KeyframeData` container; static device XMLs still round-trip fine.

## Conventions (load-bearing)

- **Errors over warnings.** Unresolved YAML references raise `ValueError` with a "did you mean"
  suggestion. No `warnings.warn`.
- **In-memory surgery.** Requires `mujoco>=3.3.4` (`MjSpec.delete`). Removals run on the live
  human `MjSpec`; `spec.delete` cascades subtrees + sensors/actuators/tendons but NOT contact
  `<pair>`s (scrubbed manually). Device XMLs are still massaged at the text level.
- **Minimal public surface.** Only names exported from `assist_sim/__init__.py` are committed API;
  internals stay underscore-prefixed.
- **Devices autodiscover.** Drop a folder under `assist_sim/models/`; no code change needed.
- **Bump `__version__`** in `assist_sim/__init__.py` for pipeline changes — it's part of the cache
  key, so a bump invalidates stale exports. Update smoke tuples in
  `tests/test_smoke_combinations.py` if `(nq, nu, nbody, nmesh)` signatures change.

## myo_sim integration status

Baseline MSKs live in `myo_sim`, not here; assist_sim resolves them via `_COMPATIBLE_MSK_KEYS` in
`registry.py`. On myo_sim's `mm_refactor` branch, leg models are **composed at runtime** (no static
XML), and assist_sim's keys mirror the myo_sim model names. `_resolve_msk` calls
`myo_sim.build_spec(<model>)`, strips the bundled myosuite scene, and returns a live `MjSpec` that
`combine.py` mutates in place. Buildable now (on `mujoco>=3.3.4`): **`myolegs26`** (26-muscle,
passive torso + legs) and **`myolegs`** (80-muscle, passive torso). **`myolegs22`** has no source yet (a
planned 26→22 mjspec reduction) and raises a clear `ValueError` when resolved.

Torso-composed models (`myolegs`, and any device that needs a torso like HMEDI) are why the
pipeline is in-memory: their serialized `to_xml` doesn't round-trip (nested unnamed `<default>` →
"empty class name" on reload), so assist_sim never serializes the human model. Background
write-up: `myo_sim-leg-integration.md`.

## More detail

`CONTRIBUTING.md` (setup, adding a device/MSK, style, PRs) and `docs/` (architecture, usage,
device-config reference, troubleshooting, how-to guides).
