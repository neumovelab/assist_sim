# Changelog

All notable changes to `assist_sim` will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] — Initial release

First public release. Ports the model-combination pipeline and aligns the package
for PyPI distribution.

### Added

- **Two-phase combination pipeline.** Phase 1 is an `ElementTree` pass
  over the MSK XML that handles removals (bodies, geoms, actuators,
  tendons, terrain) and cascade cleanup; Phase 2 is an `MjSpec`-driven
  attach pass that grafts device bodies onto the MSK, imports device-side
  tendons + actuators, applies joint overrides, and rebuilds keyframes
  by joint name. Runs on `mujoco==3.3.3`.
- **Explicit MSK registry.** `_COMPATIBLE_MSK_KEYS` in
  `assist_sim.registry` enumerates the three pipeline-compatible MSK
  variants (`myoLeg22_2D`, `myoLeg26_3D`, `myoLeg80`); files are resolved
  through `myo_sim` via `importlib.resources`.
- **Device autodiscovery.** Any `models/<DeviceDir>/<variant>config.yaml`
  is picked up on import. Seven bundled devices: `DephyExoBoot_L1`,
  `HMEDI_L1`, `Humotech_L1`, `OpenExo_L1`, `OpenSourceLeg_A_L1`,
  `OpenSourceLeg_KA_L1`, `Tutorial_L1`.
- **Per-MSK config overrides.** YAML sections `attachments`,
  `tendon_modifications`, `keyframe_overrides`, `actuator_removals`,
  `tendon_removals`, `mesh_replacements`, and `geom_removals` accept a
  `default:` + `<msk_key>:` dispatch form for MSK-specific variations.
- **Device-side tendon + actuator import.** Spatial tendons and
  tendon-transmission actuators authored in a device's `model.xml` are
  imported into the combined spec with the device prefix.
- **Geom removals.** New `geom_removals` schema section for surgical
  geom deletion (e.g. dropping the fibula geom when a residual stump mesh
  covers both tibia + fibula on transtibial amputation).
- **Model-only exports.** The terrain include is stripped from exported
  XMLs; downstream consumers (e.g. `myoassist.terrains`) layer the scene
  on top. Skybox-rendering compatibility preserved by keeping the
  texture+material binding the renderer requires.
- **Public Python API.** `load_combined_model`, `load_combined`,
  `resolve_model_path`, `get_available_combinations`,
  `validate_combination`, `DeviceConfig`.
- **CLI.** `python -m assist_sim list | validate | combine` and the
  `assist-sim` script entry point.
- **Opt-in local caching.** `cache_dir=` argument to
  `load_combined_model`; cache key includes input file mtimes and
  `__version__` so stale entries invalidate automatically.
- **Quickstart example.** `examples/quickstart.py` opens a paused
  `mujoco.viewer` at the first keyframe of a combined model with
  per-MSK initial camera pose (result of 22/26 vs 80 muscle world frame differences).
- **Documentation.** `README.md` plus `docs/` tree covering concepts,
  usage, YAML schema reference, available models, troubleshooting, and
  task-focused how-to guides.
- **Test suite.** 50 unit tests covering preprocess passes, registry
  autodiscovery, per-MSK overrides, tendon edits, mesh dedup, validator,
  terrain strip, and end-to-end smoke (50 tests). 24 additional tests
  gated by `@needs_myo_sim` are skipped when `myo_sim` isn't installed.
- **Wheel-installable package.** Device configs + XMLs + meshes ship
  inside the wheel under `assist_sim/models/`. `pip install assist_sim`
  gives a user the full bundled device set with no extra setup.

[Unreleased]: https://github.com/neumovelab/assist_sim/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/neumovelab/assist_sim/releases/tag/v0.1.0
