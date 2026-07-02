# Changelog

All notable changes to `assist_sim` will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.3.0] — in-memory pipeline (myo_sim integration, Phase 2)

Moves model surgery in-memory and unlocks the torso-composed models.

### Changed

- **Single-phase, in-memory pipeline.** `registry._resolve_msk` returns a live
  human `MjSpec` (composed by `myo_sim`, scene stripped) and `ModelCombiner`
  does all removals on it via `spec.delete` (`utils.strip_myosuite_scene_spec`,
  keyframe decompose/rebuild by joint name). The human model is never serialized
  to XML — required for torso-composed models, whose `to_xml` doesn't round-trip
  (merged fragment defaults yield a nested unnamed `<default>`). `resolve()` now
  returns `(MjSpec, Path)`.
- **`mujoco>=3.3.4`** (from `==3.3.3`) for `MjSpec.delete`. The dev env uses the
  lowest compatible version to stay close to myoassist's `3.3.3`.
- **`myolegs` (80-muscle, passive torso) is now buildable**, along with every
  device — including HMEDI (torso band) on `myolegs`.
- **OSL configs re-tuned** for the composed `myolegs` geom names
  (`r_fibula`→`fibula_r`, `r_tibia`→`tibia_r`, `r_femur`→`femur_r`).

### Removed

- **The ElementTree "Phase 1" preprocess** (removal/inline/cascade/terrain
  passes) — superseded by in-memory `spec.delete`. `preprocess.py` now holds
  only device-XML prep + the `KeyframeData` container.

### Notes

- `spec.delete` cascades subtrees + sensors/actuators/tendons but not contact
  `<pair>`s (scrubbed manually). It also removes spanning muscles the old
  re-anchor path preserved (e.g. OSL_KA quadriceps), which is valid for
  above-knee amputation. `tendon_modifications` on a *surviving* tendon are not
  yet supported in-memory (no bundled config needs it).
- Caching / XML export is unreliable for composed models (combined `to_xml`
  hits the same nested-default issue); `load_combined` raises on `cache_dir`.

## [0.2.0] — myo_sim composed-model integration (Phase 1)

Wires `assist_sim` onto `myo_sim`'s `mm_refactor` branch, where leg models are
composed at runtime rather than shipped as static XML.

### Changed

- **MSK resolution now composes via `myo_sim.build_spec`.** `_resolve_msk` calls
  `myo_sim.build_spec(<model>)`, serializes the returned `MjSpec`, strips the
  bundled myosuite scene, and caches a model-only XML (keyed by the myo_sim +
  mujoco versions) that feeds the existing preprocess+combine pipeline. Replaces
  the old `importlib.resources` `(subpackage, filename)` lookup, which pointed at
  static files that no longer exist. `_COMPATIBLE_MSK_KEYS` entries are now
  `_MskSource(myo_sim_model, min_mujoco, note)`.
- **MSK keys now mirror the myo_sim model names** (`myolegs`, `myolegs26`,
  `myolegs22`), replacing the former `myoLeg80` / `myoLeg26_3D` / `myoLeg22_2D`.
- **`myolegs26`** (legs-only, 26-muscle) is wired and tested; it is the only MSK
  buildable on the pinned `mujoco==3.3.3`.
- **`get_available_combinations`** uses a cheap availability check (no compile at
  import time; models are composed lazily on first resolve).

### Added

- **`utils._strip_myosuite_scene`** — removes the myosuite scene (floor,
  backdrop, pedestal, logo, scene lights/cameras) so composed MSKs enter the
  pipeline model-only.

### Gated / planned

- **`myolegs`** (80-muscle, passive torso) needs `mujoco>=3.3.4` for its
  `MjSpec.delete`-based conversion — resolving it on 3.3.3 raises a clear
  `ImportError` (Phase 2).
- **`myolegs22`** has no source yet (a planned 26→22 mjspec reduction) and
  raises a clear `ValueError` when resolved.

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
  variants (`myolegs22`, `myolegs26`, `myolegs`); files are resolved
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
