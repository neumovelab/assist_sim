# Changelog

All notable changes to `assist_sim` will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Surgical muscle re-anchoring** (myodesis/myoplasty). `tendon_modifications`
  now runs **before** the removals, so a biarticular muscle that the amputation
  preserves moves onto the residual bone while its wrap points still exist.
  Before this change it ran after the removals, when the `spec.delete` cascade
  already removed the muscle and gave no message. The affected muscles are
  `rect_fem` and `hamstrings` across a transfemoral amputation, and `gastroc`
  across a transtibial amputation. `MjsTendon` exposes no readable wrap list, but none
  is necessary: a wrap stores its site or geom by *name* and resolves that name
  at compile. A move of the named element therefore also moves the wrap. The
  code edits elements and does not rebuild tendons, which keeps the tendon and
  actuator objects in place and keeps the `ctrl` order.
- **`actuator_overrides`** sets the `lengthrange` of a muscle, as the re-anchor
  feature above requires. The compiler keeps authored lengthrange values
  (`LRopt.useexisting=1`), so a re-anchored muscle would otherwise describe a
  path that it no longer has. The author gave `rectfem_r` the range
  `[0.321, 0.510]`, but the muscle now operates over `[0.227, 0.329]`, which
  starts below its own lower bound. A kinematic joint sweep with a 1% span
  margin supplies the new values, and it reproduces the authored intact ranges
  to within 3% of span. Muscles that keep their original path also keep their
  authored values.
- **`body_removals` per musculoskeletal (MSK) model.** The section takes the
  `default:` + `<msk_key>:` dispatch form, the same as the sections near it.
- **`tests/test_tendon_reanchor.py`**: 16 tests. They cover all four wrap
  operations, a re-anchored tendon that survives the cascade, and the actuator
  order. They also cover the lengthrange consequence, the error for a retired
  operation, and the error for an unknown reference.
- **`load_msk(msk_key, export_xml=..., cache_dir=...)`** is the counterpart to
  `load_combined` that uses no device. It gives a bare MSK model to a
  downstream consumer. Almost all of `ModelCombiner.combine` does device work,
  so this function goes directly from the resolved spec to a compile. There is
  no surgery, so the qpos/dof layout stays the same and the keyframe decompose
  and rebuild step is also unnecessary. The CLI makes the function
  available as `python -m assist_sim msk MSK [-o OUT] [--cache-dir DIR]`.
  `load_msk` caches in the same manner as `load_combined`: a composed MSK model
  has no source file on disk and no device, so its identity is
  `(msk_key, assist_sim version, myo_sim token)`.

> The MSK-only export contains the model only, the same as the combined path:
> no ground, no hfield, no floor. An export-time terrain removal does not cause
> this. The composed spec never had terrain, because `registry._resolve_msk`
> removes the full myosuite scene at resolve time. `utils._strip_terrain` stays
> in the code, but no caller gives it `terrain_paths`, so it never runs.
>
> Both exports keep some scene elements. `_strip_scene_visual` removes the
> myosuite headlight. Then `_ensure_minimal_visual` **adds** a soft headlight
> and a neutral gradient skybox, so a bare file renders with light instead of a
> black void. A downstream scene can override both, but a caller that needs a
> specific backdrop must replace the skybox in each case.

### Changed

- **The wrap-edit operations are now `reposition_site` / `replace_site` /
  `reposition_geom` / `replace_geom`.** The geom pair is new and necessary. The
  hamstrings of the 80-muscle lineage cross condylar wrap cylinders. If one
  cylinder stays on a body that the pipeline removes, the cascade removes the
  tendon, even when every site moved.
- **`drop_site` is retired** and raises an error. It needs an editable wrap
  list, and `MjsTendon` does not expose one. An immediate error is better than
  a skipped surgical edit that the author expects the code to apply. The error
  message names the replacement operations.
- **`tendon_modifications` validates its targets.** It already raised an error
  for an unknown tendon. It now also raises an error for an unknown target site
  or geom, and for an unknown `new_body`. Each error gives a "did you mean"
  list. `assist-sim validate` makes the same three checks statically, and also
  checks every `actuator_overrides` name.
- **`tests/test_smoke_combinations.py` freezes the 16 amputee smoke signatures
  again**: the four amputee devices (`KFoot_L1`, `NEUankle_L1`,
  `OpenSourceLeg_A_L1`, `OpenSourceLeg_KA_L1`) across all four MSK models.
  Muscles that the pipeline removed before now stay in the model, so `nu`
  increases in each combination.
  `OSL_KA` on the 80-muscle lineage also loses the 3 degrees of freedom (DOFs)
  of `patella_r`.

### Fixed

- **The transfemoral residual femur had the mass of the intact segment.** The
  pipeline removes `tibia_r` and all bodies below it. The prosthetic side then
  weighed more than the intact leg (100.7% of it on `myolegs26`). `femur_r` now
  carries a mass-per-length fit that stops at the cut plane (y = −0.283012):
  6.59257 kg on 22/26, and 6.06705 kg on the 80-muscle lineage. The
  `diaginertia`, `ipos` and `iquat` values agree with the new mass.
- **`patella_r` stayed in the model after the knee removal on the 80-muscle
  lineage.** It is a *sibling* of `tibia_r`, so the cascade did not reach it and
  left 3 unconstrained DOFs inside the socket. The config now removes it for
  `OSL_KA` on `myolegs` / `myofullbody`.
- **`femur2_col_r` collided with the prosthesis.** That collider extends to
  y = −0.457 with `contype=1`, which is 0.174 m of active geometry beyond the
  transfemoral cut. The config now removes it. The 22/26 lineage has no
  equivalent geom.
- **The transfemoral model lost the sensors on the prosthetic side.** The
  cascade reduced `myolegs26` from twelve sensors to seven, and the intact side
  kept all of its sensors. The config now restores them for `OSL_KA`, which
  gives eleven sensors on `myolegs26`: the two touch sensors on all four MSK
  models, and the two `jointlimitfrc` sensors on 22/26. The 80-muscle lineage
  contains no `jointlimitfrc` sensor, so a restored sensor there would be the
  only sensor of that type in the model.
- **The configs removed muscles with no justification.** `OSL_KA` now keeps
  `addmagDist_r`, `addmagMid_r` and `addmagProx_r`, which do not cross the
  removed knee. It also keeps `grac_r`, `sart_r` and `tfl_r`, which act at the
  hip that stays in the model, and the pipeline re-anchors these three. `OSL_A`
  now keeps `bfsh_r`, which runs from `femur_r` to `tibia_r`. Both of these
  bodies stay in the model after a transtibial amputation, so `bfsh_r` had no
  wrap point on a removed body. `hamstrings_r`, `bflh_r`, `semimem_r`,
  `semiten_r`, `recfem_r`, `addmagIsch_r` and the `gastroc` / `gasmed_r` /
  `gaslat_r` group change from removal to re-anchoring.
- **`assist-sim validate` printed the repr of an `MjSpec` object.**
  `resolve_model_path` returns `(MjSpec, Path)` and not a path, because the
  pipeline composes the human model in memory. The command now reports the MSK
  key and the body count of the model.

## [0.6.1] — lower-limb mass/inertia re-tare (STRIDE, Anatomics, Hippo, Humotech)

Measured masses replacing placeholder inertials. No `(nq, nu, nbody, nmesh)`
signature changes anywhere.

Three different corrections, in increasing order of fidelity:

- **STRIDE / Anatomics shoe** — mass re-tared, `diaginertia` scaled by the mass
  ratio (uniform-density correction, `I_new = I_old × m_new/m_old`), `pos`,
  `quat` and `euler` untouched. The tensors were placeholders to begin with, so
  this fixes the mass without fixing the distribution; see the note below.
- **Hippo** — every tensor recomputed from mesh geometry at the density that
  reproduces the assigned mass, per `modeling-tools/AGENTS.md`.
- **Humotech** — mass and principal moments taken from supplied SolidWorks CAD.

### Changed

- **Hippo re-tared to 4.900 kg**, 79.1% of it at the pelvis. Was ~6 g of total
  model mass: all eight bodies sat at `mass=0.001` with
  `diaginertia="1e-9 1e-9 1e-9"` and a copy-pasted `pos`/`quat`. The two AK10-9
  hip actuators are 960 g each (catalog); the remaining 2.98 kg is spread over
  the printed structure at one uniform density. Every tensor is now computed
  from the compiled mesh geometry — note MuJoCo recentres and reorients mesh
  assets onto their principal axes and pushes the correction into
  `geom_pos`/`geom_quat`, so raw STL coordinates are *not* the body frame.
- **Humotech hardware re-tared to 2194.05 g/side** from CAD: shank assembly
  602.82 g, foot assembly 1591.23 g. SolidWorks prints off-diagonals as products
  of inertia rather than tensor components, so they are negated before use —
  verified by reproducing its own printed principal moments and by the
  parallel-axis transform to the origin. Only mass and the principal moments are
  transferable: the STLs were exported in per-part local frames (both groups
  start at `z=0`, while the CAD COMs are at `z=+183.20` and `z=-58.54`), so the
  CAD COM and tensor orientation cannot be mapped. Principal moments are
  rotational invariants and survive; `pos` is left as authored and `quat` comes
  from each body's own mesh axes.
- **HMEDI re-tared to 4.500 kg**: 4.2 kg on `hmedi_torso`, 300 g over the nine
  fabric/attachment bodies by mesh volume, every tensor recomputed from geometry.
  Nine of the ten bodies were previously `mass="0.001"` (or `0.00001`) with
  `diaginertia="1e-9 1e-9 1e-9"`; the tenth carried 233 g, a value that also
  appears three times in `DephyExoBoot`, so it was a copied default rather than a
  measurement. Torso lands at 2524 kg/m^3 and the fabric at 1077.
- **OpenExo re-tared to 1.950 kg/side.** `exo_shaft` + `exo_cuff` weld to `tibia`
  and `exo_blade` welds to `calcn`, so only the shank-vs-foot split is
  dynamically meaningful; the split within the shank group is not. Comparing each
  mesh volume against what the part physically is shows only the shaft is a stub
  — a carbon foot plate really is ~100 cm^3 and a curved shank shell ~75-90, so
  `exo_blade` (104.67) and `exo_cuff` (86.45) are faithful, while 34.54 cm^3
  cannot be an actuator assembly. The two shells therefore take geometry-based
  masses at material density (carbon 1600, shell 1400) and `exo_shaft` takes the
  remainder, 1.66 kg, which also absorbs off-board mass the 1.95 kg figure covers
  but the model does not represent.
- **OpenExo shaft inertia envelope-corrected.** The mesh-derived tensor
  understates inertia because the geometry is too small, so with mass held fixed
  and a real assembly density assumed, `V_true = m/rho`, `s = (V_true/V_mesh)^(1/3)`,
  `I_true = I_mesh * s^2` — the `AGENTS.md` "compute from the mesh and scale it"
  rule anchored on density rather than a known moment. At rho = 2500 kg/m^3 that
  is x7.18 (R) and x7.38 (L). It assumes the real part is a uniform scale-up of
  the stub. `spread` stays 1.00, so only magnitude changes, not shape.
- **Humotech weld groups collapsed onto their primary bodies.** `shin_cuff`,
  `heel_connector` and `sole_attachment` carry geometry at `mass="0"`, with the
  group total on `shin_main` / `heel_ring`. Each CAD assembly maps onto exactly
  one weld group (`shin_*` → `tibia`, `heel_*`/`sole_attachment` → `calcn`), and
  welded bodies sum into the parent's composite inertia, so this reproduces the
  CAD composite exactly. The zeros are exact, not placeholders.

- **Split shoe re-tared to 330 g/side** (was 1.79232 kg) in `STRIDE_L2` and
  `Anatomics_L1`, which share the geometry: 60 % to the soles and 40 % to the
  uppers, split evenly fore/aft — `aft_sole` and `fore_sole` 99 g each,
  `aft_upper` and `fore_upper` 66 g each. The old distribution was heel-biased
  (`aft_sole` alone was 924.5 g, over half the per-side total).
- **`STRIDE_L2` exo re-tared to 770 g/side** (was 849.65 g): the `shank`
  segment carrying the actuator is 590 g, and the 180 g distal budget — four
  linkage bodies plus the carbon plate, which is folded into `foot` — keeps its
  CAD-derived relative distribution (every sub-shank mass × 180/127.44).
- **`STRIDE_L2` per-leg total is now 1.100 kg**, down from 2.642 kg.
- **`Humotech_L1` shoe** re-tared to the same 330 g/side, so all three models
  sharing this mesh set now agree.

### Known-outstanding

> The shoe `diaginertia` values in all three models are still the original
> placeholders scaled by mass. Their implied densities disagree by 7.8x for what
> is one material (`aft_sole` 98, `fore_sole` 186, `aft_upper` 393,
> `fore_upper` 766 kg/m^3), and `mt-check-inertia` puts them at 0.74-0.76x the
> geometry. Masses are right; distributions are not. They want the same
> mesh-based recompute Hippo received.

> `OpenExo_L1`'s `exo_shaft` sits at ~48000-50000 kg/m^3 against its own mesh.
> That number is expected and is the honest record of a stub carrying both the
> actuator and unmodelled off-board mass; the envelope correction above is what
> makes the *inertia* approximately right despite it. Note `mt-check-inertia`
> reports 0 flagged here — a tensor computed from a mesh is self-consistent with
> that mesh however wrong the mesh is, so `rho` is the only column that reveals
> the problem. Supplied meshes are as-received from the OpenExo project.

> `mt-check-inertia` flags `heel_ring_r`/`_l` as REACH: radius of gyration
> 0.1138 m against a mesh reach of 0.0934 m. This is correct behaviour and the
> tensor is right — the Humotech foot meshes cover only 5.7% of the real CAD
> assembly (75.3 cm^3 against 1309.97 cm^3), so the geometry on disk cannot
> corroborate an inertia measured on the real hardware. Resolving the flag needs
> better meshes, not different numbers.

## [0.6.0] — NEU environments (NEUankle, STRIDE) + four new config sections

Adds two Northeastern device environments and the config surface they needed.
Every existing device's `(nq, nu, nbody, nmesh)` signature is unchanged.

### Added

- **`NEUankle_L1`** — powered right transtibial prosthesis: socket → pyramid
  adapter → actively driven ankle (`neuankle_ankle_angle_r`, 50 Nm joint torque)
  → foot plate → cosmetic shell. Same biological scope as `KFoot_L1` / `OSL_A`.
  Every inertial computed from its own mesh by signed-tetrahedron integration.
- **`STRIDE_L2`** — bilateral cable-driven ankle exoskeleton, and the first
  **Level 2** lower-limb device: a real closed chain (Watt six-bar behind each
  ankle, five hinges per side, one net DOF) closed by `equality: joint` quartics
  rather than by the body tree. Bundles the split shoe; 400 N Bowden cables
  (`cable_r`/`_l`); intra-device contact filtered by `contype=2` rather than
  ~30 excludes per side. Verified: hinge exactly coaxial with the biological
  ankle (axis dot −1.000000), plate-to-sole relative rotation constant to
  ±0.002° across the ROM, couplings settling to single-digit microradians, and a
  cable moment arm of 72–117 mm/rad giving 29–47 Nm of plantarflexion at 400 N.
- **`body_overrides`** — override a body's `mass` / `diaginertia` / `fullinertia`
  / `ipos` / `iquat`. The mass-side counterpart to `mesh_replacements`; raises
  rather than silently zeroing a compiler-derived inertia.
- **`equality: type: joint`** — couple two scalar joints by a quartic
  (`joint1`, `joint2`, `polycoef`), for closing a kinematic loop and for tying a
  linkage to the joint it spans. Each name resolves bare-first then prefixed, so
  one entry can couple two device joints or a device joint to an MSK joint.
- **`contact`** — `pairs` and `excludes` on the combined model. Device-XML
  `<contact>` sections do not migrate through `attach_body`.
- **`sensors` / `sensor_removals`** — add and remove sensors; removal plus
  re-addition under the same name is how a sensor gets *re-pointed*.
- Per-MSK `default:` + `<msk_key>:` dispatch for `joint_overrides`, alongside all
  four new sections.

### Fixed

- **`forcerange` / `forcelimited` were dropped** when importing a device's
  tendon-driven actuators, silently discarding an authored force limit. This
  matters because `forcerange` clamps in actuator space, *before* `gear`: a cable
  motor with `gear="-1"` and `forcerange="-400 0"` produces exactly zero force.
- **`joint_overrides` `axis` and `pos` were parsed but never applied** — dead
  config fields that accepted a value and did nothing.
- **Transtibial devices carried the intact shank's mass.** `KFoot_L1`,
  `OpenSourceLeg_A_L1` and `NEUankle_L1` now reduce `tibia_r` to the residuum
  (1.85375 kg), removing ~1.85 kg of phantom mass from the prosthetic side.
- **`tibia2_col_r` reached past the amputation.** On `myolegs`/`myofullbody` that
  collision capsule runs y = −0.14 to −0.35, i.e. 0.13 m beyond the y = −0.219
  amputation and straight through the socket and pylon, and genuinely collided
  with the device. Removed for the transtibial devices.
- **Prosthetic-side sensors were gone with no way back.** Deleting `talus_r`
  cascades away `r_foot`, `r_toes`, `r_ankle_sensor` and `r_mtp_sensor`, dropping
  the baseline's twelve sensors to eight and leaving nothing reading the
  prosthetic side while the intact side kept all four counterparts. Restored for
  all three transtibial devices (minus `r_mtp_sensor`, which has no joint to
  sense on a single-plate foot).
- **K-Foot's five device inertials** were hand-entered and overstated 1.5×–2.5×,
  three of them isotropic placeholders (one with an unnormalized quat that
  resolved to identity). Recomputed from the meshes.
- README's combination matrix was stale: `myolegs22` is buildable, `myofullbody`
  was missing entirely, and `HMEDI_L1` was marked n/a on `myolegs26` despite
  being covered by the smoke tests.

### Known issues

- `Humotech_L1` and `Anatomics_L1` still read foot contact through the baseline
  touch sites, which sit above a shoe sole's contact surface — the same gap
  `STRIDE_L2` fixes with `sensor_removals`. Not addressed here.
- `myolegs22`'s keyframes have the right leg's `ankle_angle_r` / `mtp_angle_r`
  values shifted by one slot (upstream in `myo_sim` / `reduce_legs`, not here);
  `squat` puts 0.349 rad on the right MTP. The other three MSKs ship no
  keyframes, so only `myolegs22` is affected.
- `STRIDE_L2` on `myolegs` / `myofullbody`: those models have a `subtalar_angle`
  DOF the 26/22-muscle ones lack. A single-plane linkage cannot follow
  inversion/eversion and nothing couples the plate to subtalar, so under subtalar
  motion the plate and sole separate with no force resisting it.

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
