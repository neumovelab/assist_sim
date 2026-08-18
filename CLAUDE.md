# CLAUDE.md

`assist_sim` is the middle layer between [`myo_sim`](https://github.com/MyoHub/myo_sim) and
downstream training such as `myoassist`. `myo_sim` supplies the baseline musculoskeletal (MSK)
models. `assist_sim` takes a baseline MSK model and an assistive device that YAML describes. It
applies the prosthetic surgery, attaches the device, and returns a compiled `MjModel`. `assist_sim`
keeps the baseline MSK model on disk unchanged.

Read `CONTRIBUTING.md` and `docs/concepts.md` before any substantial change.

## Quickstart (pip)

```bash
pip install -e .
pip install -r requirements-dev.txt
pytest                              # 277 pass, 1 skip with myo_sim installed (~3 min)
ruff check . && ruff format --check .

python -m assist_sim list           # discoverable combinations (also: validate, combine)
python -m assist_sim msk myolegs26 -o out.xml   # baseline MSK, no device
python examples/quickstart.py myolegs26 DephyExoBoot_L1   # visual inspection
```

> Note: a migration to uv is planned, for tool parity with `myo_sim`, which uses `uv sync` and
> `uv run pytest -n auto`. Until then, assist_sim uses pip.

Supported Python is 3.10 to 3.13. The MuJoCo range is `>=3.4,<3.12`; 3.4 through 3.11 are
verified, and the `mujoco-range` CI job runs the suite at both ends. Do not widen the ceiling
without that job passing on the newer release: MuJoCo keeps widening scalar `MjSpec` fields and
moving `MjData` arrays, and an unbounded range once shipped a wheel that could not combine a
device on any MuJoCo past 3.6.

## Architecture

The pipeline has a single phase and runs fully in memory.

1. **Resolve**: `registry._resolve_msk` calls `myo_sim.load_spec(<model>)`. It then removes the
   bundled myosuite scene with `utils.strip_myosuite_scene_spec` and returns a live human `MjSpec`.
   The *pipeline* never serializes mid-flow, because torso-composed models do not round-trip
   through a bare `to_xml`. Every later step therefore edits the spec directly. Export is a
   separate matter: `utils.export_combined_xml` applies the default hoist/name fixups that make
   those models reload, and all four MSK keys round-trip through it.
2. **Re-anchor**: `combine._apply_tendon_modifications` moves the wrap sites and the wrap geoms
   onto the residual bone, for each muscle that an amputation keeps. This step runs **before** the
   removals. After the removals, the `spec.delete` cascade removes the tendon, and no element is
   left to re-anchor.
3. **Combine** (`combine.py`, MjSpec): the surgery runs through `spec.delete` for the body, geom,
   actuator, tendon and sensor removals. `spec.delete` cascades to subtrees and to the elements
   that reference them, but the code removes contact `<pair>` elements manually. The step then
   attaches the device bodies and edits attributes (mesh swaps, inertial, actuator and joint
   overrides). It adds the actuators, imports the tendons of the device, and emits the equalities,
   contacts and sensors. Last, it rebuilds the keyframes. The pipeline decomposes the keyframes by
   joint name before the surgery and restores them after the final compile.

`attach_body` copies a body subtree, and the first attach copies the device's whole asset library.
**It leaves every top-level MJCF (MuJoCo XML format) section behind.** The code reads the tendons
and the tendon-driven actuators back out of the device XML: every tendon kind (`<spatial>` and
`<fixed>`) and every wrap kind (`site`, `geom`, `pulley`, `joint`), in document order, because wrap
order *is* the routing. You author `<equality>`, `<contact>` and `<sensor>` in YAML against the
*combined* model. In the combined model, the device names carry the prefix and the MSK names stay
bare.

Names in those sections resolve **bare first, then prefixed**. A device element can use the same
name as an MSK element that the surgery keeps. In that case, the MSK element shadows the device
element, and the pipeline gives no message. See `r_sole_touch` in `STRIDE_L2`.

Never compile a spec you are about to keep editing. Compiling and then editing corrupts a later
add/delete pair: measured on MuJoCo 3.4 and 3.11, "add three joints then delete four others" loses
one added joint and leaves one delete target in place, because a delete resolves to the wrong
element. The counts still come out right, so the result compiles and looks correct while being a
different model. `combine._decompose_keyframes` and `reduce_legs` both probe a `spec.copy()` for
this reason, and `tests/test_spec_edit_safety.py` fails if either copy is removed.

### Module map

| Module | Holds |
|---|---|
| `combine.py` | the pipeline: surgery, attach, actuators, tendons, equalities, contacts, sensors, keyframes |
| `registry.py` | MSK key resolution + device autodiscovery; `resolve_device_config` is the compose-free half |
| `config.py` | the `DeviceConfig` dataclass, the strict YAML loader, and the per-MSK resolvers |
| `loading.py` | `load_combined` / `load_msk` / `resolve_model_path`, and the cache key tokens |
| `cache.py` | the opt-in on-disk cache (keys, atomic publish, corrupt-entry recovery) |
| `preprocess.py` | device XML preparation (`prepare_device_xml`) and the `KeyframeData` container |
| `reduce_legs.py` | the in-spec 26 -> 22 planar reduction that produces `myolegs22` |
| `canonical_keyframes.py` | the fallback pose table for an MSK that ships no keyframes |
| `root_frame.py` | `to_planar_root`, the CO-only freejoint -> named-pelvis-DOF reframe |
| `upper_body.py` | the four composed upper-body envs (wheelchair, MPL, Auxivo liftsuit, bionic bimanual) |
| `utils.py` | XML export, myosuite-scene strip, default hoisting, mesh dedup and path rewriting |
| `validate.py` | the standalone, test-only config validator |

## Conventions (mandatory)

- **Use errors, not warnings.** An unresolved YAML reference raises `ValueError` with a "did you
  mean" suggestion. Never call `warnings.warn`. The config loader also rejects what it cannot use:
  an unknown section, an unknown key inside an entry, a per-MSK block keyed by a name that is not
  an MSK key, an unimplemented actuator `type`. A silent no-op is a bug, not a convenience.
- **In-memory surgery.** The removals run on the live human `MjSpec`. `spec.delete` cascades to
  subtrees, sensors, actuators and tendons, but NOT to contact `<pair>` elements. The code removes
  the contact `<pair>` elements manually. The code still edits the device XML files at the text
  level.
- **Re-anchor before you remove.** `tendon_modifications` has four operations (`reposition_site`,
  `replace_site`, `reposition_geom`, `replace_geom`). `drop_site` is retired and raises an error.
  Wrap cylinders count in the same way as sites. If one geom stays on a body that you remove, the
  cascade removes the tendon, whatever number of sites you moved. A re-anchor changes the path of a
  muscle, and the compiler keeps the authored lengthranges. Therefore always set it with
  `actuator_overrides: [{name, lengthrange}]`.
- **Exports are model-only, but not scene-free.** `_strip_scene_visual` removes the MSK headlight.
  Then `_ensure_minimal_visual` adds a soft headlight and a neutral gradient skybox to *every*
  export, both combined and `load_msk`, so that a bare file renders. `utils._strip_terrain` has no
  caller in the package (nothing passes `terrain_paths`) but `tests/test_terrain_strip.py` covers
  it, so it is unused rather than dead. The terrain goes away at resolve time with the myosuite
  scene.
- **Keep the public surface minimal.** Only the names that `assist_sim/__init__.py` exports are
  committed API. Internal names keep an underscore prefix.
- **The registry finds devices automatically.** Add a folder under `assist_sim/models/`. You do not
  have to change any code.
- **Bump `version` in `pyproject.toml`** for every pipeline change, and add a `CHANGELOG.md` entry.
  `assist_sim.__version__` is read from installed package metadata, so it is *not* edited in
  `assist_sim/__init__.py`, and on an editable install it does not move until you reinstall. The
  cache key folds in the newest source mtime as well as the version, so local edits invalidate
  cached exports even without a bump. Also update the smoke tuples in
  `tests/test_smoke_combinations.py` if an `(nq, nu, nbody, nmesh)` signature changes.
- **A contact `<exclude>` does not cancel a `<pair>`.** They act at different stages: a predefined
  pair is always evaluated, an exclude only filters what the broadphase generates. Measured on 3.4
  and 3.11. Earlier versions of these docs claimed the opposite.

## Caching

Caching is **opt-in and off by default**, and it matters more downstream than here. Pass
`cache_dir=` to `load_combined`, `load_msk` or `load_combined_model`. A hit skips the compose as
well as the combine, so it is a real hit — the compose used to run *before* the lookup, which made
a hit cost the same as a miss.

Whether it pays depends on the model, because reload cost is dominated by XML text parsing:

| MSK | miss | hit | |
|---|--:|--:|---|
| `myolegs22` | 0.64 s | 0.08 s | 8.2x |
| `myolegs26` | 0.52 s | 0.19 s | 2.8x |
| `myolegs` | 0.59 s | 0.27 s | 2.2x |
| `myofullbody` | 0.99 s | 1.11 s | **0.9x — do not cache** |

`myofullbody` exports 0.6 MB of MJCF (418 actuators, 108 meshes) and parsing that costs more than
composing it from scratch.

The key folds in a per-package token (release version **plus** newest source mtime) for both
assist_sim and myo_sim, so an editable-install edit to either invalidates. Entries are published
with `os.replace` under a per-writer name, because the case this exists for is N processes racing
a cold cache. An unreadable entry is treated as a miss and rebuilt.

**Downstream this is not optional in practice.** `myoassist` composes a model per CMA candidate and
per `SubprocVecEnv` worker, so a controller-optimization run at the shipped
`--popsize 32 --maxiter 1000` composes on the order of 32,000 times. Uncached, the composed
architecture costs 13-15x more per env than the static model files MyoAssist 0.1 shipped; with
`MYOASSIST_CACHE_DIR` set it is back to parity (0.045 s against 0.037 s for `myolegs22`). Anyone
running training without it is paying that multiple on every model build.

## myo_sim integration status

The baseline MSK models live in `myo_sim`, not in this repository. `assist_sim` resolves them
through `_COMPATIBLE_MSK_KEYS` in `registry.py`. myo_sim **composes the leg models at runtime**,
with no static XML. The assist_sim keys mirror the myo_sim model names, and `myo-sim>=0.2.1` is a
hard runtime dependency, so `pip install -e .` brings it in.

`_resolve_msk` calls `myo_sim.load_spec(<model>)`, removes the bundled myosuite scene, and returns
a live `MjSpec`. `combine.py` then changes that spec in place. Note the name: the published myo_sim
exposes `load_spec`, **not** `build_spec` — an older name these docs used for a while.

| Key | nq | nu | Base keyframes | Notes |
|---|--:|--:|--:|---|
| `myolegs22`   |  39 |  22 | 5 | planar 26→22 reduction of `myolegs26` (`reduce_legs.py`), applied after the build |
| `myolegs26`   |  47 |  26 | 0 | 26-muscle, passive torso + legs |
| `myolegs`     |  35 |  80 | 0 | 80-muscle, passive torso |
| `myofullbody` | 129 | 416 | 0 | full body (torso muscles + arms + legs) |

Only `myolegs22` *ships* keyframes. The other three get the canonical `stand` / `walk_left` /
`walk_right` / `squat` / `lunge` poses injected at combine time (`canonical_keyframes.py`), so
**every leg combination compiles with `nkey=5`** and `keyframe_overrides` applies to all four keys.
A downstream consumer that seats or poses from a named keyframe depends on this; myoassist asserts
`nkey > 0`. Verify these numbers against `assist_sim.registry` before you quote them.

Two things differ per lineage and bite the poses:

- **Knee sign.** `myolegs22` and `myolegs26` flex negative (`[-2.53, 0]`); `myolegs` and
  `myofullbody` flex positive (`[0, +2.09]`). The canonical table is authored myoLeg-negative and
  flipped for a positive-flexion host. The probe scans *every* knee, because a transfemoral
  amputation deletes the operated side's.
- **Root joints.** `myolegs22` roots on named planar joints; the others float on a `freejoint`, so
  a `pelvis_ty` override has nothing to land on unless `planar_root=True` re-frames them.

`planar_root` is a CO-only flag on `ModelCombiner.combine` / `load_combined` (`root_frame.py`). It
yaws a freejoint 3D-lineage leg MSK into the `myolegs22` frame and swaps the freejoint for the six
named pelvis DOF joints, making it a structural and frame drop-in for the reflex controller. It is
a no-op on `myolegs22`, and RL leaves it off to keep the floating base.

Per-MSK `keyframe_overrides` **merge** onto `default` joint by joint; every other per-MSK section
replaces. That is deliberate — the section is a patch, and a lineage usually needs one joint of one
pose changed.

Torso-composed models are the reason for the in-memory pipeline. These models are `myolegs` and
every device that needs a torso, such as HMEDI. Their bare `to_xml` output does not round-trip: a
nested unnamed `<default>` gives "empty class name" at reload. `export_combined_xml` fixes that up,
which is why exports work while mid-pipeline serialization is still avoided.

## More detail

Read `CONTRIBUTING.md` for the setup, the steps to add a device or an MSK model, the style rules
and the pull request rules. Read `docs/` for the architecture, the usage, the device config
reference, the troubleshooting page and the how-to guides.
