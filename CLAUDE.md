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
pytest                              # 231 pass, 1 skip with myo_sim installed (~5 min)
ruff check . && ruff format --check .

python -m assist_sim list           # discoverable combinations (also: validate, combine)
python -m assist_sim msk myolegs26 -o out.xml   # baseline MSK, no device
python examples/quickstart.py myolegs26 DephyExoBoot_L1   # visual inspection
```

> Note: a migration to uv is planned, for tool parity with `myo_sim`, which uses `uv sync` and
> `uv run pytest -n auto`. Until then, assist_sim uses pip.

## Architecture

The pipeline has a single phase and runs fully in memory. It requires `mujoco>=3.3.4` for
`MjSpec.delete`.

1. **Resolve**: `registry._resolve_msk` calls `myo_sim.build_spec(<model>)`. It then removes the
   bundled myosuite scene with `utils.strip_myosuite_scene_spec` and returns a live human `MjSpec`.
   The pipeline never serializes the composed model, because torso-composed models do not
   round-trip through `to_xml`. Every later step therefore edits the spec directly.
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

`attach_body` copies a body subtree and the assets that the subtree references. **It leaves every
top-level MJCF (MuJoCo XML format) section behind.** The code reads the tendons and the
tendon-driven actuators back out of the device XML. You author `<equality>`, `<contact>` and
`<sensor>` in YAML against the *combined* model. In the combined model, the device names carry the
prefix and the MSK names stay bare.

Names in those sections resolve **bare first, then prefixed**. A device element can use the same
name as an MSK element that the surgery keeps. In that case, the MSK element shadows the device
element, and the pipeline gives no message. See `r_sole_touch` in `STRIDE_L2`.

`registry.py` resolves the MSK keys that `myo_sim` composes. It also finds the device configs
automatically when it scans `assist_sim/models/*/L1config.yaml`. `config.py` holds the
`DeviceConfig` dataclass and the per-MSK resolvers. `preprocess.py` now holds only the device XML
preparation (`prepare_device_xml`) and the `KeyframeData` container. Static device XML files still
round-trip correctly.

## Conventions (mandatory)

- **Use errors, not warnings.** An unresolved YAML reference raises `ValueError` with a "did you
  mean" suggestion. Never call `warnings.warn`.
- **In-memory surgery.** This needs `mujoco>=3.3.4` for `MjSpec.delete`. The removals run on the
  live human `MjSpec`. `spec.delete` cascades to subtrees, sensors, actuators and tendons, but NOT
  to contact `<pair>` elements. The code removes the contact `<pair>` elements manually. The code
  still edits the device XML files at the text level.
- **Re-anchor before you remove.** `tendon_modifications` has four operations (`reposition_site`,
  `replace_site`, `reposition_geom`, `replace_geom`). `drop_site` is retired and raises an error.
  Wrap cylinders count in the same way as sites. If one geom stays on a body that you remove, the
  cascade removes the tendon, whatever number of sites you moved. A re-anchor changes the path of a
  muscle, and the compiler keeps the authored lengthranges. Therefore always set it with
  `actuator_overrides: [{name, lengthrange}]`.
- **Exports are model-only, but not scene-free.** `_strip_scene_visual` removes the MSK headlight.
  Then `_ensure_minimal_visual` adds a soft headlight and a neutral gradient skybox to *every*
  export, both combined and `load_msk`, so that a bare file renders. `utils._strip_terrain` is dead
  code, because no caller passes `terrain_paths`. The terrain goes away at resolve time with the
  myosuite scene.
- **Keep the public surface minimal.** Only the names that `assist_sim/__init__.py` exports are
  committed API. Internal names keep an underscore prefix.
- **The registry finds devices automatically.** Add a folder under `assist_sim/models/`. You do not
  have to change any code.
- **Increase `__version__`** in `assist_sim/__init__.py` for every pipeline change. The version is
  part of the cache key, so an increase invalidates the old exports. Also update the smoke tuples
  in `tests/test_smoke_combinations.py` if an `(nq, nu, nbody, nmesh)` signature changes.

## myo_sim integration status

The baseline MSK models live in `myo_sim`, not in this repository. `assist_sim` resolves them
through `_COMPATIBLE_MSK_KEYS` in `registry.py`. On the `dev` branch of myo_sim, the code
**composes the leg models at runtime**, with no static XML. The assist_sim keys mirror the myo_sim
model names.

`_resolve_msk` calls `myo_sim.build_spec(<model>)`, removes the bundled myosuite scene, and returns
a live `MjSpec`. `combine.py` then changes that spec in place. All four keys build on
`mujoco>=3.3.4`; `myolegs26` also builds on 3.3.3:

| Key | nq | nu | Keyframes | Notes |
|---|--:|--:|--:|---|
| `myolegs22`   |  39 |  22 | 5 | planar 26→22 reduction of `myolegs26` (`reduce_legs.py`), applied after the build |
| `myolegs26`   |  47 |  26 | 0 | 26-muscle, passive torso + legs |
| `myolegs`     |  35 |  80 | 0 | 80-muscle, passive torso |
| `myofullbody` | 129 | 416 | 0 | full body (torso muscles + arms + legs) |

Only `myolegs22` supplies keyframes (`stand`, `walk_left`, `walk_right`, `squat`, `lunge`).
Therefore `keyframe_overrides` does nothing on the other three keys, and it gives no message.
Verify these numbers against `assist_sim.registry` before you quote them.

Torso-composed models are the reason for the in-memory pipeline. These models are `myolegs` and
every device that needs a torso, such as HMEDI. Their serialized `to_xml` output does not
round-trip: a nested unnamed `<default>` gives "empty class name" at reload. Therefore assist_sim
never serializes the human model.

## More detail

Read `CONTRIBUTING.md` for the setup, the steps to add a device or an MSK model, the style rules
and the pull request rules. Read `docs/` for the architecture, the usage, the device config
reference, the troubleshooting page and the how-to guides.
