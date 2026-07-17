# Concepts

This doc covers the architecture: why `assist_sim` exists, the in-memory
pipeline, and how it fits with `myo_sim` and downstream training frameworks.

## The three-repo split

`assist_sim` sits between an upstream MSK source and downstream training
frameworks. Four packages collaborate:

- **`myo_sim`** provides the baseline MSK models and their meshes. On the
  `mm_refactor` branch these leg models are *composed at runtime* (no static
  XML), so `assist_sim` obtains an editable `MjSpec` via
  `myo_sim.build_spec(<model>)`, serializes it, strips the bundled myosuite
  scene, and caches the model-only XML. assist_sim's MSK keys mirror the
  myo_sim model names. `myolegs26` (26-muscle, passive torso) and `myolegs`
  (80-muscle, passive torso) are wired; `myolegs22` follows when the 26→22
  reduction lands.
- **`assist_sim`** (this repo) holds the *combination pipeline* and *device
  configurations*. It produces compiled `MjModel` objects (and optional
  exported XMLs) where an MSK and a device are combined into one runnable
  model.
- **`myoassist`** consumes those combined models as the simulation backbone
  for control optimization, reinforcement learning, etc. It wraps the
  model with a scene (terrain, lighting, sensors specific to the training
  task) and the policy training loop.
- **`myoassist.terrains`** is a separate package that owns the *scene*
  layer (ground plane, hfields, skybox). `assist_sim` outputs are
  *model-only* -- no ground body, no terrain include -- and
  `myoassist.terrains` layers the scene on top before simulation.

> *Diagram placeholder -- figure of package flow
> will replace this in a later docs pass.* 
<!-- # TODO: #1 -->

## The in-memory pipeline

The inputs are an MSK (a registry key that `myo_sim` composes on demand, or an
explicit baseline XML path) and a device YAML; the outputs are an
`(MjModel, MjData)` pair ready to step and (optionally) an exported combined
XML.  Everything runs on a single live `MjSpec` -- the human model is never
serialized to XML and reloaded (see [Why in-memory](#why-in-memory)).

**Resolve.**  `registry._resolve_msk` calls `myo_sim.build_spec(<model>)` and
strips the bundled myosuite scene (worldbody-direct floor / backdrop / pedestal
/ logo geoms, scene lights and cameras, plus the meshes only they referenced),
returning a model-only human `MjSpec`.  (The explicit-path entry point instead
loads the spec from the given XML.)

**Combine** (`combine.py`).  Operates on that spec, requiring `mujoco>=3.3.4`
for `MjSpec.delete`:

- Decompose each source keyframe's qpos/qvel into per-joint slices by *name*
  (a pre-surgery compile), then blank the keyframe arrays so surgery can change
  the layout.
- Surgery -- apply body / geom / actuator / tendon removals via `spec.delete`.
  Deleting a body cascades its subtree plus the sensors, equality constraints,
  and actuators/tendons whose sites lived on it; contact `<pair>`s referencing a
  removed geom are scrubbed manually (delete does not cascade those).
- Attach each device body under its parent body, applying the device-name
  prefix to all imported elements; honor per-MSK attachment overrides.
- Apply joint range / damping overrides; add YAML-declared joint actuators;
  import device-side spatial tendons + tendon-transmission actuators.
- Compile, rebuild keyframes by joint name (restore surviving joints' authored
  values, apply `keyframe_overrides`), and recompile to lock in the keyframes.

The compiled `MjModel` and a fresh `MjData` are returned.

### Why in-memory

MuJoCo's `MjSpec.delete` (3.3.4+) lets removals happen on the live spec, so
there is no separate ElementTree removal phase.  This is also *required* for
`myo_sim`'s torso-composed models: their serialized `to_xml` output does not
round-trip (the merged fragment default trees produce a nested unnamed
`<default>`, which MuJoCo rejects on reload as an "empty class name").  Working
the spec in memory sidesteps serialization entirely.  Device models are still
static XML files that round-trip fine, so `preprocess.prepare_device_xml` does
a little text-level massaging of the device side before attach.

## Naming conventions

### Registry keys

- **MSK keys**: `myolegs22`, `myolegs26`, `myolegs`. Curated list in
  `assist_sim/registry.py:_COMPATIBLE_MSK_KEYS`. Each binds to a
  `myo_sim.build_spec` model name and a minimum MuJoCo version; the model is
  composed on demand and cached as a model-only XML. Keys with no source yet
  (`myolegs22`) or that need a newer MuJoCo (`myolegs`) raise a clear error
  when resolved.
- **Device keys**: derived from `models/<DeviceDir>/<variant>config.yaml`.
  Example: `models/DephyExoBoot/L1config.yaml` → `DephyExoBoot_L1`,
  `models/OpenSourceLeg/A_L1config.yaml` → `OpenSourceLeg_A_L1`. The
  device's `device.name` field is also registered as an alias.

### Namespace prefix

When a device attaches to an MSK, the `device.name` is used as a prefix on
every body, site, mesh, joint, actuator, and tendon imported from the device
XML. Example: `DephyExoBootL1_exo_1_r`, `OSL_KA_L1_osl_ankle_angle_r`. This
prevents collisions with names in the MSK and makes the device's contribution
identifiable in the compiled model.

## Per-MSK configuration overrides

A single device YAML can carry per-MSK variations for any of these sections:
`attachments`, `tendon_modifications`, `keyframe_overrides`,
`actuator_removals`, `tendon_removals`, `mesh_replacements`. The schema
shape:

```yaml
tendon_modifications:
  default:
    - name: gastroc_r_tendon
      wraps: ...
  myolegs:
    - name: gasmed_r_tendon       # 80-muscle equivalent
      wraps: ...
```

The resolver picks the matching MSK key if present, else `default`. See
[device-config-reference.md](device-config-reference.md) for which sections
support per-MSK overrides.

## What `assist_sim` does *not* do

- **Provide MSK models** -- those live in `myo_sim`.
- **Provide terrain / scene** -- that's `myoassist.terrains`.
- **Train policies** -- that's `myoassist`.
- **Simulate** -- `assist_sim` produces models; you simulate them with
  MuJoCo as you would any model.
- **Provide a viewer** -- `examples/quickstart.py` opens `mujoco.viewer` for
  inspection, but the package itself has no viewer logic.

This narrow scope is intentional: `assist_sim` is the *model composition*
layer. Everything else is upstream or downstream.
