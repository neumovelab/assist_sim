# Concepts

This document gives the architecture. It tells you why `assist_sim` exists, how
the pipeline operates in memory, and how `assist_sim` operates with `myo_sim`
and with the downstream training frameworks.

## The three-repo split

`assist_sim` is between an upstream musculoskeletal (MSK) source and the
downstream training frameworks. Four packages operate together:

- **`myo_sim`** supplies the baseline MSK models and their meshes. On the `dev`
  branch, `myo_sim` *composes* these leg models at run time, with no static
  XML. `assist_sim` gets an editable `MjSpec` with
  `myo_sim.load_spec(<model>)`, then removes the bundled myosuite scene from
  that live spec. The pipeline serializes nothing between the stages, and the
  MSK keys of `assist_sim` are the same as the `myo_sim` model names. The
  package supports `myolegs26` (26 muscles, passive torso), `myolegs` (80
  muscles, passive torso) and `myofullbody`. It derives `myolegs22` (planar, 22
  muscles) from `myolegs26` with a 26→22 reduction in the spec.
- **`assist_sim`** (this repository) holds the *combination pipeline* and the
  *device configurations*. It produces compiled `MjModel` objects that combine
  an MSK model and a device into one model that you can run. It can also export
  those models as XML files.
- **`myoassist`** uses those combined models as the simulation base for control
  optimization, reinforcement learning and related tasks. It adds a scene
  (terrain, lights, and the sensors for the training task) and the training
  loop for the policy.
- **`myoassist.terrains`** is a separate package for the *scene* layer (ground
  plane, hfields, lights). The outputs of `assist_sim` contain no terrain: no
  ground body, no hfield, no terrain include. They contain a minimal visual,
  that is a soft headlight and a neutral gradient skybox. Thus an export
  renders correctly on its own. If a downstream scene adds its own headlight
  and skybox, those take precedence.

> *Diagram placeholder. A figure of the package flow will replace this text in
> a later documentation pass.*
<!-- # TODO: #1 -->

## The in-memory pipeline

The inputs are an MSK model and a device YAML file. You give the MSK model as a
registry key that `myo_sim` composes on demand, or as a path to an explicit
baseline XML file. The outputs are an `(MjModel, MjData)` pair that you can
step, and an optional export of the combined XML. All steps operate on one live
`MjSpec`. The pipeline does not serialize the human model to XML and reload it.
See [Why in-memory](#why-in-memory).

**Resolve.**  `registry._resolve_msk` calls `myo_sim.load_spec(<model>)`. It
then removes the bundled myosuite scene from the live spec. That scene contains
the floor, backdrop, pedestal and logo geoms directly under worldbody, the
scene lights and cameras, and the meshes that only these elements use. The
`MjSpec` with no scene goes directly to `combine.py`. The pipeline writes
nothing to disk between the two stages. The entry point that takes an explicit
path loads the spec from the given XML file instead.

**Combine** (`combine.py`).  This stage operates on that spec. It needs
`mujoco>=3.4,<3.12`, the declared range, for `MjSpec.delete`. The steps are:

- Decompose the qpos and qvel of each source keyframe into slices per joint, by
  *name*. This step needs a compile before the removals. Then clear the
  keyframe arrays, so that the removals can change the layout.
- Re-anchor. Apply `tendon_modifications`, which move the wrap sites and the
  wrap geoms of a kept muscle onto the bone that remains. This step runs
  *before* the removals. After the removals, the cascade already removed the
  tendon.
- Remove. Apply the body, geom, actuator, tendon and sensor removals with
  `spec.delete`. When you remove a body, the cascade also removes its subtree,
  the sensors, the equality constraints, and the actuators and tendons that
  referred to it. The pipeline removes a contact `<pair>` that refers to a
  removed geom separately, because `spec.delete` does not cascade to those.
- Attach each device body under its parent body. The pipeline applies the
  device-name prefix to all imported elements, and it obeys the per-MSK
  attachment overrides.
- Apply the mesh replacements, the body inertial overrides, the actuator
  overrides (`lengthrange`), and the joint range and damping overrides. Add the
  joint actuators from the YAML file. Import the spatial tendons and the
  tendon-transmission actuators of the device. Add the equalities, contacts and
  sensors that `attach_body` does not move.
- Compile the spec. Rebuild the keyframes by joint name: restore the authored
  values of the joints that remain, then apply `keyframe_overrides`. Compile
  again, to make the keyframes permanent.

The pipeline returns the compiled `MjModel` and a new `MjData`. If you request
an export, `utils.export_combined_xml` writes it from the final spec. This is
the only step that makes XML from a combined model. The optional `cache_dir`
cache in `cache.py` stores those *exports*. That cache has no relation to the
resolve stage, which caches nothing.

### Why in-memory

`MjSpec.delete` removes elements from the live spec (it arrived in MuJoCo 3.3.4,
well below the 3.4 floor the package requires),
so the pipeline needs no separate removal phase with ElementTree. The `myo_sim`
models that compose a torso also make this necessary. Their serialized `to_xml`
output does not round-trip: the merged default trees of the fragments give a
nested unnamed `<default>`, and MuJoCo rejects it on reload with an "empty
class name" error. When the pipeline operates on the spec in memory, it avoids
serialization completely. Device models are still static XML files that
round-trip correctly, so `preprocess.prepare_device_xml` makes small text
changes to the device side before the attachment.

## Naming conventions

### Registry keys

- **MSK keys**: `myolegs22`, `myolegs26`, `myolegs`, `myofullbody`.
  `assist_sim/registry.py:_COMPATIBLE_MSK_KEYS` holds the selected list. Each
  key refers to a `myo_sim.load_spec` model name and to a minimum MuJoCo
  version. `myo_sim` composes the model on demand and gives it as a live spec,
  never as a cached XML file. A key with no `myo_sim` source raises a clear
  error when you resolve it. A key that needs a more recent MuJoCo does the
  same. Today all four keys sit at the package floor, so that gate cannot trip on a
  resolvable install; it is kept for a future MSK needing a *newer* MuJoCo than the floor.
- **Device keys**: the registry derives these from
  `models/<DeviceDir>/<variant>config.yaml`. For example,
  `models/DephyExoBoot/L1config.yaml` gives `DephyExoBoot_L1`, and
  `models/OpenSourceLeg/A_L1config.yaml` gives `OpenSourceLeg_A_L1`. The
  registry also adds the `device.name` field of the device as an alias.

### Namespace prefix

When a device attaches to an MSK model, the pipeline uses `device.name` as a
prefix. It applies that prefix to every body, site, mesh, joint, actuator and
tendon that it imports from the device XML file. Two examples are
`DephyExoBootL1_exo_1_r` and `OSL_KA_L1_osl_ankle_angle_r`. The prefix prevents
collisions with the names in the MSK model. It also lets you identify the
contribution of the device in the compiled model.

## Per-MSK configuration overrides

One device YAML file can contain per-MSK variations for each section except
`actuators` and the legacy `keyframes`. These sections are `attachments`,
`equality`, `joint_overrides`, `keyframe_overrides`, `body_removals`,
`geom_removals`, `mesh_replacements`, `actuator_removals`, `tendon_removals`,
`tendon_modifications`, `body_overrides`, `actuator_overrides`, `contact`,
`sensors` and `sensor_removals`. The schema has this shape:

```yaml
tendon_modifications:
  default:
    - name: hamstrings_r_tendon
      wraps: ...
  myolegs:
    - name: bflh_r_tendon         # 80-muscle model splits the lumped muscle
      wraps: ...
```

The resolver selects the entry for the matching MSK key. If that entry is not
present, the resolver selects `default`. For the sections that support per-MSK
overrides, see [device-config-reference.md](device-config-reference.md).

## What `assist_sim` does *not* do

- **Supply MSK models.** `myo_sim` supplies them.
- **Supply terrain or a scene.** `myoassist.terrains` supplies them.
- **Train policies.** `myoassist` does this.
- **Simulate.** `assist_sim` produces models. You simulate them with MuJoCo, as
  you do with any other model.
- **Supply a viewer.** `examples/quickstart.py` opens `mujoco.viewer` for
  inspection, but the package contains no viewer code.

This narrow scope is intentional. `assist_sim` is the layer for *model
composition*. All other functions are upstream or downstream.
