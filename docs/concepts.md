# Concepts

This doc covers the architecture: why `assist_sim` exists, the two-phase
pipeline, and how it fits with `myo_sim` and downstream training frameworks.

## The three-repo split

`assist_sim` sits between an upstream MSK source and downstream training
frameworks. Four packages collaborate:

- **`myo_sim`** ships the baseline MSK XML files (myoLeg22_2D, myoLeg26_3D,
  myoleg80) and their meshes. `assist_sim` depends on it as a pip-installed
  package, resolving MSK paths through `importlib.resources`.
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
  *model-only* — no ground body, no terrain include — and
  `myoassist.terrains` layers the scene on top before simulation.

> *Diagram placeholder — figure of package flow
> will replace this in a later docs pass.*

## The two-phase pipeline

When `load_combined_model(...)` is called, the model flows through two
distinct phases.  The inputs are a baseline MSK XML and a device YAML;
the outputs are an `(MjModel, MjData)` pair ready to step and (optionally)
an exported combined XML.

**Phase 1 — Preprocess (XML pass).**  An ElementTree pass over the MSK
XML applies every operation that *removes* content from the model.  In
order:

- Inline `<include>` directives so all referenced subtrees are present in
  the working tree.
- Merge duplicate top-level sections (`worldbody`, `asset`, `default`,
  `contact`, `sensor`, `tendon`, `actuator`, `equality`, `keyframe`) that
  the inlining may have produced, mirroring MuJoCo's compile-time merge.
- Apply body / geom / actuator / tendon removals — the prosthetic surgery
  side of the pipeline.
- Apply tendon wrap edits (auto-prune wraps whose sites lived on removed
  bodies; honor any `tendon_modifications` overrides).
- Prune the qpos / qvel slots of every keyframe to drop indices owned by
  removed joints, preserving authored values for surviving joints.
- Cascade cleanup: remove contact pairs, sensors, and equality constraints
  that reference any removed element.
- Strip the terrain content (ground body, ground-plane geom, terrain
  include) — assist_sim outputs are model-only.

A temp XML is written to disk at the end of Phase 1, in the same directory
as the source MSK (so relative paths still resolve).

**Phase 2 — MjSpec (attach pass).**  `MjSpec.from_file` loads the
preprocessed temp XML and the device XML.  Every operation here is either
additive or an in-place attribute edit (no `MjSpec.delete` calls — those
don't exist on MuJoCo 3.3.3):

- Attach each device body under its parent body in the MSK, applying the
  device-name prefix to all imported elements (bodies, sites, meshes,
  joints, actuators, tendons).
- Honor per-MSK attachment overrides (different `parent_body`, `pos`, or
  `quat` per MSK).
- Apply joint range / damping overrides on existing MSK joints.
- Import device-side spatial tendons and tendon-transmission actuators
  from the device XML, with the device prefix.
- Add YAML-declared joint-transmission actuators.
- Compile the spec into an `MjModel`.
- Rebuild keyframes by joint *name* (model-agnostic) — restores authored
  values to surviving joints, applies `keyframe_overrides`, then
  recompiles to lock the keyframe table into the final model.

The compiled `MjModel` and a fresh `MjData` are returned.  If `export_xml`
was provided, the spec is also serialized to a clean XML at that path.

> *Diagram placeholder — a figure of the two phases (with their
> ordered passes and the temp-XML handoff) will replace
> in a later docs pass.*

### Why two phases

MuJoCo 3.3.3 (the version pinned by downstream `myoassist`) does not expose
`MjSpec.delete`. Anything that *removes* content from the model has to happen
before the spec is constructed — i.e. at the XML level. Hence Phase 1
operates on `ElementTree`. Anything that *adds* or *edits in place*
(attachments, joint property changes, actuators, keyframe writes) works fine
on the spec — Phase 2.

This split also produces cleaner semantics: removals are model surgery (done
once, on the baseline), and the spec-level operations are device-side
authoring (additive, namespaced via the device prefix).

## Naming conventions

### Registry keys

- **MSK keys**: `myoLeg22_2D`, `myoLeg26_3D`, `myoLeg80`. Curated list in
  `assist_sim/registry.py:_COMPATIBLE_MSK_KEYS`. Each maps to a tuple
  `(myo_sim_subpackage, filename)`; the file is loaded via
  `importlib.resources`.
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
  myoLeg80:
    - name: gasmed_r_tendon       # 80-muscle equivalent
      wraps: ...
```

The resolver picks the matching MSK key if present, else `default`. See
[device-config-reference.md](device-config-reference.md) for which sections
support per-MSK overrides.

## What `assist_sim` does *not* do

- **Provide MSK models** — those live in `myo_sim`.
- **Provide terrain / scene** — that's `myoassist.terrains`.
- **Train policies** — that's `myoassist`.
- **Simulate** — `assist_sim` produces models; you simulate them with
  MuJoCo as you would any model.
- **Provide a viewer** — `examples/quickstart.py` opens `mujoco.viewer` for
  inspection, but the package itself has no viewer logic.

This narrow scope is intentional: `assist_sim` is the *model composition*
layer. Everything else is upstream or downstream.
