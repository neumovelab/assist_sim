# Collaboration Environments (upper-body)

The **collaboration environments** in `assist_sim` are upper-body models. Each
one pairs a `myo_sim` human with one item of collaborator hardware: a
wheelchair, a back-exosuit or a bionic manipulation setup. The set also includes
one standalone collaborator robot, the MPL. `assist_sim` took these environments
from a retired myosuite fork and rebuilt them on the current `myo_sim`
composition path. They differ from the
[lower-limb devices](available-models.md#device-models): they are **not registry
devices**, and they are **not modular**. A dedicated builder function in
`assist_sim/upper_body.py` makes each one, not `load_combined`.

## How they differ from the modular lower-limb devices

The [lower-limb devices](available-models.md#device-models) are modular
compositions of a musculoskeletal (MSK) model and a device. You can combine any
registry MSK model with any device. `registry` and `load_combined` resolve the
pair. The registry finds new devices automatically in
`models/<Device>/*config.yaml`. See [concepts.md](concepts.md) for that
pipeline.

The collaboration environments have the opposite shape. Each environment is one
fixed, fully composed model:

| | Lower-limb devices | Collaboration environments |
|---|---|---|
| Shape | Modular MSK × device composition | Single composed model per environment |
| Entry point | `load_combined("<msk>", "<device>")` | `build_<env>(...)` in `assist_sim.upper_body` |
| Discovery | Found automatically in `models/<Device>/*config.yaml`; in the [registry](usage.md#registry) | Not registry devices; `list` and `get_available_combinations` do not show them |
| MSK choice | Any compatible registry MSK model | The builder composes the human, if there is one (no MSK key) |
| Configuration | Device YAML + per-MSK overrides | Builder keyword arguments (for example, `arms=`, `torso=`) |
| Output | `(MjModel, MjData)` | `(MjModel, MjData)` |

The collaboration environments are not registry devices. Therefore
`python -m assist_sim list` and `get_available_combinations()` do **not** show
them. You cannot make them with `load_combined` or `combine`.

## The build API

Each environment has a builder in `assist_sim/upper_body.py`. The builder
returns a compiled model and a new `MjData`:

```python
from assist_sim.upper_body import (
    build_wheelchair,
    build_mpl,
    build_auxivo_liftsuit,
    build_bionic_bimanual,
)

model, data = build_wheelchair(arms="both", torso="passive")  # "both"|"right"|"left"; "passive"|"muscled"
model, data = build_mpl()               # standalone bimanual MPL robot (no myo_sim human)
model, data = build_auxivo_liftsuit()   # passive back-exosuit on the muscled myotorso
model, data = build_bionic_bimanual()   # MyoChallenge biological-arm + MPL-prosthesis manipulation task
```

Every builder returns `(mujoco.MjModel, mujoco.MjData)`. This is a standard
compiled MuJoCo model with data that comes from `qpos0`. You can step it or
render it immediately.

The three **composed** environments also give a `build_*_spec(...)` companion:
`build_wheelchair_spec`, `build_auxivo_liftsuit_spec` and
`build_bionic_bimanual_spec`. The companion returns the uncompiled `MjSpec`. Use
it if you want to compose more elements, or if you want to serialize the spec
(see [How to export a composed environment](#how-to-export-a-composed-environment)).
`build_mpl` has **no** spec companion, because `assist_sim` loads the MPL
directly from its XML file and does not compose it.

## Common properties

The three **composed** environments (Wheelchair, AuxivoLiftsuit,
bionic-bimanual) use the same construction conventions. **MPL is the
exception.** MPL is a self-contained collaborator *robot* with no `myo_sim`
human, and `assist_sim` loads it directly (see [MPL](#mpl) below).

- **The human comes from the current `myo_sim` import.** `assist_sim` composes
  the anatomical body from `myo_sim` at build time. The Wheelchair uses the
  `myoarms` composition, AuxivoLiftsuit uses the muscled `myotorso`, and
  bionic-bimanual uses a passive anatomical torso with a right arm.
  **assist_sim does not hold the anatomical meshes.** They come from the
  `myo_sim` package, the same as for the lower-limb devices.
- **Device hardware meshes only.** The hardware meshes for each environment
  (chair frame, exosuit shell, prosthetic parts, task object) are in
  `models/<Name>/`. This is the *only* mesh content that assist_sim holds for
  these environments. MPL is a robot, so it holds its full mesh set in
  `models/MPL/meshes/`. That mesh set is also device hardware, so it agrees with
  the policy.
- **The parts that do not articulate are rigid.** Some parts do not need a
  degree of freedom (DOF). Examples are the seated legs of the Wheelchair and
  the legs of bionic-bimanual in a standing pose. `assist_sim` writes the pose
  of each such part into the body geometry and removes its joints. The result
  reproduces the original collaborator model.
- **Transcribed keyframes.** If the original environment supplied keyframes,
  `assist_sim` copies them and maps them by joint name onto the joints of this
  build. These are the two propulsion poses of the Wheelchair (`start_return`
  and `pushing`) and the four manipulation task keyframes of bionic-bimanual.
  MPL and AuxivoLiftsuit have no keyframes, because their originals supplied
  none.
- **One `CONVERSION.md` for each environment.** Each `models/<Name>/` directory
  holds a `CONVERSION.md` file. That file records how the assist_sim
  environment maps back to the original collaborator model, so you can trace
  the port. The `CONVERSION.md` in the MPL directory covers both `build_mpl`
  and `build_bionic_bimanual`.
- **Model-only output (composed environments).** The compiled composed model
  holds the human and the device, the same as the rest of assist_sim. A
  downstream step adds the scene and the terrain (see
  [concepts.md](concepts.md)). The Wheelchair adds a temporary ground plane for
  standalone inspection, and the terrain composition replaces that plane.
  AuxivoLiftsuit is strictly model-only, and bionic-bimanual supplies the base
  pedestal that it stands on, but no ground plane. MPL is the exception,
  because it supplies its own basic scene (floor, skybox, lights).

## The environments

| Environment | Description | Builder call | Conversion doc |
|---|---|---|---|
| **Wheelchair** | Seated human who propels a manual wheelchair | `build_wheelchair(arms=..., torso=...)` | [`models/Wheelchair/CONVERSION.md`](../assist_sim/models/Wheelchair/CONVERSION.md) (**available**) |
| **MPL** | Standalone bimanual Modular Prosthetic Limb robot | `build_mpl()` | [`models/MPL/CONVERSION.md`](../assist_sim/models/MPL/CONVERSION.md) (**available**) |
| **AuxivoLiftsuit** | Passive back-exosuit on the muscled `myotorso` | `build_auxivo_liftsuit()` | [`models/AuxivoLiftsuit/CONVERSION.md`](../assist_sim/models/AuxivoLiftsuit/CONVERSION.md) (**available**) |
| **bionic-bimanual** | MyoChallenge manipulation task with a biological arm and an MPL prosthesis | `build_bionic_bimanual()` | [`models/MPL/CONVERSION.md`](../assist_sim/models/MPL/CONVERSION.md), bionic section (**available**) |

All four environments are ported and available today. Each `CONVERSION.md`
gives the full detail for one environment. The essential facts follow.

### Wheelchair

This environment is a seated human who propels a manual wheelchair. It is a
port of the retired myosuite `myowc+arm.xml` environment.
`build_wheelchair(arms="both", torso="passive")` composes the selected muscled
arms on the selected torso. It sets the legs to a rigid seated pose and fixes
the chair hardware to the torso. It also supplies the `start_return` and
`pushing` propulsion keyframes.

```python
from assist_sim.upper_body import build_wheelchair

model, data = build_wheelchair(arms="both", torso="passive")
```

- **`arms`**: `"both"` (mirrored bimanual, 126 muscles), `"right"` or `"left"`
  (63 muscles each). The original model has a single right arm, which is
  `arms="right"`.
- **`torso`**: `"passive"` or `"muscled"`. The default is `"passive"`, a locked
  scaffold with no muscles that matches the rigid torso of the original.
  `"muscled"` is the active `myotorso` with spine joints and trunk muscles.
- The **legs** have no muscles. The build writes the seated pose into the body
  geometry and removes every leg joint. The legs are rigid and have no leg DOF.
  Only the arms articulate.
- The build fixes the **chair** hardware (from `models/Wheelchair/`) rigidly to
  the torso. A freejoint on the rig lets the seated human and the chair roll as
  one free body on the ground plane. The wheels and the casters have their own
  joints.
- The **keyframes** `start_return` and `pushing` drive the propulsion cycle.
  The build transcribes the arm joint values of the original and mirrors them
  onto the active arms. The timestep is 1 ms, the same as the original.

At the `pushing` keyframe, the hand position in the chair frame matches the
original model to less than 1 mm. This check uses the arm joint angles of the
original without a change. See the
[conversion doc](../assist_sim/models/Wheelchair/CONVERSION.md) for the full
diff against the original, the fidelity check and the file inventory.

### MPL

The **Modular Prosthetic Limb** (JHU/APL) is a self-contained *robotic*
bimanual arm and hand model. It has its own meshes and actuators, and it has
**no `myo_sim` human**. It comes as the bimanual "SALLY" configuration: a torso
with two MPL arms and simplified hands. `assist_sim` holds it almost without a
change from the collaborator fork. Therefore `build_mpl()` **loads it directly**
and does not compose it (nbody 26, 19 actuators, 25 meshes).

The meshes are robot hardware, so the full set is in `models/MPL/meshes/`. MPL
carries its own basic scene (floor, skybox, lights), and it supplies no
keyframes.

```python
from assist_sim.upper_body import build_mpl

model, data = build_mpl()
```

See [`models/MPL/CONVERSION.md`](../assist_sim/models/MPL/CONVERSION.md) for the
pruned-variant list and the path and attribute cleanups.

### AuxivoLiftsuit

This environment is a passive back-exosuit in the style of the Auxivo Liftsuit.
The human wears it over the **muscled** `myo_sim` `myotorso`, which has spine
joints and trunk muscles. `build_auxivo_liftsuit()` attaches the exosuit
hardware fragment to the torso at the original exo-to-trunk pose. A rigid map
from the torso pose as authored to the torso pose of this build gives that pose.
The builder then couples the exosuit with the two original body welds
(`torso`↔`exo_torso`, `lumbar4`↔`exo_lumbar4`) and four spring tendons.

It compiles to nu 210, ntendon 214 (210 muscle tendons and 4 exo spring
tendons) and neq 17. These actuator, tendon and equality counts are identical
to the counts of the original. This environment supplies no keyframes. Only the
three exosuit meshes are in `models/AuxivoLiftsuit/mesh/`; the human comes from
the `myo_sim` package.

```python
from assist_sim.upper_body import build_auxivo_liftsuit

model, data = build_auxivo_liftsuit()
```

See
[`models/AuxivoLiftsuit/CONVERSION.md`](../assist_sim/models/AuxivoLiftsuit/CONVERSION.md)
for the placement map, the restored default classes, and the fidelity check
against the compiled original.

### bionic-bimanual

This environment is the MyoChallenge *"bionic bimanual"* manipulation task. A
biological **right** arm faces an MPL **left** prosthetic arm. The biological
arm is a `myo_sim` human on a passive anatomical torso with rigid legs in a
standing pose.
Between the two arms is a YCB gelatin box (`manip_object`), a body with a
freejoint. The box starts on a `start` pillar, and the task moves it to a `goal`
pillar (two `mocap` cylinders). A touch sensor sits on the box, and the full
model stands on a myosuite-sized base pedestal.

`build_bionic_bimanual()` composes the human. The current `myo_sim` `myoarm_r`
cannot assemble itself, because its chest muscle origins moved to `myotorso` in
the 2026-06 refactor. Therefore the builder makes the human as a passive torso
with a right arm. It then aligns the human rigidly to the world pose of the
original arm.

The builder then attaches the static half (MPL prosthesis, object, pillars,
pedestal, sensor) from `models/MPL/assets/` and `models/YCB/`. It compiles to
nu 80, nq 71 and nsensor 1, with 4 task keyframes that the builder transcribes
by joint name. All four keyframes reproduce the world poses of the object, the
prosthesis and the hand in the original, to float precision.

```python
from assist_sim.upper_body import build_bionic_bimanual

model, data = build_bionic_bimanual()
```

See the bionic section of
[`models/MPL/CONVERSION.md`](../assist_sim/models/MPL/CONVERSION.md) for the
source of each part, the `multiccd` contact note, and the faithfulness
comparison against the baseline.

## How to export a composed environment

`export_upper_body_xml(spec, output_path)` serializes the three composed
environments to a standalone XML file that you can reload. Give it the output
of the `build_*_spec(...)` companion, not the output of the compiled builder:

```python
from assist_sim.upper_body import build_auxivo_liftsuit_spec, export_upper_body_xml

export_upper_body_xml(build_auxivo_liftsuit_spec(), "auxivo_liftsuit.xml")
```

`export_upper_body_xml` calls `utils.export_combined_xml`, the same path that
the lower-limb devices use. That function hoists the merged fragment defaults,
gives them names, and rewrites the mesh paths as absolute paths to the output.

A **raw `spec.to_xml()` does not reload**. The unnamed `main` defaults of the
attached fragments collapse into anonymous `<default>` blocks, and the myo_sim
asset directories go away. Therefore the output fails to reload. The reloaded
model reproduces the live build to float round-trip precision. `build_mpl` has
no spec path and no export path, because the MPL is already a standalone XML
file on disk.

The exported XML is **model-only, but not scene-free**. It carries no ground, no
hfield and no floor; a downstream scene or terrain supplies those elements. It
does carry lighting and a backdrop. `_strip_scene_visual` removes the myosuite
headlight. Then `_ensure_minimal_visual` adds a soft headlight and a neutral
gradient skybox, so that the file renders correctly on its own.

You can override both additions. The `<headlight>` or skybox of a downstream
scene has priority when you put it on top. If you want a specific backdrop,
replace the skybox in your wrapper XML. Every export takes this treatment,
upper-body and lower-limb.

## See also

- [available-models.md](available-models.md): the registry MSK models and the
  lower-limb devices, with the inventory of the upper-body environments.
- [concepts.md](concepts.md): the composition pipeline and the repository
  split.
