# Collaboration Environments (upper-body)

`assist_sim`'s **collaboration environments** are upper-body models that pair a
`myo_sim` human with a specific piece of collaborator hardware — a wheelchair, a
back-exosuit, a bionic manipulation setup — plus one standalone collaborator
robot (the MPL). They were relocated from a retired myosuite fork and rebuilt on
the current `myo_sim` composition path. Unlike the
[lower-limb devices](available-models.md#device-models), they are **not
registry devices** and are **not modular**: each one is produced by a dedicated
builder function in `assist_sim/upper_body.py`, not by `load_combined`.

## How they differ from the modular lower-limb devices

The [lower-limb devices](available-models.md#device-models) are modular
**MSK × device** compositions: any registry MSK can be combined with any
device, the pairing is resolved through `registry` / `load_combined`, and new
devices are autodiscovered from `models/<Device>/*config.yaml`. See
[concepts.md](concepts.md) for that pipeline.

The collaboration environments are the opposite shape — one fixed, fully
composed model per environment:

| | Lower-limb devices | Collaboration environments |
|---|---|---|
| Shape | Modular MSK × device composition | Single composed model per environment |
| Entry point | `load_combined("<msk>", "<device>")` | `build_<env>(...)` in `assist_sim.upper_body` |
| Discovery | Autodiscovered from `models/<Device>/*config.yaml`; in the [registry](usage.md#registry) | Not registry devices; not returned by `list` / `get_available_combinations` |
| MSK choice | Any compatible registry MSK | The human (if any) is composed inside the builder (no MSK key) |
| Configuration | Device YAML + per-MSK overrides | Builder keyword arguments (e.g. `arms=`, `torso=`) |
| Output | `(MjModel, MjData)` | `(MjModel, MjData)` |

Because they are not registry devices, they do **not** appear in
`python -m assist_sim list` or `get_available_combinations()`, and they cannot
be produced through `load_combined` / `combine`.

## The build API

Each environment has a builder in `assist_sim/upper_body.py` that returns a
compiled model paired with a fresh `MjData`:

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

Every builder returns `(mujoco.MjModel, mujoco.MjData)` — a standard compiled
MuJoCo model plus data initialized from `qpos0`, ready to step or render. The
three **composed** environments also expose a `build_*_spec(...)` companion —
`build_wheelchair_spec`, `build_auxivo_liftsuit_spec`,
`build_bionic_bimanual_spec` — that returns the uncompiled `MjSpec` if you want
to keep composing, or to serialize it (see
[Exporting a composed env](#exporting-a-composed-env)). `build_mpl` has **no**
spec companion: the MPL is loaded directly from its XML, not composed.

## Common properties

The three **composed** environments (Wheelchair, AuxivoLiftsuit,
bionic-bimanual) share the same construction conventions. **MPL is the
exception** — a self-contained collaborator *robot* with no `myo_sim` human,
loaded directly (see [MPL](#mpl) below).

- **Human from the current `myo_sim` import.** The anatomical body is composed
  at build time from `myo_sim` — the Wheelchair uses the `myoarms` composition,
  AuxivoLiftsuit the muscled `myotorso`, and bionic-bimanual a passive
  anatomical torso plus a right arm. **Anatomical meshes are not housed in
  assist_sim** — they come from the `myo_sim` package, same as for the
  lower-limb devices.
- **Device hardware meshes only.** The environment-specific hardware meshes
  (chair frame, exosuit shell, prosthetic parts, task object) live under
  `models/<Name>/`. That is the *only* mesh content assist_sim houses for these
  environments. (MPL, being a robot, houses its full mesh set under
  `models/MPL/meshes/` — still device hardware, consistent with the policy.)
- **Non-articulating parts are baked rigid.** Anything that does not need a DOF
  (the Wheelchair's seated legs, bionic-bimanual's standing legs) has its pose
  baked into the body geometry and its joints deleted, reproducing the original
  collaborator model.
- **Transcribed keyframes.** Where the original environment shipped keyframes,
  they are copied over and mapped by joint name onto this build's joints — the
  Wheelchair's `start_return` + `pushing` propulsion poses (2) and
  bionic-bimanual's 4 manipulation-task keyframes. MPL and AuxivoLiftsuit carry
  none (their originals shipped none).
- **A per-environment `CONVERSION.md`.** Each `models/<Name>/` directory carries
  a `CONVERSION.md` recording how the assist_sim environment maps back to the
  original collaborator model, so the port is traceable. The MPL directory's
  `CONVERSION.md` covers both `build_mpl` and `build_bionic_bimanual`.
- **Model-only output (composed envs).** As with the rest of assist_sim, the
  compiled composed model is the human + device with scene/terrain layered on
  downstream (see [concepts.md](concepts.md)). The Wheelchair adds a temporary
  ground plane for standalone inspection (terrain composition replaces it),
  AuxivoLiftsuit is strictly model-only, and bionic-bimanual ships the base
  pedestal it stands on (no ground plane). MPL is the exception — it ships its
  own basic scene (floor, skybox, lights).

## The environments

| Environment | Description | Builder call | Conversion doc |
|---|---|---|---|
| **Wheelchair** | Seated human propelling a manual wheelchair | `build_wheelchair(arms=..., torso=...)` | [`models/Wheelchair/CONVERSION.md`](../assist_sim/models/Wheelchair/CONVERSION.md) — **available** |
| **MPL** | Standalone bimanual Modular Prosthetic Limb robot | `build_mpl()` | [`models/MPL/CONVERSION.md`](../assist_sim/models/MPL/CONVERSION.md) — **available** |
| **AuxivoLiftsuit** | Passive back-exosuit on the muscled `myotorso` | `build_auxivo_liftsuit()` | [`models/AuxivoLiftsuit/CONVERSION.md`](../assist_sim/models/AuxivoLiftsuit/CONVERSION.md) — **available** |
| **bionic-bimanual** | MyoChallenge biological-arm + MPL-prosthesis manipulation task | `build_bionic_bimanual()` | [`models/MPL/CONVERSION.md`](../assist_sim/models/MPL/CONVERSION.md) (bionic section) — **available** |

All four environments are ported and available today. Full per-environment
detail lives in each `CONVERSION.md`; the essentials follow.

### Wheelchair

A seated human propelling a manual wheelchair, ported from the retired myosuite
`myowc+arm.xml` environment. `build_wheelchair(arms="both", torso="passive")`
composes the selected muscled arm(s) on the chosen torso, freezes the legs into
a rigid seated pose, fixes the chair hardware to the torso, and ships the
`start_return` + `pushing` propulsion keyframes.

```python
from assist_sim.upper_body import build_wheelchair

model, data = build_wheelchair(arms="both", torso="passive")
```

- **`arms`** — `"both"` (mirrored bimanual, 126 muscles), `"right"`, or
  `"left"` (63 muscles each). The original single-right-arm model corresponds to
  `arms="right"`.
- **`torso`** — `"passive"` (default: a locked, muscle-less scaffold matching the
  original's rigid torso) or `"muscled"` (the active `myotorso` with spine joints
  and trunk muscles).
- **Legs** are muscle-less with the seated pose baked into the body geometry and
  every leg joint deleted — rigid seated legs with no leg DOF; only the arm(s)
  articulate.
- **Chair** hardware (from `models/Wheelchair/`) is fixed rigidly to the torso; a
  freejoint on the rig plus jointed wheels/casters lets the seated human + chair
  roll as one free body on the ground plane.
- **Keyframes** `start_return` and `pushing` drive the propulsion cycle, with the
  original's arm joint values transcribed and mirrored onto the active arm(s).
  The timestep is 1 ms, matching the original.

At the `pushing` keyframe the hand position (in the chair frame) matches the
original model to under 1 mm, using the original's arm joint angles verbatim.
See the [conversion doc](../assist_sim/models/Wheelchair/CONVERSION.md) for the
full diff against the original, the fidelity check, and the file inventory.

### MPL

The **Modular Prosthetic Limb** (JHU/APL) is a self-contained *robotic*
bimanual arm/hand model — its own meshes and actuators, with **no `myo_sim`
human**. It ships as the bimanual "SALLY" configuration (a torso with two MPL
arms plus simplified hands) and is relocated ~verbatim from the collaborator
fork, so `build_mpl()` **loads it directly** rather than composing it (nbody 26,
19 actuators, 25 meshes). Because the meshes are robot hardware, the full set
lives under `models/MPL/meshes/`. MPL carries its own basic scene (floor,
skybox, lights) and ships no keyframes.

```python
from assist_sim.upper_body import build_mpl

model, data = build_mpl()
```

See [`models/MPL/CONVERSION.md`](../assist_sim/models/MPL/CONVERSION.md) for the
pruned-variant list and the path/attribute cleanups.

### AuxivoLiftsuit

A passive Auxivo Liftsuit–style back-exosuit worn over the **muscled** `myo_sim`
`myotorso` (spine joints + trunk muscles). `build_auxivo_liftsuit()` composes the
exosuit hardware fragment onto the torso at the original exo→trunk pose (via a
rigid map from the authoring torso pose to this build's torso pose) and couples
it with the two original body welds (`torso`↔`exo_torso`, `lumbar4`↔`exo_lumbar4`)
plus four spring tendons. It compiles to nu 210, ntendon 214 (210 muscle
+ 4 exo spring tendons) and neq 17 — identical actuator/tendon/equality counts to
the original — and ships no keyframes. Only the three exosuit meshes live in
`models/AuxivoLiftsuit/mesh/`; the human comes from the `myo_sim` package.

```python
from assist_sim.upper_body import build_auxivo_liftsuit

model, data = build_auxivo_liftsuit()
```

See
[`models/AuxivoLiftsuit/CONVERSION.md`](../assist_sim/models/AuxivoLiftsuit/CONVERSION.md)
for the placement map, the restored default classes, and the fidelity check
against the compiled original.

### bionic-bimanual

The MyoChallenge *"bionic bimanual"* manipulation task. A biological **right**
arm (a `myo_sim` human, on a passive anatomical torso with rigid standing legs)
faces an MPL **left** prosthetic arm across a YCB gelatin box (`manip_object`, a
freejointed body) that starts on a `start` pillar and is to be moved to a `goal`
pillar (two `mocap` cylinders); a touch sensor sits on the object, and everything
stands on a myosuite-sized base pedestal. `build_bionic_bimanual()` composes the
human — the current `myo_sim` `myoarm_r` cannot self-assemble (its chest muscle
origins moved to `myotorso` in the 2026-06 refactor), so it is built as a
passive torso + right arm and rigidly aligned to the original arm world pose —
and attaches the static half (MPL prosthesis, object, pillars, pedestal, sensor)
sourced from `models/MPL/assets/` and `models/YCB/`. It compiles to nu 80, nq 71,
nsensor 1 with 4 task keyframes transcribed by joint name; all four reproduce the
original object / prosthesis / hand world poses to float precision.

```python
from assist_sim.upper_body import build_bionic_bimanual

model, data = build_bionic_bimanual()
```

See the bionic section of
[`models/MPL/CONVERSION.md`](../assist_sim/models/MPL/CONVERSION.md) for how each
piece is sourced, the `multiccd` contact note, and the faithfulness comparison
against the baseline.

## Exporting a composed env

The three composed environments can be serialized to a standalone, reloadable
XML with `export_upper_body_xml(spec, output_path)`, using the `build_*_spec(...)`
companion (not the compiled builder):

```python
from assist_sim.upper_body import build_auxivo_liftsuit_spec, export_upper_body_xml

export_upper_body_xml(build_auxivo_liftsuit_spec(), "auxivo_liftsuit.xml")
```

`export_upper_body_xml` routes through `utils.export_combined_xml` (the same path
the lower-limb devices use), which hoists and names the merged fragment defaults
and rewrites mesh paths absolute to the output. A **raw `spec.to_xml()` does not
reload** — the attached fragments' unnamed `main` defaults collapse into
anonymous `<default>` blocks and the myo_sim asset dirs are stripped, so the
output fails to reload. The exported XML is model-only (no scene/lighting); a
downstream scene or terrain supplies those, and the reloaded model reproduces the
live build to float round-trip precision. `build_mpl` has no spec/export path —
the MPL is already a standalone XML on disk.

## See also

- [available-models.md](available-models.md) — the registry MSKs and lower-limb
  devices, plus the upper-body environment inventory.
- [concepts.md](concepts.md) — the composition pipeline and the repo split.
