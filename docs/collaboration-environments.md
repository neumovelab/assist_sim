# Collaboration Environments (upper-body)

`assist_sim`'s **collaboration environments** are single *composed models*
that pair a `myo_sim` human with a specific piece of collaborator hardware —
a wheelchair, a prosthetic arm, an exosuit. They were relocated from a retired
myosuite fork and rebuilt on the current `myo_sim` composition path. Unlike the
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
| MSK choice | Any compatible registry MSK | The human is composed inside the builder (no MSK key) |
| Configuration | Device YAML + per-MSK overrides | Builder keyword arguments (e.g. `arms=`, `torso=`) |
| Output | `(MjModel, MjData)` | `(MjModel, MjData)` |

Because they are not registry devices, they do **not** appear in
`python -m assist_sim list` or `get_available_combinations()`, and they cannot
be produced through `load_combined` / `combine`.

## The build API

Each environment has a builder in `assist_sim/upper_body.py` that returns a
compiled model paired with a fresh `MjData`:

```python
from assist_sim.upper_body import build_wheelchair, build_mpl, build_auxivo_liftsuit

model, data = build_wheelchair(arms="both", torso="passive")   # "both" | "right" | "left"; "passive" | "muscled"
model, data = build_mpl(...)               # MPL — Modular Prosthetic Limb   (being added in parallel)
model, data = build_auxivo_liftsuit(...)   # AuxivoLiftsuit — torso back-exosuit (being added in parallel)
```

Every builder returns `(mujoco.MjModel, mujoco.MjData)` — a standard compiled
MuJoCo model plus data initialized from `qpos0`, ready to step or render.
`build_wheelchair` also has a `build_wheelchair_spec(...)` companion that
returns the uncompiled `MjSpec` if you want to keep composing.

> **Only Wheelchair is fully ported today.** `build_mpl` and
> `build_auxivo_liftsuit` are being added in parallel; the shape above is what
> they will follow. Until they land, calling them will not resolve.

## Common properties

All three environments share the same construction conventions:

- **Human from the current `myo_sim` import.** The anatomical body is composed
  at build time from `myo_sim` (the wheelchair uses the `myoarms` composition).
  **Anatomical meshes are not housed in assist_sim** — they come from the
  `myo_sim` package, same as for the lower-limb devices.
- **Device hardware meshes only.** The environment-specific hardware meshes
  (chair frame, prosthetic parts, exosuit shell) live under
  `models/<Name>/mesh/`. That is the *only* mesh content assist_sim houses for
  these environments.
- **Non-articulating parts are baked rigid.** Anything that does not need a DOF
  (e.g. the wheelchair's seated legs) has its pose baked into the body geometry
  and its joints deleted, reproducing the original collaborator model.
- **Transcribed keyframes.** Each environment ships keyframes copied from the
  original collaborator environment, mapped by joint name onto this build's
  joints (e.g. the wheelchair's `start_return` + `pushing` propulsion poses).
- **A per-environment `CONVERSION.md`.** Each `models/<Name>/` directory carries
  a `CONVERSION.md` recording how the assist_sim environment maps back to the
  original collaborator model, so the port is traceable.
- **Model-only output.** As with the rest of assist_sim, the compiled model is
  the human + device; scene/terrain is layered on downstream (see
  [concepts.md](concepts.md)). The wheelchair adds a temporary ground plane for
  standalone inspection, which terrain composition replaces.

## The environments

| Environment | Description | Builder call | Conversion doc |
|---|---|---|---|
| **Wheelchair** | Seated human propelling a manual wheelchair | `build_wheelchair(arms=..., torso=...)` | [`models/Wheelchair/CONVERSION.md`](../assist_sim/models/Wheelchair/CONVERSION.md) — **available** |
| **MPL** | Modular Prosthetic Limb — a robotic prosthetic arm/hand | `build_mpl(...)` | `models/MPL/CONVERSION.md` — *forthcoming* |
| **AuxivoLiftsuit** | Passive torso back-exosuit | `build_auxivo_liftsuit(...)` | `models/AuxivoLiftsuit/CONVERSION.md` — *forthcoming* |

MPL and AuxivoLiftsuit are being ported now; document-level details will land in
their own `CONVERSION.md` as each port completes. Only the Wheelchair is fully
ported and documented today.

## Worked example: Wheelchair

The wheelchair is a seated human propelling a manual wheelchair, ported from the
retired myosuite `myowc+arm.xml` environment. Full detail is in
[`models/Wheelchair/CONVERSION.md`](../assist_sim/models/Wheelchair/CONVERSION.md);
the essentials:

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
  every leg joint deleted — rigid seated legs with no leg DOF, only the arm(s)
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

## See also

- [available-models.md](available-models.md) — the registry MSKs and lower-limb
  devices, plus the upper-body environment inventory.
- [concepts.md](concepts.md) — the composition pipeline and the repo split.
