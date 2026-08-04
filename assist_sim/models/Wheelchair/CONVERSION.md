# Wheelchair — assist_sim conversion

How this environment maps to the original collaborator model, so the change is
traceable. Describes the **final product**, not the porting process.

Build it with `assist_sim.upper_body.build_wheelchair(arms="both"|"right"|"left",
torso="passive"|"muscled")` → `(MjModel, MjData)`.

## Original (collaborator env, retired myosuite fork)
`myosuite/envs/myo/wheelchair/myowc+arm.xml` (+ `myowc+leftarm.xml`):
- Single **right arm** (63 muscles) on a `myotorso_abdomen` torso + legs, nested
  rigidly inside a freejointed wheelchair body; wheels/casters jointed; rolls on a floor.
- Torso + legs headed *"MODIFIED TO LOCK JOINTS IN A SITTING POSITION"* → **no torso or
  leg joints** (rigid seated); only the arm articulates.
- Two keyframes, `start_return` + `pushing`, drive the propulsion cycle.
- Meshes: the vendored myo_sim v0.1.0 set (anatomical **and** chair hardware).

## assist_sim version (final)
- **Human from current myo_sim** (`myoarms` composition), built at runtime. Anatomical
  meshes come from the `myo_sim` package — **none are housed in assist_sim**.
- **Arms configurable** — `both` (mirrored bimanual, 126 muscles), `right`/`left` (63).
  The original single-right-arm == `arms="right"`.
- **Torso configurable** — `passive` (default: locked, muscle-less scaffold, matching the
  original's rigid torso) or `muscled` (active `myotorso`; trunk muscles, nu 336).
- **Legs** — muscle-less; the seated pose is baked into the leg body geometry and every
  leg joint is deleted → rigid seated legs, no leg DOF (reproducing the original).
- **Chair** — the wheelchair hardware (here in `models/Wheelchair/`) is fixed rigidly to
  the torso; a freejoint on the rig + jointed wheels/casters roll on a ground plane.
- **Keyframes** — `start_return` + `pushing`, the original's arm joint values transcribed
  and mirrored onto the active arm(s). Timestep 1 ms (matches the original).

## Diff vs the original
| | Original | assist_sim |
|---|---|---|
| Human source | vendored static XML (myo_sim v0.1.0) | current myo_sim `myoarms`, runtime compose |
| Anatomical meshes | duplicated in the fork | from the myo_sim package (none housed here) |
| Arms | right only | `both` / `right` / `left` (mirrored) |
| Torso | rigid abdomen | `passive` (default) or `muscled` |
| Legs | rigid (joints removed) | rigid (pose baked + joints deleted) — same result |
| Base | wheelchair freejoint, human nested inside | freejoint on the human rig, chair rigid to torso (equivalent) |
| Keyframes | 2 (arm) | 2 (arm), transcribed by joint name |
| Meshes housed | full vendored set | only 35 chair-hardware meshes (`mesh/`) |

## Fidelity
At the `pushing` keyframe the hand position (in the chair frame) matches the original to
**< 1 mm**, using the original's arm joint angles verbatim.

## Files
- `wheelchair.xml` + `assets/wheelchair_{assets,chain}.xml` — chair-hardware model.
- `mesh/` — 35 chair-hardware meshes (19 STL + 16 handrail collision OBJ).
- Built by `assist_sim/upper_body.py`.
