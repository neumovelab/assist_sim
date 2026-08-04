# Wheelchair env — status / iterate TODO

`assist_sim.upper_body.build_wheelchair(arms="both"|"right"|"left")` composes a seated
wheelchair environment from current myo_sim + the chair hardware here (`wheelchair.xml`
+ `assets/` + 35 meshes in `mesh/`).

## Verified against the ORIGINAL (compiled from the myosuite fork)
- **Arm keyframes** `start_return` / `pushing` are transcribed from the original's
  keyframes; at `pushing` the hand position (in the humerus frame) matches the original
  to **0.0000 m**. Mirrored onto the active arm(s).
- **Legs** are baked rigid into a seated pose and the leg joints deleted — **0 leg
  joints**, matching the original (which shipped zero leg joints). The model **loads
  seated at `qpos0`** (no keyframe needed) and the exported XML round-trips.
- `both` nu=126, `right`/`left` nu=63.

## Structure (current myo_sim; faithful to the collaborator's original)
- **Torso**: locked, muscle-less scaffold (from `myoarms`).
- **Legs**: muscle-less, baked rigid seated (no leg DOF). `ref` cannot pose a hinge (it
  only relabels the zero), so the seated rotation is baked into the leg body frames and
  the joints removed — reproducing the original's rigid legs.
- **Arms**: only the selected arm(s) carry muscles + articulate; `qpos0` is neutral and
  the pose comes from the keyframes (net rotation = keyframe qpos, ref=0).
- **Chair**: rigid; a freejoint on the human root lets the rig roll.

## Remaining (fine / later)
- **Feet ↔ footplates**: nudge `_SEATED_LEGS` (ankle/knee) so the soles rest flat on the
  footplates.
- **Ground / terrain**: myo_sim scene (incl. floor) is stripped; add a ground or compose
  a terrain for the wheels to roll on when running a task.
- **Task / actuation**: define the propulsion task (targets, rewards).

## Viewing
It is actuated at the arm(s): in the live viewer the arm droops under passive dynamics and
the rig free-falls (no ground). To see the true arm pose, **pause + select a keyframe**
(`start_return` / `pushing`). The legs are rigid geometry and always seated.

## Provenance / mesh policy
Chair relocated from the retired `myoassist` vendored myosuite fork (`myowheelchair`);
`assets/wheelchair_{assets,chain}.xml` repointed to the local `mesh/` (19 STL + 16 handrail
collision OBJs). **No anatomical/MSK meshes are housed here** — the human comes from
`myo_sim` at build time.
