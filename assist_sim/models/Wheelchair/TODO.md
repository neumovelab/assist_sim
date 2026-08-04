# Wheelchair env — status / iterate TODO

`assist_sim.upper_body.build_wheelchair(arms="both"|"right"|"left")` composes a
seated wheelchair environment from current myo_sim + the chair hardware here
(`wheelchair.xml` + `assets/` + 35 meshes in `mesh/`). Verified: all three configs
compile and reload; `both` nu=126, `right`/`left` nu=63; ships `start_return` +
`pushing` keyframes.

## Structure (mirrors the collaborator's original, on current myo_sim)
- **Torso**: locked, muscle-less passive scaffold (from `myoarms`).
- **Arms**: only the selected arm(s) carry muscles + articulate; `both` mirrors the
  push symmetrically.
- **Legs**: muscle-less scaffold, frozen in the seated pose via joint-equality locks.
- **Chair**: rigid; a freejoint on the human root lets the rig roll (physically
  identical to the original's chair-rooted freejoint — the human can't be the
  attached child without dropping its cross-body propulsion muscles).

## Remaining (iterate with visual feedback)
- **Pushrim grip** — fine-tune `_ARM_PUSH` / `_ARM_START` (and `_CHAIR_SEAT_OFFSET`)
  so the hands land on the pushrims through the push/recovery cycle. Values are
  seeded, not yet rim-accurate. Optionally weld/contact the hand to the rim.
- **Feet on footplates** — nudge `_SEATED_LEGS` (hip/knee/ankle) so the feet rest on
  the footplates.
- **Ground / terrain** — the myo_sim scene (incl. floor) is stripped; add a ground or
  compose a terrain for the wheels to roll on when running a task.
- **Task / actuation** — define the propulsion task (targets, rewards). Out of scope
  for the model port.

## Provenance / mesh policy
- Chair relocated from the retired `myoassist` vendored myosuite fork (`myowheelchair`);
  `assets/wheelchair_{assets,chain}.xml` repointed to the local `mesh/` (19 STL + 16
  handrail collision OBJs). Grip/target sites (`wheelchair_grip_*`, `hand_TARGET_*`)
  carried over from the original chain.
- **No anatomical/MSK meshes are housed here** — the human comes from `myo_sim` at
  build time.
