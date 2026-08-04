# Wheelchair env — cleanup / iterate TODO

Status: **scaffold + prototype**. `assist_sim.upper_body.build_wheelchair()` composes
`myo_sim.build_spec("myoarms")` (bimanual, 126 muscles, meshes from myo_sim) with the
chair hardware here (`wheelchair.xml` + `assets/` + 35 meshes in `mesh/`) and compiles.
The human sits upright in the seat with arms by the pushrims (verified by render).

Remaining to make it a usable task env (iterate with visual feedback):

- **Seat pose polish** — `_CHAIR_SEAT_OFFSET` in `upper_body.py` seats the pelvis on
  the pan; fine-tune offset + facing/yaw so the ischia rest cleanly and the hands land
  on the pushrims.
- **Hand ↔ pushrim coupling** — keyframe the arm pose gripping the handrails, and decide
  the grip mechanism (weld/equality during push, or contact-only). The chain carries
  `wheelchair_grip_{l,r}` / `hand_start_*` / `hand_TARGET_*` sites for this.
- **L / R single-arm variants** — `build_wheelchair` currently does bimanual (`myoarms`).
  myo_sim has no single-arm *spec* builder (`myoarm_r` is compiled-model-only), so a
  single-arm variant means pruning one arm from the `myoarms` spec (mind the cross-body
  muscles — see the attach note in `upper_body.py`).
- **Ground / terrain** — the myo_sim scene (incl. floor) is stripped; add a ground or
  compose a terrain for the wheels to roll on when running the task.
- **Task / actuation** — define the propulsion task (targets, rewards). Out of scope for
  the model port.

## Provenance / mesh policy

- Chair relocated from the retired `myoassist` vendored myosuite fork (`myowheelchair`).
  `wheelchair.xml` + `assets/wheelchair_{assets,chain}.xml` are repointed to the local
  `mesh/` dir (35 chair-hardware meshes: 19 STL + 16 handrail collision OBJs).
- **No anatomical/MSK meshes are housed here** — the human comes from `myo_sim` at build
  time. The old fork carried a full duplicate MSK mesh set; that is intentionally dropped.
