# MPL — assist_sim conversion

How this environment maps to the original collaborator model. Final product, not the
porting process. Build it with `assist_sim.upper_body.build_mpl()` → `(MjModel, MjData)`.

## Original (collaborator env, retired myosuite fork)
`myosuite/simhive/MPL_sim/` — the JHU/APL **Modular Prosthetic Limb**, a self-contained
**robotic** arm/hand model (its own meshes + actuators; **no musculoskeletal human**).
The shipped bimanual setup is **"SALLY"** (`scenes/sally.xml`): a torso with two MPL arms
+ simplified hands, on a basic scene (floor, skybox, lights).

## assist_sim version (final)
- **Relocated ~verbatim** into `models/MPL/` (`scenes/` + `assets/` + `meshes/`); loaded
  directly by `build_mpl()` (nbody 26, 19 actuators, 25 meshes). It is a robotic device
  model — **not composed** with a myo_sim human (contrast the Wheelchair/AuxivoLiftsuit).
- Because the meshes are robot hardware (not anatomical), the full mesh set lives here
  under `models/MPL/meshes/` — consistent with the "device hardware stays in assist_sim"
  policy.

## Diff vs the original
- **Kept** the SALLY bimanual config (`arms_chain` + `simpleHand{R,L}`); **pruned** the
  unused legacy variants (full `handR/handL`, single `left_arm`/`right_arm`, myochallenge,
  the `left_arm` scene) so every shipped file is valid.
- **Removed** the deprecated `convexhull="false"` compiler attribute (MuJoCo 2.x → 3.x).
- **Renamed** the directory `MPL_sim` → `MPL` and rewrote its internal `../MPL_sim/...`
  self-references to directory-relative paths.
- No geometry, joints, actuators, or meshes were otherwise altered.

## Files
- `scenes/sally.xml` — the bimanual env (entry point for `build_mpl`).
- `scenes/basic_scene.xml` + `scenes/textures/` — floor/lights/skybox.
- `assets/` — `arms_{assets,chain}.xml`, `simpleHand{R,L}_{assets,chain}.xml`.
- `meshes/` — MPL robot meshes.

## Follow-ups (not done)
- No keyframes shipped in the original SALLY scene (none carried).
- If a *human–prosthetic* interaction env is wanted later, attach MPL to a myo_sim
  residual-limb MSK — out of scope for this straight relocation.
