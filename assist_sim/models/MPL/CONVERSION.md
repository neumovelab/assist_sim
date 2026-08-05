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

---

# Bionic bimanual (MyoChallenge) — assist_sim conversion

A second MPL-family env: the MyoChallenge **"bionic bimanual"** manipulation task. Build
it with `assist_sim.upper_body.build_bionic_bimanual()` → `(MjModel, MjData)` (or
`build_bionic_bimanual_spec()` for the uncompiled `MjSpec`).

## What it is
A biological **RIGHT arm** (myo_sim human, standing on a base pedestal) faces an MPL
**LEFT** prosthetic arm across a YCB **gelatin box** (`manip_object`, freejoint) that
starts on a `start` pillar and is to be moved to a `goal` pillar (two `mocap="true"`
cylinders); a touch sensor sits on the object. Original:
`myosuite/envs/myo/assets/arm/myoarm_bionic_bimanual.xml`.

## How the pieces are sourced
- **Human right arm + torso + legs — from the CURRENT myo_sim package**, never housed here.
  The original included a fork `myoarm_body.xml` whose `thorax` body owned the
  pectoralis/latissimus muscle **origins**. The current myo_sim (2026-06 refactor, *"moved
  chest ownership to myotorso"*) removed those from the arm, so `myoarm_r` **cannot
  self-assemble** — the muscle/tendon files reference thorax sites (`PECM2/3-P3/P4_r`,
  `LAT1-3-P4..P6_r`, `Thorax_ellipsoid_PECM2/3`) that now live in `myotorso`'s `chest_r`.
  The human is therefore composed as a **passive anatomical torso + right arm** (the same
  `MjSpec`-attach path the Wheelchair/AuxivoLiftsuit envs use), and the arm is rigidly
  repositioned so `humerus_r` lands at the original arm world pose — registering it to the
  fixed-world prosthesis / object / pillars exactly.
- **Legs — rigid standing scaffold.** Both myo_sim legs are attached muscle-less and frozen
  in the standing (qpos0) pose: leg actuators/tendons/**sensors** and every leg joint (+ its
  knee couplings) are deleted, so the figure has anatomical legs to stand on the base but
  **no leg DOF or actuators** (the original's lower body was a decorative shell). This keeps
  the exact `nu`/`nq`/`nsensor` match and adds only rigid leg bodies for grounding.
- **MPL left prosthesis — restored into `assets/`** (`left_arm_assets.xml`,
  `left_arm_chain_myochallenge.xml`, `handL_assets.xml`, `handL_chain.xml`) from the fork's
  `MPL_sim/assets/`. Meshes reuse the existing `meshes/mplL/` set via the `__MPLMESH__`
  token. Deprecated `convexhull="false"` stripped; per-file `meshdir` removed; the nested
  `handL_chain` include rewritten to `../assets/handL_chain.xml` (MuJoCo resolves every
  include relative to the main model file).
- **YCB gelatin box — relocated into `models/YCB/`** (`meshes/009_gelatin_box.msh`,
  `textures/009_gelatin_box.png`, `defaults_ycb.xml`, `assets_009_gelatin_box.xml`,
  `body_009_gelatin_box.xml`) from the fork's `YCB_sim/`, mesh/texture paths tokenized to
  `__YCB__`.
- **Base pedestal + grounding.** The figure stands on a static base pedestal sized to the
  **original myosuite scene pedestal** (radius 1.05, half-height 0.205) — the turntable-style
  UV-wrapped, myosuite-branded cylinder (`scenes/meshes/pedestal_side.obj` +
  `scenes/textures/pedestal_myosuite.png`). At build time the pedestal top is seated exactly
  under the feet, and the `start`/`goal` pillars are **trimmed** to run from the pedestal top
  up to their original top height (the object rest height — hence every keyframe — is
  unchanged). No ground plane: everything rests on the fixed pedestal.
- **Contact / `multiccd`.** The original enables `multiccd` (multi-point convex contacts) so
  the box rests stably on a pillar (5 contact points, not 1). `MjSpec.attach` drops the
  scene's `<flag>`, so the builder re-asserts `multiccd` (and the 2 ms timestep) on the
  composite. Without it the single box–cylinder contact is unstable and the object falls
  through under manipulation.
- **Keyframes — transcribed by joint NAME** (not positionally): the 4 original keyframes
  (`bionic_bimanual_keyframes.json`) are keyed by the *original* joint names; the builder
  maps each onto this build (human arm joints gain `_r`; `prosthesis/*` and
  `manip_object/freejoint` match exactly) and writes the values onto the composed qpos
  layout. All 4 reproduce the original object / prosthesis / hand-body world poses to float
  precision.

## Files
- `scenes/bionic_bimanual.xml` — the **static half** (base pedestal + MPL prosthesis + YCB
  object + pillars + touch sensor + scene materials), a self-contained token scene
  (`__MPLMESH__`, `__YCB__`, `__MYOSIM__`) that `<include>`s the restored MPL/YCB fragments.
  The builder inlines the includes, substitutes the dir tokens, and attaches this onto the
  composed human.
- `scenes/bionic_bimanual_keyframes.json` — name-based keyframes (see above).
- `scenes/meshes/pedestal_side.obj` + `scenes/textures/pedestal_myosuite.png` — the base
  pedestal (myosuite-branded, R=1.05).
- `scenes/textures/stone1.png` — the original `tabletop` material's texture (decorative;
  the `matwood` texture is sourced live from myo_sim via `__MYOSIM__/scene/floor0.png`).

## Faithfulness (relocated vs the original baseline)
- **Matches exactly:** `nu`=80, `nq`=71, `nkey`=4, `nsensor`=1; the **actuator name set is
  identical** (63 muscles + 17 prosthesis actuators); the joint name set matches after
  normalizing the myo_sim `_r` arm suffix; the prosthesis + object/pillar body set matches.
  `multiccd` on, 2 ms timestep. Object / MPL palm / distal-hand world poses match the
  baseline to **0.0 mm** across all 4 keyframes, and the box rests on the pillar (5 contacts)
  exactly as in the baseline.
- **Intended differences (Option A + grounding):** `nbody` 73→101, `ngeom` 196→302,
  `nmesh` 77→88 — the current myo_sim brings a real **passive anatomical torso + rigid legs**
  (spine/head/leg bodies: `sacrum`, `lumbar1-5`, `Abdomen`, `torso`, `chest_r`,
  `cervical_spine`, `neck`, `head*`, `pelvis`, `femur/tibia/talus/calcn/toes/patella_{l,r}`)
  plus the base `pedestal`, in place of the original's single **decorative `body_nohand`
  shell** (absent from the current myo_sim package). Human body/joint names carry `_r`; a few
  are renamed by the refactor (`thorax`→`chest_r`, `proxph2`→`2proxph_r`, `full_body`→
  `Full Body`). The torso + legs are passive/rigid (muscle-less, joints removed) so they add
  **no** actuators/DOF — hence the exact `nu`/`nq`/`nsensor` match.

## Build / export
```python
from assist_sim.upper_body import build_bionic_bimanual, build_bionic_bimanual_spec, export_upper_body_xml
model, data = build_bionic_bimanual()                       # (MjModel, MjData)
export_upper_body_xml(build_bionic_bimanual_spec(), "bionic_bimanual.xml")  # standalone, reloadable
```

## Follow-ups (not done)
- If a *residual-limb* human–prosthetic interface is wanted (prosthesis socketed to an
  amputated MSK rather than a full biological arm facing it), that is a separate model.
