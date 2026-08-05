# AuxivoLiftsuit — assist_sim conversion

How this environment maps to the original collaborator model. Final product, not the
porting process. Build it with `assist_sim.upper_body.build_auxivo_liftsuit()` →
`(MjModel, MjData)`.

## Original (collaborator env, retired myosuite fork)
`myosuite/simhive/myo_sim/torso/myotorso_exosuit.xml` (`MyoExoTorso`) — a **passive
back-exosuit** (Auxivo Liftsuit style) worn on a myo torso: an upper-back panel, a waist
belt, and lower leg straps, coupled to the body by two **welds** (`torso`, `lumbar4`) and
four **spring tendons**. MyoAssist-specific — it has no upstream myo_sim equivalent.

## assist_sim version (final)
- Relocated to `models/AuxivoLiftsuit/`. The **human torso, scene, and head are pulled
  from the installed myo_sim package at load** (via `build_auxivo_liftsuit`), so no
  anatomical assets are housed here; only the **three exosuit meshes** live in `mesh/`.
- Loaded as `(MjModel, MjData)`: nbody 19, 4 spring tendons, 2 welds (17 equalities incl.
  torso couplers), passive (nu 0) — matching the original (a passive exosuit on a passive
  torso, for studying the suit's assistance mechanics).
- The exosuit's geometry/welds/tendons are preserved **verbatim** relative to the torso,
  so the original fit/alignment carries over unchanged.

## Diff vs the original
- Include paths (`scene/myosuite_scene`, `torso/assets/myotorso_{assets,chain}`,
  `head/assets/myohead_simple_assets`) and `meshdir` are **repointed to the myo_sim
  package** at load (tokens `__MYOSIM__`); the three exo meshes to the local `mesh/` dir
  (`__EXO__`). No hard-coded absolute paths.
- **Restored** the `myoBack_wrap` (tendon-wrap cylinder) and `sidesite` default classes in
  this file — they were dropped from the current myo_sim `myotorso_assets.xml`, so the
  exosuit's spring-tendon wrap geom needs them locally.
- No geometry, welds, tendons, or meshes were otherwise altered.

## Files
- `auxivo_liftsuit.xml` — the exosuit model (includes tokenized to resolve at load).
- `mesh/` — the 3 exosuit meshes (`lower_exo_belt`, `lower_exo_legs`, `upper_exo`).

## Follow-ups (not done)
- The original ships no keyframes (none carried).
- The torso is passive here (as authored). Composing the exosuit onto the *active*
  `myotorso` (trunk muscles) for a controlled trunk-assist task is a possible extension.
