# AuxivoLiftsuit — assist_sim conversion

How this environment maps to the original collaborator model. Final product, not the
porting process. Build it with `assist_sim.upper_body.build_auxivo_liftsuit()` →
`(MjModel, MjData)` (or `build_auxivo_liftsuit_spec()` for the uncompiled `MjSpec`).

## Original (collaborator env, retired myosuite fork)
`myosuite/simhive/myo_sim/torso/myotorso_exosuit.xml` (`MyoExoTorso`) — an Auxivo
Liftsuit–style back-exosuit worn on the **muscled** myo torso: an upper-back panel
(`exo_torso`), a waist belt (`exo_lumbar4`), and lower leg straps (`exo_pelvis`),
coupled to the trunk by two **body welds** (`torso`↔`exo_torso` anchored at
`-0.1 0.2 0`, `lumbar4`↔`exo_lumbar4`) and driven by four **spring tendons** (two
shoulder→leg springs routed over a back wrap cylinder, two passive belt→leg straps).
The trunk itself is the full muscled `myotorso` (nu 210, ntendon 210). MyoAssist-specific
— the exosuit has no upstream myo_sim equivalent.

## assist_sim version (final)
- Relocated to `models/AuxivoLiftsuit/`. Built as a **runtime composition**: the human
  is the muscled myo_sim `myotorso` pulled from the installed package
  (`myo_sim.load_spec("myotorso")`), so no anatomical assets are housed here — only the
  **three exosuit meshes** live in `mesh/`.
- The exosuit hardware is a **device fragment** (`auxivo_liftsuit.xml`: the three exo
  bodies + wrap geom + sites + four spring tendons) attached onto the torso at load, then
  coupled with the two original body welds (anchors preserved).
- Compiles to nu 210, ntendon 214 (210 muscle + 4 exo spring tendons), neq 17 (spine
  couplers + the 2 exo welds) — **identical actuator/tendon/equality counts to the
  original**. The trunk is active (muscled) with the passive spring exosuit worn over it.

## Faithfulness (how it was verified)
Rather than eyeballing renders, the port was checked by **diffing against the compiled
original**:
- **Rest pose** — each exo body's pose *relative to its trunk segment* matches the
  original to `dpos = 0`, `dquat = 0` (`exo_torso|torso`, `exo_lumbar4|lumbar4`,
  `exo_pelvis|torso`).
- **Dynamics** — 2 s of free settling reproduces the original's weld drift
  (exo_torso ≈ 0.010 m, exo_lumbar4 ≈ 0.0002 m).
- **Counts** — nu / ntendon / neq match exactly.

## Diff vs the original
- **Placement.** `myo_sim.load_spec("myotorso")` orients the `torso` body ~90° from the
  original env's authoring frame (it nests the torso under a `Full Body` wrapper). The
  suit is therefore placed by the **rigid map** taking the original authoring torso pose
  (`_AUXIVO_AUTHOR_TORSO_POS/QUAT`) to this build's torso pose, applied as the attach
  frame — so every exo→trunk relative pose is reproduced exactly (see above). The exo
  bodies' own poses, geoms, sites, welds, tendons, and mesh scales are otherwise
  **verbatim** from the original.
- **Restored classes.** `myoBack_wrap` (tendon-wrap cylinder) and `sidesite` default
  classes are restored locally in `auxivo_liftsuit.xml` — they were dropped from the
  current myo_sim `myotorso_assets.xml`, so the exosuit's spring-tendon wrap geom needs
  them. The exo geoms use their explicit dark rgba (the original also tagged them with the
  trunk skin material `mat_myotorso`, which is cosmetic and not carried).
- **Weld anchors.** Set explicitly (`-0.1 0.2 0` and `0 0 0`) because MjSpec-created
  equalities do not default the anchor to zero; the relpose is auto-solved from the placed
  rest pose (matching the original's omitted-relpose behavior).
- **Angle units.** The fragment declares `<compiler angle="radian"/>` — its body/geom
  eulers are radians (as in the original). Without it, MjSpec defaults to degrees and the
  mesh panels rotate ~1.57° instead of 90° (a "splayed open" suit), while site positions
  (no euler) still match — so the guard is a geom-*orientation* check, not position.
- No exosuit geometry, welds, tendons, or meshes were otherwise altered.

## Export
`build_auxivo_liftsuit_spec()` returns the composed `MjSpec`; serialize it to a
standalone, reloadable XML with `assist_sim.upper_body.export_upper_body_xml(spec, path)`
(routes through `utils.export_combined_xml`, which flattens the merged defaults and
rewrites mesh paths). A raw `spec.to_xml()` will *not* reload. The env is model-only
(no scene/lighting) — a downstream scene/terrain supplies those.

## Files
- `auxivo_liftsuit.xml` — the exosuit device fragment (attached at load; `__EXO__` mesh token).
- `mesh/` — the 3 exosuit meshes (`lower_exo_belt`, `lower_exo_legs`, `upper_exo`).

## Follow-ups (not done)
- The original ships no keyframes (none carried).
