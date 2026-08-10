# Code review — upper-body collaboration environments (`git diff 6caa7d0..HEAD`)

## Summary

The change adds four upper-body collaboration environments to `assist_sim`: a
composed **Wheelchair** (myo_sim `myoarms` + chair, seated rigid legs, propulsion
keyframes), a standalone **MPL** robot (`sally.xml`, loaded directly), a composed
**AuxivoLiftsuit** back-exosuit welded onto the muscled `myotorso`, and a composed
**bionic-bimanual** MyoChallenge manipulation task (biological right arm + MPL left
prosthesis + YCB object on a pedestal). New Python is `assist_sim/upper_body.py`
(634 lines, all builders + helpers), two touched export helpers in
`assist_sim/utils.py` (`_reassert_named_geom_contacts`, site-material handling in
`_strip_orphan_scene_assets`), and `tests/test_upper_body_export.py`. The rest is
device meshes/scene XML, docs (`README.md`, `docs/available-models.md`,
`docs/collaboration-environments.md`), and two CI hotfixes (`pytest.importorskip`
+ a `ruff format` pass). Scope reviewed read-only; the bulk of the diff is binary
mesh assets, not reviewed.

Note on verifiability: `myo_sim` is **not importable in this environment**, so any
finding that depends on the runtime shape of the myo_sim legs/arms specs is marked
**PLAUSIBLE** and moved to Open questions rather than asserted.

Counts: **1 major, 4 minor, 3 nit, 3 open questions.**

---

## Major

### 1. `_add_ground` places the floor at geom *centers*, not the lowest surface — wheelchair spawns interpenetrating the ground
- **file:line** `assist_sim/upper_body.py:315`
- **Claim** `floor_z = float(td.geom_xpos[:, 2].min())` uses geom **center** z, but the sibling helper `_lowest_geom_z` (`:461-473`) correctly computes the lowest **AABB-corner** (surface) z. Every geom extends below its center, so the plane is set too high by roughly the smallest wheel radius.
- **Why it matters** The wheelchair rear-wheel collision cylinders have radius `0.3048 m` (`Wheelchair/assets/wheelchair_chain.xml:22,54`) and the casters are also sizeable. With the ground at the lowest geom *center*, the wheels’ lower surfaces start below the plane; because the whole rig gets a freejoint (`_attach_chair`, `:284`), at `t=0` the model is deeply interpenetrating the ground → a large contact impulse / “pop” or blow-up on the first steps, contradicting the docstring “ground plane at the resting wheel height.”
- **Fix** Reuse `_lowest_geom_z(human)` (already written and used for bionic) instead of `geom_xpos[:,2].min()`, so the plane sits exactly under the lowest wheel surface.
- **Status** Verified against the code (two helpers compute different quantities; wheel radii read from the chair XML). Physics magnitude inferred, not simulated. Not covered by any test (only structural counts are asserted for the wheelchair).

---

## Minor

### 2. `upper_body.py` imports `myo_sim` at module top — breaks the lazy convention and blocks `build_mpl()` (which needs no myo_sim)
- **file:line** `assist_sim/upper_body.py:23`, `:25`, and `:46`
- **Claim** `import myo_sim`, `from myo_sim.build.compose import (...)`, and `_MYOSIM_MODELS = str(_files("myo_sim").joinpath("models"))` all execute at import. `loading.py:29` and `registry.py:125,167` deliberately import `myo_sim` *inside* functions.
- **Why it matters** Importing `assist_sim.upper_body` at all fails without `myo_sim` installed. `build_mpl()` is documented as “a self-contained robotic model (no myo_sim human)” and loads `sally.xml` directly, yet it is unreachable without myo_sim. This is what caused the CI test-collection error (patched at `tests/test_upper_body_export.py:22` with `pytest.importorskip("myo_sim")` — a correct symptom fix, but the module stays coupled). `assist_sim/__init__.py` does *not* import `upper_body`, so `import assist_sim` itself is unaffected.
- **Fix** Move `import myo_sim` / the `myo_sim.build.compose` import inside the composed builders (`_build_human`, `_freeze_legs_*`, `build_auxivo_liftsuit_spec`), and defer `_MYOSIM_MODELS` resolution into `_bionic_scene_spec`. Then `build_mpl` works myo_sim-free, matching the library convention.

### 3. Reload-contact preservation and keyframe fidelity are effectively untested in CI
- **file:line** `tests/test_upper_body_export.py:62-75`, `:104-110`, `:112-113`
- **Claim** `test_upper_body_export_reloads_matching_live` only asserts `(nu, ntendon, neq, nbody)` equality — it never checks contacts. The reason `_reassert_named_geom_contacts` exists (box + pillars must still collide *after reload*) is exercised only on the **live** build (`:104-110` simulate + assert object doesn’t fall through), not on the reloaded XML. The keyframe-pose faithfulness comparison is gated behind `_BIONIC_BASELINE.exists()` and skips when absent (always, in CI).
- **Why it matters** A regression that drops contype/conaffinity on reload (the exact bug the helper guards) would pass the suite. Likewise, wrong keyframe joint-name mapping (see Open question 2) would go uncaught.
- **Fix** In the reload test, also `mj_step` the reloaded bionic model from its `start` keyframe and assert the object stays above the pillar (mirror the live check), and add a small self-contained keyframe sanity check that doesn’t require the external baseline.

### 4. `_strip_scene_decor` is a weaker duplicate of the existing `strip_myosuite_scene_spec`
- **file:line** `assist_sim/upper_body.py:182-191` vs `assist_sim/utils.py:158-182`
- **Claim** `_strip_scene_decor` deletes only worldbody geoms + lights. `strip_myosuite_scene_spec` (already in the repo) deletes geoms + lights + **cameras** and then prunes now-orphaned meshes (backdrop/logo). `build_auxivo_liftsuit_spec` builds from `myo_sim.build_spec("myotorso")`, which ships the full myosuite scene, so the auxivo model retains the scene **cameras** and orphaned backdrop/logo **meshes** (`_strip_orphan_scene_assets` prunes only textures/materials, not meshes).
- **Why it matters** Contradicts the “model-only” intent and leaves dead scene assets in the export (mesh bloat, stray cameras). Not a correctness break (round-trip test only compares live-vs-reloaded, which both carry the residue), but it is inconsistent with the repo’s own scene-strip helper.
- **Fix** Reuse `strip_myosuite_scene_spec(spec)` (or extend `_strip_scene_decor` to also drop cameras + orphan meshes).

### 5. `_reassert_named_geom_contacts` recompiles the spec and swallows failures silently
- **file:line** `assist_sim/utils.py:430-433`
- **Claim** The helper calls `spec.compile()` again (the builder and `spec.to_xml()` already compiled), and on any exception does `return` with no log. It also, by design, does not fix **unnamed** geoms that resolve to 1/1 under a root `main` default of 0/0.
- **Why it matters** Extra full compile on every export (all lower-limb exports too, since it’s called unconditionally at `utils.py:70`) — minor cost. The silent `except Exception: return` means a genuinely broken spec produces an export with *unrepaired* contacts and no diagnostic. The unnamed-geom gap is acknowledged in the docstring; it is currently safe only because the affected geoms (pillars, box) happen to be named.
- **Fix** Reuse the already-compiled model if the caller can pass it (avoid the third compile); at minimum don’t swallow the exception without a warning. Leave the named-only scope but note it explicitly.

---

## Nit

### 6. Hardcoded absolute Windows baseline path committed in a test
- **file:line** `tests/test_upper_body_export.py:48`
- **Claim** `_BIONIC_BASELINE = Path(os.environ.get("BIONIC_BASELINE_XML", r"C:\Users\calde\Work\compile_check\bionic_bimanual.xml"))` bakes a user-specific path as the default.
- **Why it matters** Harmless at runtime (guarded by `.exists()` skip + env override) but it’s personal, non-portable, and clutters the repo. **Formatter won’t touch this.**
- **Fix** Default to `None` (skip when unset) or a repo-relative fixtures path.

### 7. Pedestal geometry magic constant duplicated between XML and Python
- **file:line** `assist_sim/upper_body.py:58` (`_PED_TOP_FROM_CENTER = 0.205 + 2 * 0.006`)
- **Claim** The `0.205` half-height and `0.006` cap thickness re-encode the pedestal sizes hardcoded in `MPL/scenes/bionic_bimanual.xml:54-55` (`size="1.053 0.006"`, `pos="0 0 0.211"`). The Python value (0.217) matches the XML (0.211 + 0.006) today, but the two can silently drift.
- **Why it matters** If the pedestal mesh/cap sizes ever change in the XML, `_ground_bionic` would seat the feet at the wrong height with no error. Correctness is fine as-is; this is maintainability.
- **Fix** Derive the top-from-center offset from the compiled `ped_top` geom, or add a comment cross-linking the XML source of `0.205`/`0.006`.

### 8. Export path compiles the spec 2–3 times
- **file:line** `assist_sim/upper_body.py:623` + `assist_sim/utils.py:50, 431`
- **Claim** `export_upper_body_xml` calls `spec.compile()`, then `export_combined_xml` calls `spec.to_xml()` (compiles) and `_reassert_named_geom_contacts` calls `spec.compile()` again.
- **Why it matters** Pure inefficiency on large composed specs; no correctness impact.
- **Fix** Compile once and thread the model through.

---

## Open questions (PLAUSIBLE — need a myo_sim-enabled run to confirm)

1. **`_freeze_legs_seated` does not delete leg sensors, unlike `_freeze_legs_standing`.**
   `_freeze_legs_standing` (`upper_body.py:443-448`) explicitly deletes the legs’
   proprioceptive sensors (its comment: “the legs ship proprioceptive sensors”),
   but `_freeze_legs_seated` (`:214-259`) deletes actuators + tendons + **joints**
   yet leaves sensors in place. If those sensors are joint-referencing
   (jointpos/vel/actuatorfrc) on the leg joints that `_freeze_legs_seated` then
   deletes (`:254-256`), the wheelchair spec would either fail to compile with a
   dangling-sensor reference, or silently retain stale sensors pointing at
   baked-out DOF. The wheelchair evidently compiles for the author, so either the
   sensors don’t target the deleted joints or `MjSpec.delete(joint)` cascades —
   needs confirmation. If it’s the latter, add the sensor-deletion loop to
   `_freeze_legs_seated` for consistency and to keep the export clean.

2. **`_arm_joint` is asymmetric for the left arm.**
   `upper_body.py:173-179`: the right branch tries the bare name then `name+"_r"`;
   the left branch tries **only** `name+"_l"`. The keyframe dicts use side-neutral
   names with embedded DOF indices, e.g. `sternoclavicular_r2` (the `_r2`/`_r3` are
   axis indices, not “right”). If `load_mirrored_left_arm_spec` renames by an `_r`→
   `_l` substring rule, `sternoclavicular_r2` becomes `sternoclavicular_l2`, but
   `_arm_joint` looks up `sternoclavicular_r2_l` → `None` → that joint’s keyframe
   value is silently skipped, giving a wrong left-arm pose for `arms="left"` and
   `"both"`. If the mirror instead appends an `_l` suffix, the current code is
   correct. Unverifiable without myo_sim; keyframe correctness is not asserted in
   CI (see Minor 3), so this would not be caught. Worth confirming the mirror
   naming and, if it’s substring-based, mapping left joints the same way the right
   branch handles the bare/embedded case.

3. **Auxivo weld relpose/torquescale defaults.**
   `build_auxivo_liftsuit_spec` sets only `eq.data[:3] = anchor` (`:595`) and
   relies on the comment’s claim that “relpose is left to auto-solve from the
   placed rest pose.” This is correct **iff** a fresh `spec.add_equality()` leaves
   the relpose quaternion at the `(0,0,0,0)` auto-compute sentinel and torquescale
   at `1`. If the spec defaults zero the whole `data` array, torquescale would be
   `0` (weld applies no orientation coupling) — a silent weakening of the weld.
   Could not verify the `add_equality` defaults without running mujoco; confirm the
   compiled `eq_data` has the expected relpose + torquescale.

---

## Verified-correct (no action needed)

- `_inline_includes` + `_bionic_scene_spec` token replacement: the nested
  `<include file="../assets/handL_chain.xml"/>` inside
  `left_arm_chain_myochallenge.xml` is written relative to the **scenes** (main)
  dir, matching the recursion’s fixed `base_dir` — MuJoCo’s main-file include
  semantics hold here. `sally.xml`’s chain uses real relative meshdir paths (no
  `__…__` tokens), so `build_mpl`’s direct `from_xml_path` load is consistent.
- `_ground_bionic` pillar/pedestal math (`:476-489`): keeps each pillar top at its
  original world z (object rest height unchanged) while seating the bottom at
  `feet_low`, and seats the pedestal top cap exactly at `feet_low`. Correct.
- `_align_bionic_arm` rigid map (`:416-433`): `new_root = (mq·rq, mq*rp + mp)` is
  the correct application of `M = (q_target·conj(q_hum), p_target − R·p_hum)` to
  the root pose.
- `utils.py` `_strip_orphan_scene_assets` site-material change: only *adds* to the
  referenced set (direct sites + default-class sites), so it can only keep more —
  no over-strip regression for the lower-limb exports; it fixes a real
  missing-material reload break for the MPL touch sites.
- `_reassert_named_geom_contacts` for named geoms writes the actual compiled
  contype/conaffinity/condim, which is behavior-preserving for existing lower-limb
  exports (see Minor 5 for the caveats).
