# Code review: upper-body collaboration environments (`git diff 6caa7d0..HEAD`)

## Summary

The change adds four upper-body collaboration environments to `assist_sim`. The
first is a composed **Wheelchair** (the myo_sim `myoarms` model and a chair,
with rigid seated legs and propulsion keyframes). The second is a standalone
**MPL** robot (`sally.xml`, which the code loads directly). The third is a
composed **AuxivoLiftsuit** back exosuit, which a weld attaches to the muscled
`myotorso`. The fourth is a composed **bionic-bimanual** MyoChallenge
manipulation task (a biological right arm, an MPL left prosthesis and a YCB
object on a pedestal).

The new Python file is `assist_sim/upper_body.py` (634 lines, with all the
builders and the helpers). The change also touches two export helpers in
`assist_sim/utils.py` (`_reassert_named_geom_contacts` and the site-material
code in `_strip_orphan_scene_assets`), and it adds
`tests/test_upper_body_export.py`. The remainder is device meshes, scene XML,
docs (`README.md`, `docs/available-models.md`,
`docs/collaboration-environments.md`) and two CI corrections
(`pytest.importorskip` and a `ruff format` pass). This review is read-only. Most
of the diff is binary mesh assets, which this review does not cover.

Note on verifiability: this environment cannot import `myo_sim`. Thus this
review marks each finding that depends on the runtime shape of the myo_sim legs
spec or arms spec as **PLAUSIBLE**. It puts those findings in Open questions
instead of asserting them.

Counts: **1 major, 4 minor, 3 nit, 3 open questions.**

---

## Major

### 1. `_add_ground` places the floor at the geom *centers*, not at the lowest surface: the wheelchair starts inside the ground
- **file:line** `assist_sim/upper_body.py:315`
- **Claim** `floor_z = float(td.geom_xpos[:, 2].min())` uses the **center** z of each geom. The related helper `_lowest_geom_z` (`:461-473`) correctly computes the lowest corner of the axis-aligned bounding box (AABB), which is the surface z. Every geom extends below its center, thus the code puts the plane too high, by approximately the smallest wheel radius.
- **Why it matters** The rear-wheel collision cylinders of the wheelchair have a radius of `0.3048 m` (`Wheelchair/assets/wheelchair_chain.xml:22,54`), and the casters are also large. With the ground at the lowest geom *center*, the lower surfaces of the wheels start below the plane. `_attach_chair` (`:284`) gives the full assembly a freejoint. Thus at `t=0` the model has a deep intersection with the ground. The result is a large contact impulse and an unstable simulation on the first steps. This contradicts the docstring, which says “ground plane at the resting wheel height”.
- **Fix** Use `_lowest_geom_z(human)` instead of `geom_xpos[:,2].min()`. That helper is already written, and the bionic builder uses it. Then the plane sits exactly below the lowest wheel surface.
- **Status** Verified against the code: the two helpers compute different quantities, and the wheel radii come from the chair XML. The magnitude of the physics effect is an inference, because this review ran no simulation. No test covers this behavior, because the wheelchair tests assert only the structural counts.

---

## Minor

### 2. `upper_body.py` imports `myo_sim` at the top of the module: this breaks the lazy convention and blocks `build_mpl()`, which needs no myo_sim
- **file:line** `assist_sim/upper_body.py:23`, `:25`, and `:46`
- **Claim** `import myo_sim`, `from myo_sim.build.compose import (...)`, and `_MYOSIM_MODELS = str(_files("myo_sim").joinpath("models"))` all run at import time. `loading.py:29` and `registry.py:125,167` deliberately import `myo_sim` *inside* functions.
- **Why it matters** An import of `assist_sim.upper_body` fails if `myo_sim` is absent. The documentation calls `build_mpl()` “a self-contained robotic model (no myo_sim human)”, and the function loads `sally.xml` directly, but the function stays unreachable without myo_sim. This caused the CI test-collection error. The patch at `tests/test_upper_body_export.py:22` adds `pytest.importorskip("myo_sim")`, which corrects the symptom, but the module keeps the coupling. `assist_sim/__init__.py` does *not* import `upper_body`, thus `import assist_sim` itself continues to operate.
- **Fix** Move `import myo_sim` and the `myo_sim.build.compose` import into the composed builders (`_build_human`, `_freeze_legs_*` and `build_auxivo_liftsuit_spec`). Resolve `_MYOSIM_MODELS` in `_bionic_scene_spec` instead. Then `build_mpl` operates without myo_sim, which matches the convention of the library.

### 3. The CI tests omit the reload of the contacts and the fidelity of the keyframes
- **file:line** `tests/test_upper_body_export.py:62-75`, `:104-110`, `:112-113`
- **Claim** `test_upper_body_export_reloads_matching_live` asserts only the equality of `(nu, ntendon, neq, nbody)`, and it omits the contacts. `_reassert_named_geom_contacts` exists because the box and the pillars must still collide *after the reload*. Only the **live** build exercises that behavior (`:104-110` simulates and asserts that the object does not fall through). The reloaded XML has no such check. `_BIONIC_BASELINE.exists()` gates the comparison of the keyframe pose, and the test skips when the baseline is absent, which is always true in CI.
- **Why it matters** A regression that removes contype or conaffinity on reload is the exact problem that the helper prevents, but the suite would still pass. An incorrect map of the keyframe joint names (see Open question 2) would also stay undetected.
- **Fix** In the reload test, also step the reloaded bionic model with `mj_step` from its `start` keyframe. Then assert that the object stays above the pillar, which mirrors the live check. Then add a small keyframe check that needs no external baseline.

### 4. `_strip_scene_decor` is a less complete duplicate of the existing `strip_myosuite_scene_spec`
- **file:line** `assist_sim/upper_body.py:182-191` vs `assist_sim/utils.py:158-182`
- **Claim** `_strip_scene_decor` removes only the worldbody geoms and the lights. `strip_myosuite_scene_spec` is already in the repo. It removes the geoms, the lights and the **cameras**, then it removes the meshes that become orphans (the backdrop and the logo). `build_auxivo_liftsuit_spec` builds from `myo_sim.build_spec("myotorso")`, which supplies the full myosuite scene. Thus the auxivo model keeps the scene **cameras** and the orphan backdrop and logo **meshes**, because `_strip_orphan_scene_assets` removes only the textures and the materials.
- **Why it matters** This contradicts the “model-only” intent, and it keeps unused scene assets in the export (extra mesh data and unwanted cameras). It is not a correctness failure, because the round-trip test compares the live model with the reloaded model, and both keep the same residue. But it does not agree with the scene-removal helper of the repo.
- **Fix** Use `strip_myosuite_scene_spec(spec)`. As an alternative, extend `_strip_scene_decor` to also remove the cameras and the orphan meshes.

### 5. `_reassert_named_geom_contacts` compiles the spec again and discards failures with no message
- **file:line** `assist_sim/utils.py:430-433`
- **Claim** The helper calls `spec.compile()` again, although the builder and `spec.to_xml()` already compiled the spec. On any exception, it returns with no log entry. By design, it also does not correct the **unnamed** geoms that resolve to 1/1 under a root `main` default of 0/0.
- **Why it matters** Each export does one more full compile, because `utils.py:70` calls the helper unconditionally. This includes all the lower-limb exports, at a minor cost. The `except Exception: return` writes no message, thus a defective spec gives an export with *uncorrected* contacts and no diagnostic. The docstring records the gap for the unnamed geoms. The behavior is safe now only because the affected geoms (the pillars and the box) have names.
- **Fix** Use the model that is already compiled, if the caller can pass it, to prevent the third compile. As a minimum, write a warning when an exception occurs. Keep the named-only scope, but state it explicitly.

---

## Nit

### 6. A test contains a hardcoded absolute Windows path for the baseline
- **file:line** `tests/test_upper_body_export.py:48`
- **Claim** `_BIONIC_BASELINE = Path(os.environ.get("BIONIC_BASELINE_XML", r"C:\Users\calde\Work\compile_check\bionic_bimanual.xml"))` sets a user-specific path as the default.
- **Why it matters** The effect at runtime is safe, because the `.exists()` skip and the environment variable override protect the test. But the path is personal, it is not portable, and it adds clutter to the repo. **The formatter does not correct this.**
- **Fix** Use `None` as the default, and skip the test when the variable is unset. As an alternative, use a fixtures path that is relative to the repo.

### 7. The XML and the Python code both contain the pedestal dimensions
- **file:line** `assist_sim/upper_body.py:58` (`_PED_TOP_FROM_CENTER = 0.205 + 2 * 0.006`)
- **Claim** The `0.205` half-height and the `0.006` cap thickness repeat the pedestal sizes that `MPL/scenes/bionic_bimanual.xml:54-55` hardcodes (`size="1.053 0.006"`, `pos="0 0 0.211"`). The Python value (0.217) agrees with the XML value (0.211 + 0.006) today, but the two can become different, with no message.
- **Why it matters** If the mesh sizes or the cap sizes of the pedestal change in the XML, `_ground_bionic` puts the feet at the wrong height, with no error. The code is correct as it is. This is a maintenance risk.
- **Fix** Calculate the offset from the center to the top from the compiled `ped_top` geom. As an alternative, add a comment that gives the XML source of `0.205` and `0.006`.

### 8. The export path compiles the spec 2 to 3 times
- **file:line** `assist_sim/upper_body.py:623` + `assist_sim/utils.py:50, 431`
- **Claim** `export_upper_body_xml` calls `spec.compile()`. Then `export_combined_xml` calls `spec.to_xml()`, which compiles the spec. Then `_reassert_named_geom_contacts` calls `spec.compile()` again.
- **Why it matters** This is only an inefficiency on large composed specs. It has no effect on correctness.
- **Fix** Compile one time. Then pass the compiled model to the other functions.

---

## Open questions (PLAUSIBLE: a run with myo_sim is necessary to confirm them)

1. **`_freeze_legs_seated` keeps the leg sensors, but `_freeze_legs_standing` removes them.**
   `_freeze_legs_standing` (`upper_body.py:443-448`) explicitly removes the
   proprioceptive sensors of the legs; its comment says “the legs ship
   proprioceptive sensors”. `_freeze_legs_seated` (`:214-259`) removes the
   actuators, the tendons and the **joints**, but it keeps the sensors. Those
   sensors can reference joints (jointpos, jointvel or actuatorfrc) on the leg
   joints that `_freeze_legs_seated` then removes (`:254-256`). In that
   condition, the wheelchair spec fails to compile with an unresolved sensor
   reference. As an alternative, it keeps sensors that point at a removed
   degree of freedom (DOF), with no message.

   The wheelchair evidently compiles for the author. Thus the sensors do not
   target the removed joints, or `MjSpec.delete(joint)` cascades to them, and
   this needs confirmation. If the cascade is the reason, add the sensor-removal
   loop to `_freeze_legs_seated`, for consistency and for a clean export.

2. **`_arm_joint` is asymmetric for the left arm.**
   `upper_body.py:173-179`: the right branch tries the bare name, then
   `name+"_r"`, but the left branch tries **only** `name+"_l"`. The keyframe
   dicts use side-neutral names that contain DOF indices, for example
   `sternoclavicular_r2`. In that name, `_r2` and `_r3` are axis indices, and
   they do not mean “right”. If `load_mirrored_left_arm_spec` renames with an
   `_r` to `_l` substring rule, `sternoclavicular_r2` becomes
   `sternoclavicular_l2`. Then `_arm_joint` looks up `sternoclavicular_r2_l`,
   gets `None`, and skips the keyframe value of that joint with no message. The
   result is a wrong left-arm pose for `arms="left"` and `"both"`.

   If the mirror instead appends an `_l` suffix, the current code is correct.
   Confirmation needs myo_sim. CI does not assert the correctness of the
   keyframes (see Minor 3), thus a test would not catch this. Confirm the naming
   of the mirror. If the rule is substring-based, map the left joints in the same
   way as the right branch. That branch handles both the bare name and the
   embedded index.

3. **The Auxivo weld defaults for relpose and torquescale.**
   `build_auxivo_liftsuit_spec` sets only `eq.data[:3] = anchor` (`:595`), and it
   depends on the claim in the comment: “relpose is left to auto-solve from the
   placed rest pose.” This is correct **only if** a fresh `spec.add_equality()`
   leaves the relpose quaternion at the `(0,0,0,0)` auto-compute sentinel and
   torquescale at `1`. If the spec defaults zero the whole `data` array,
   torquescale is `0`, and the weld applies no orientation coupling. That
   weakens the weld, with no message. This review could not verify the defaults
   of `add_equality`, because it could not run mujoco. Confirm that the compiled
   `eq_data` has the expected relpose and torquescale.

---

## Verified-correct (no action needed)

- The token replacement in `_inline_includes` and `_bionic_scene_spec` is
  correct. The nested `<include file="../assets/handL_chain.xml"/>` in
  `left_arm_chain_myochallenge.xml` is relative to the **scenes** (main)
  directory, which matches the fixed `base_dir` of the recursion. The
  main-file include semantics of MuJoCo hold here. The chain of `sally.xml`
  uses real relative meshdir paths, with no `__…__` tokens, thus the direct
  `from_xml_path` load in `build_mpl` is consistent.
- The pillar and pedestal calculation in `_ground_bionic` (`:476-489`) is
  correct. It keeps the top of each pillar at its original world z, so the rest
  height of the object stays the same. It seats the bottom of each pillar at
  `feet_low`. It seats the top cap of the pedestal exactly at `feet_low`.
- The rigid map in `_align_bionic_arm` (`:416-433`) is correct.
  `new_root = (mq·rq, mq*rp + mp)` is the correct application of
  `M = (q_target·conj(q_hum), p_target − R·p_hum)` to the root pose.
- The site-material change in `_strip_orphan_scene_assets` (`utils.py`) only
  *adds* to the referenced set (the direct sites and the default-class sites),
  thus it can only keep more assets. It removes nothing extra from the
  lower-limb exports, and it corrects a real reload failure from a missing
  material for the MPL touch sites.
- For the named geoms, `_reassert_named_geom_contacts` writes the actual
  compiled contype, conaffinity and condim values. This preserves the behavior
  of the existing lower-limb exports. See Minor 5 for the caveats.
