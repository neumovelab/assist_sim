# Device Config Reference

Every device under `models/<DeviceDir>/` carries a YAML config (typically
named `L1config.yaml` or `<variant>_L1config.yaml`). This doc is the schema
reference — every section, every field, with examples.

For walkthroughs see [how-to/add-a-device.md](how-to/add-a-device.md).

## Top-level shape

```yaml
device:                      # required
  name: ...
  model_xml: ...
  compatible_msk: ...        # optional

attachments: ...             # required

# Optional sections, all default to empty:
joint_overrides: ...
actuators: ...
keyframe_overrides: ...
keyframes: ...
body_removals: ...
mesh_replacements: ...
actuator_removals: ...
tendon_removals: ...
tendon_modifications: ...
geom_removals: ...
```

Only `device` and `attachments` are required. Every other section defaults
to empty.

## `device`

```yaml
device:
  name: "DephyExoBoot_L1"
  model_xml: "L1model.xml"
  compatible_msk: ["myoLeg22_2D", "myoLeg26_3D"]   # optional
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `name` | string | yes | Namespace prefix applied to every body / site / mesh / joint / actuator / tendon imported from the device XML. Convention: PascalCase + `_L1` suffix (e.g. `DephyExoBoot_L1`, `OpenSourceLeg_A_L1`). |
| `model_xml` | path | yes | Path to the device's MuJoCo XML, relative to this YAML file. |
| `compatible_msk` | list | no | Restricts which MSKs this device may combine with. Absent → compatible with all. |

## `attachments`

Maps each top-level device body to a parent body in the MSK.

```yaml
attachments:
  - device_body: "exo_1_r"
    parent_body: "tibia_r"
  - device_body: "fanny_pack"
    parent_body: "pelvis"
    pos: [0.0, 0.05, 0.0]      # optional frame offset
    quat: [1, 0, 0, 0]         # optional frame rotation
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `device_body` | string | yes | Name of a top-level body in the device XML. |
| `parent_body` | string | yes | Name of the MSK body the device body attaches under. |
| `pos` | `[x, y, z]` | no | Frame offset on the parent (composes with the device body's authored pos). Use when the device-body's authored pos needs adjustment per attach point. |
| `quat` | `[w, x, y, z]` | no | Frame rotation. Useful when the parent body's frame differs across MSKs (e.g. 22 vs 80 torso). |

Each attachment is implemented as `parent.add_frame(pos, quat).attach_body(device_body, prefix=device.name + "_")`.

### Per-MSK attachments

When a device needs different attachment topology per MSK (different
`parent_body`, different pos/quat), use the per-MSK form:

```yaml
attachments:
  default:
    - device_body: "hmedi_torso"
      parent_body: "torso"
  myoLeg80:
    - device_body: "hmedi_torso"
      parent_body: "pelvis"
      pos: [-0.105, 0.08, 0]
    # ...repeat any other attachments unchanged
```

The resolver returns the matching MSK key's list if present, else `default`.
Use this when even one attachment needs to differ per MSK.

## `joint_overrides`

Modify properties of existing joints in the MSK.

```yaml
joint_overrides:
  - name: "ankle_angle_r"
    range: [-0.45, 0.349]
    damping: 0.5
  - name: "mtp_angle_r"
    range: [0.2, 0.5]
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `name` | string | yes | MSK joint name. |
| `range` | `[lo, hi]` | no | New range of motion. |
| `damping` | float | no | New damping value. |
| `axis` | `[x, y, z]` | no | Joint axis (rarely used). |
| `pos` | `[x, y, z]` | no | Joint position (rarely used). |

## `actuators`

Add new actuators to the combined model. For joint-transmission actuators
only — tendon-transmission actuators are authored in the device XML directly.

```yaml
actuators:
  - name: "Exo_R"
    type: "general"
    joint: "ankle_angle_r"
    gaintype: "fixed"
    gainprm: [100, 0, 0]
    biastype: "none"
    biasprm: [0, 0, 0]
    dyntype: "none"
    dynprm: [1, 0, 0]
    ctrlrange: [-1, 0]
    ctrllimited: true
    gear: [1.0]
```

| Field | Required | Meaning |
|---|---|---|
| `name` | yes | Actuator name (will *not* be prefixed; declare with the final name you want). |
| `joint` | yes | Target joint. If the bare name isn't found, the pipeline tries `<prefix>_<joint>` (for device-added joints like `osl_ankle_angle_r`). |
| `type` | no | Reserved; currently always "general". |
| `gaintype` / `biastype` / `dyntype` | no | `"fixed"`, `"affine"`, `"muscle"`, `"user"`, `"none"`, `"integrator"`, `"filter"`, `"filterexact"`. Mapped to the appropriate MuJoCo enum. |
| `gainprm` / `biasprm` / `dynprm` | no | Numeric arrays (length-padded to 10). |
| `ctrlrange` / `ctrllimited` | no | Standard MuJoCo. |
| `gear` | no | Length-padded to 6. |

For *tendon-transmission* actuators (e.g. HMEDI's cables driving spatial
tendons), author them in the device XML's `<actuator>` section. They get
imported automatically with the device prefix at attach time.

## `keyframe_overrides`

Patch joint values in the MSK's existing keyframes. Model-agnostic: refers
to joints by name, not index.

```yaml
keyframe_overrides:
  stand:
    pelvis_ty: 0.96
  walk_left:
    pelvis_ty: 0.93285
  squat:
    pelvis_ty: 0.77
    osl_ankle_angle_r: 0.385      # device-added joint; prefix auto-resolved
```

Each top-level key is a keyframe name that must already exist in the MSK.
Joints not listed keep their authored value.

### Per-MSK keyframe_overrides

```yaml
keyframe_overrides:
  default:
    stand:
      pelvis_ty: 0.91
  myoLeg80:
    squat:
      osl_ankle_angle_r: 0.393
```

Use when joint names differ across MSKs (e.g. `pelvis_ty` doesn't exist in
80-muscle, which uses a freejoint root).

## `keyframes` (legacy)

Replace keyframes entirely with explicit qpos/qvel arrays. Model-specific
(must match `nq` / `nv` exactly). Avoid unless you really need to author
full arrays — use `keyframe_overrides` instead for model-agnostic patches.

```yaml
keyframes:
  stand:
    time: 0.0
    qpos: [0.0, 0.96, 0.0, ...]
    qvel: [0.0, 0.0, ...]
```

## `body_removals`

Delete biological body subtrees from the MSK before attaching the device.
Removes all child bodies, joints, geoms, sites recursively. Cascade cleanup
removes contact pairs, sensors, equalities, and tendon wraps referencing
removed elements.

```yaml
body_removals:
  - "talus_r"   # cascades to calcn_r, toes_r (transtibial amputation)
```

For prosthetics. Also auto-prunes qpos / qvel slots from keyframes for any
joint inside removed subtrees.

## `actuator_removals` / `tendon_removals`

Remove named actuators / tendons.

```yaml
actuator_removals:
  default:
    - "soleus_r"
    - "tibant_r"
  myoLeg80:
    - "soleus_r"
    - "tibant_r"
    - "gaslat_r"        # 80-only equivalent

tendon_removals:
  default:
    - "soleus_r_tendon"
    - "tib_ant_r_tendon"
```

Both sections support per-MSK overrides (top-level `default:` + MSK key).

## `tendon_modifications`

Edit tendon wraps without rebuilding the whole tendon. Three operations
per wrap:

```yaml
tendon_modifications:
  default:
    - name: "rect_fem_r_tendon"
      wraps:
        # Reposition a wrap site on the same body (move xyz only)
        - reposition_site: "rect_fem_r_rect_fem_r-P2"
          pos: [0.045, -0.2, 0.005]

        # Replace a wrap site -- re-anchor onto a different body at xyz
        - replace_site: "rect_fem_r_rect_fem_r-P3"
          new_body: "femur_r"
          pos: [0.025, -0.275, 0.0075]

        # Drop a wrap entirely
        - drop_site: "some_obsolete_wrap_site"
  myoLeg80: []                      # explicitly no mods on 80
```

| Op | Required fields | Meaning |
|---|---|---|
| `reposition_site` | `pos` | Move the wrap to a new xyz on its *current* body. |
| `replace_site` | `new_body`, `pos` | Re-anchor the wrap onto a different body at xyz. |
| `drop_site` | (none) | Remove the wrap entirely. |

Synthesized sites are named `{original_name}__mod` (e.g.
`rect_fem_r_rect_fem_r-P2__mod`).

**Default behavior (no mods listed):** when `body_removals` removes a body
whose sites are referenced by a tendon wrap, those wraps are *auto-pruned*
in the preprocess pass. `tendon_modifications` is only needed when you want
to re-anchor / reposition rather than drop.

Per-MSK supported. Use `myoLeg80: []` to disable mods on 80 when the
default block references 22/26-specific tendon names.

## `mesh_replacements`

Swap a geom's mesh to a replacement mesh defined in the device XML's
`<asset>` section.

```yaml
mesh_replacements:
  default:
    - geom: "tibia_r_geom_1"
      mesh: "osl_tibia_fibula_trans_r"        # device-XML mesh name; prefix added automatically
  myoLeg80:
    - geom: "r_tibia"                          # different geom name in 80
      mesh: "osl_tibia_fibula_trans_r"
```

The replacement mesh must be declared in the device's `model_xml` `<asset>`
section. Its name in the *combined* model is the prefixed version (e.g.
`OSL_A_L1_osl_tibia_fibula_trans_r`); the YAML uses the bare name and the
prefix is added at substitution time.

Per-MSK supported (typical use: different MSKs have different geom names
on the same body, e.g. `tibia_r_geom_1` vs `r_tibia`).

## `geom_removals`

Surgical geom removal — for cases where `mesh_replacements` swaps one geom
on a body but a sibling geom needs to disappear too.

```yaml
geom_removals:
  default:
    - "tibia_r_geom_2"     # fibula geom (the residual stump mesh covers both bones)
  myoLeg80:
    - "r_fibula"
```

Canonical use: transtibial amputation where the residual mesh covers
tibia + fibula but the MSK had them as two separate geoms. The strip
cascades into contact pair cleanup.

Per-MSK supported.

## Per-MSK overrides — summary

Sections that support the `default:` + `<msk_key>:` dispatch:

| Section | Per-MSK? |
|---|---|
| `attachments` | ✓ |
| `joint_overrides` | (planned; currently default form only) |
| `actuators` | (planned; currently default form only) |
| `keyframe_overrides` | ✓ |
| `body_removals` | (planned; currently default form only) |
| `mesh_replacements` | ✓ |
| `actuator_removals` | ✓ |
| `tendon_removals` | ✓ |
| `tendon_modifications` | ✓ |
| `geom_removals` | ✓ |

Sections marked "planned" use the flat list form for now; per-MSK support
is incremental as real configs need it.

## Authoring rules of thumb

1. **Order matters in the schema's expression but not in execution.**
   The YAML can list sections in any order; the pipeline runs the
   removals before attachments and keyframe rebuilds at compile time.
2. **Tendons/actuators that *cross* an amputation level need explicit
   removal.** Body removal cascades remove tendons whose wrap sites are
   entirely inside the removed subtree; tendons that wrap *across* a
   removed body need explicit `tendon_removals`.
3. **Use the device prefix in `keyframe_overrides` for device-added
   joints.** `keyframe_overrides` can refer to either bare names (MSK
   joints) or prefixed names (device joints like `OSL_KA_L1_osl_knee_angle_r`),
   but the bare-then-prefixed fallback in the resolver means you can
   often omit the prefix — the resolver tries both.
4. **For unnamed joint inside a body, you still must remove the body if
   you want it gone.** Removals are by body name; the joints come along.
