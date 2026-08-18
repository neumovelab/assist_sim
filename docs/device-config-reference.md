# Device Config Reference

Each device under `models/<DeviceDir>/` has a YAML config file. The usual name
is `L1config.yaml` or `<variant>_L1config.yaml`. This document is the schema
reference. It gives each section and each field, with examples.

For step-by-step instructions, see
[how-to/add-a-device.md](how-to/add-a-device.md).

## Top-level shape

```yaml
device:                      # required
  name: ...
  model_xml: ...
  compatible_msk: ...        # optional

attachments: ...             # required

# Optional sections, all default to empty:
equality: ...
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
body_overrides: ...
actuator_overrides: ...
contact: ...
sensors: ...
sensor_removals: ...
```

Only the `device` and `attachments` sections are necessary. Each other section
is empty by default.

## `device`

```yaml
device:
  name: "DephyExoBoot_L1"
  model_xml: "L1model.xml"
  compatible_msk: ["myolegs22", "myolegs26"]   # optional
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `name` | string | yes | The namespace prefix. The pipeline applies it to every body, site, mesh, joint, actuator and tendon that it imports from the device XML file. The convention is PascalCase with an `_L1` suffix, for example `DephyExoBoot_L1` or `OpenSourceLeg_A_L1`. |
| `model_xml` | path | yes | The path to the MuJoCo XML file of the device, relative to this YAML file. |
| `compatible_msk` | list | no | Limits the musculoskeletal (MSK) models that this device can combine with. If the field is absent, the device is compatible with all MSK models. |

## `attachments`

This section maps each top-level device body to a parent body in the MSK model.

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
| `device_body` | string | yes | The name of a top-level body in the device XML file. |
| `parent_body` | string | yes | The name of the MSK body that the device body attaches under. The special value `world` (or `worldbody`) attaches the device body directly under worldbody and keeps its own `<freejoint>`. See [Free-rooted attachment](#free-rooted-attachment-parent_body-world). |
| `pos` | `[x, y, z]` | no | The frame offset on the parent. It composes with the authored pos of the device body. Use it when the authored pos of the device body needs a change for each attachment point. |
| `quat` | `[w, x, y, z]` | no | The frame rotation. Use it when the frame of the parent body is different between MSK models, for example the 22-muscle torso and the 80-muscle torso. |

The pipeline implements each attachment as `parent.add_frame(pos, quat).attach_body(device_body, prefix=device.name + "_")`.

### Per-MSK attachments

A device can need a different attachment topology for each MSK model, with a
different `parent_body` or with different pos and quat values. In that
condition, use the per-MSK form:

```yaml
attachments:
  default:
    - device_body: "hmedi_torso"
      parent_body: "torso"
  myolegs:
    - device_body: "hmedi_torso"
      parent_body: "pelvis"
      pos: [-0.105, 0.08, 0]
    # ...repeat any other attachments unchanged
```

The resolver returns the list for the matching MSK key. If that list is not
present, the resolver returns `default`. Use this form if only one attachment
must be different for an MSK model.

### Free-rooted attachment (`parent_body: world`)

The pipeline re-parents most devices *rigidly* onto a leg body. But some
devices are a separate mechanism that straps to the leg. An example is the UT
ankle exoskeleton, which is a parallel linkage with clamps at several points.
Such a device attaches to `world` instead, and [`equality`](#equality)
constraints then connect it to the leg:

```yaml
attachments:
  - device_body: "part3_dx"      # a top-level body carrying its own <freejoint>
    parent_body: "world"
    pos: [-0.1574, 0.0345, -0.5832]   # world pose of the free root at qpos0
    quat: [0.1209, 0.0, 0.0, 0.9927]
```

The device body must specify a `<freejoint>` in the device XML file. If it does
not, MuJoCo welds the body to the world and the body cannot move. `pos` and
`quat` give the world pose of the free root at the start. Set them so that the
exoskeleton sits on the leg, because the MuJoCo `connect` constraints below
hold the two bodies at the relative pose that exists at compile time. Per-MSK
`attachments` are usually necessary here, because each baseline puts the leg in
a different world frame.

## `equality`

This section adds MuJoCo `<equality>` constraints that connect a device body to
an MSK body. A free-rooted device attaches to `world`, and these constraints
then hold it to the leg. They are the equivalent of the rigid re-parenting that
a plain attachment does. The pipeline adds them after the attachment, so that
both endpoints exist.

```yaml
equality:
  - type: "connect"            # point-to-point (ball) joint
    device_body: "part3_dx"    # prefixed automatically
    parent_body: "calcn_r"     # bare MSK body
    anchor: [-0.071, 0.05, 0.005]   # in the device body's local frame
  - type: "weld"               # fixes the full relative pose
    device_body: "cuff_r"
    parent_body: "tibia_r"
    relpose: [0, 0, 0, 1, 0, 0, 0]  # optional pos + quat (default identity)
    torquescale: 1.0                # optional
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `type` | `connect` \| `weld` \| `joint` | yes | `connect` = point-to-point; `weld` = full pose lock; `joint` = scalar joint coupling. |
| `device_body` | string | connect/weld | The device body. The pipeline adds the device prefix at combine time. |
| `parent_body` | string | connect/weld | The MSK body, with no prefix. |
| `joint1` / `joint2` | string | joint | The coupled joints. The pipeline resolves each name without a prefix first, then with the prefix. |
| `polycoef` | up to 5 floats | no (joint) | The quartic coefficients. Trailing terms that you do not give are zero. |
| `anchor` | `[x, y, z]` | connect | The connection point, in the local frame of the device body. |
| `relpose` | `[x, y, z, qw, qx, qy, qz]` | no (weld) | The relative pose. The default is identity. |
| `torquescale` | float | no (weld) | The weld torque scale. The default is 1.0. |
| `solref` / `solimp` | list | no | The constraint-solver parameters. The defaults are the MuJoCo defaults. |
| `active` | bool | no | Sets if the constraint is active at the start. The default is `true`. |

This section supports the per-MSK `default:` and `<msk_key>:` form, like
`attachments`.

> **A connect anchor records the coincidence at compile time.** A `connect`
> gives *no* initial violation, whatever its position. It holds the two bodies
> at their relative pose at qpos0. Therefore the *attachment* `pos` and `quat`
> control the position of the exoskeleton, not the anchor.

### `type: joint`: close a kinematic loop

The MuJoCo body graph is strictly a tree. Thus nested bodies alone cannot give
a closed linkage. A joint equality couples two scalar joints with a quartic:

```
y - y0 = a0 + a1*(x - x0) + a2*(x - x0)^2 + a3*(x - x0)^3 + a4*(x - x0)^4
```

In this equation, `y` is `joint1`, `x` is `joint2`, and `x0` and `y0` are their
reference values at qpos0. If you do not give `joint2`, the constraint holds
`joint1` at the constant in `polycoef[0]`.

```yaml
equality:
  # device joint <-> device joint: closes the loop
  - type: joint
    joint1: "shank_r__link_bcd_r"
    joint2: "shank_r__link_ag_r"
    polycoef: [3.2455e-05, 1.1296e+00, -2.3405e-02, -2.0102e-02, 8.6030e-02]
    solimp: [0.9999, 0.9999, 0.001, 0.5, 2]
    solref: [0.002, 1]
  # device joint <-> MSK joint: ties the linkage to the joint it spans
  - type: joint
    joint1: "shank_r__link_ag_r"
    joint2: "ankle_angle_r"
    polycoef: [2.0673e-02, -8.7548e-01, 6.1119e-02, 3.4395e-02, 1.0843e-01]
```

`STRIDE_L2` is the example: five couplings on each side. Four of them close the
six-bar, and one connects its master hinge to `ankle_angle`.

> **Use a joint coupling instead of `connect` for a loop.** A `connect` applies
> three constraints when two are sufficient, and the redundancy causes drift.
> The measurement on STRIDE gives 3.7 mm of link separation with `connect`, and
> 0.020 mm with joint couplings.

> **A loop needs stiff solver settings.** With the MuJoCo defaults, the links
> drift visibly through the range of motion. `solimp: [0.9999, 0.9999, 0.001,
> 0.5, 2]` with `solref: [0.002, 1]` holds the STRIDE loop to less than ten
> microradians.

> **Restrict the driven joint to the fit window.** A quartic fit over the
> travel of a mechanism gives incorrect values outside that travel, and the
> incorrect target then acts against the limits of the linkage joints. Clamp
> the MSK joint with a `joint_overrides` range. Use the intersection with the
> range that the MSK model already has, so that the device never *increases* an
> anatomical limit.

## `joint_overrides`

This section changes the properties of joints that exist in the MSK model.

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
| `name` | string | yes | The name of the MSK joint. |
| `range` | `[lo, hi]` | no | The new range of motion. |
| `damping` | float | no | The new damping value. |
| `axis` | `[x, y, z]` | no | The joint axis. This field is rarely necessary. |
| `pos` | `[x, y, z]` | no | The joint position. This field is rarely necessary. |

## `actuators`

This section adds new actuators to the combined model. It is only for
joint-transmission actuators. You author tendon-transmission actuators directly
in the device XML file.

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
| `name` | yes | The actuator name. The pipeline does **not** add a prefix, so give the final name that you want. |
| `joint` | yes | The target joint. If the pipeline does not find the name without a prefix, it tries `<prefix>_<joint>`. This applies to joints that the device adds, such as `osl_ankle_angle_r`. |
| `type` | no | Reserved. The value is always "general". |
| `gaintype` / `biastype` / `dyntype` | no | `"fixed"`, `"affine"`, `"muscle"`, `"user"`, `"none"`, `"integrator"`, `"filter"`, `"filterexact"`. The pipeline maps the value to the correct MuJoCo enum. |
| `gainprm` / `biasprm` / `dynprm` | no | Numeric arrays. The pipeline pads them to length 10. |
| `ctrlrange` / `ctrllimited` | no | Standard MuJoCo. |
| `gear` | no | The pipeline pads this array to length 6. |

Author *tendon-transmission* actuators in the `<actuator>` section of the
device XML file. An example is the HMEDI cables that drive the spatial tendons.
The pipeline imports them automatically with the device prefix at attachment
time.

## `keyframe_overrides`

This section changes joint values in the keyframes that the MSK model already
has. It is model-agnostic, because it refers to joints by name and not by
index.

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

Each top-level key is a keyframe name. That keyframe must already exist in the
MSK model. A joint that you do not list keeps its authored value.

### Per-MSK keyframe_overrides

```yaml
keyframe_overrides:
  default:
    stand:
      pelvis_ty: 0.91
  myolegs:
    squat:
      osl_ankle_angle_r: 0.393
```

Use this form when the joint names are different between MSK models. For
example, `pelvis_ty` does not exist in the 80-muscle model, which uses a
freejoint root.

## `keyframes` (legacy)

This section replaces the keyframes completely with explicit qpos and qvel
arrays. It is model-specific: the arrays must agree with `nq` and `nv` exactly.
Use it only when you must author full arrays. For model-agnostic changes, use
`keyframe_overrides` instead.

```yaml
keyframes:
  stand:
    time: 0.0
    qpos: [0.0, 0.96, 0.0, ...]
    qvel: [0.0, 0.0, ...]
```

## `body_removals`

This section removes biological body subtrees from the MSK model. `spec.delete`
cascades: it removes the full subtree (child bodies, joints, geoms and sites),
the sensors, the equalities, and the actuators and tendons that referred to it.
The pipeline removes a contact `<pair>` that names a removed geom separately,
because `spec.delete` does not cascade to those.

```yaml
body_removals:
  default:
    - "tibia_r"      # cascades to talus_r, calcn_r, toes_r
  myolegs:
    - "tibia_r"
    - "patella_r"    # a sibling of tibia_r, so the cascade misses it
```

The pipeline decomposes the keyframes by joint name before the removals, then
rebuilds them after the final compile. Thus the slots of a removed joint
disappear.

This section supports per-MSK entries, and they are usually necessary. The
lineages have anatomical differences: only the 80-muscle models have a
`patella_r`.

> **You must re-anchor a muscle that you want to keep, before the removals.**
> The cascade removes each tendon that has a wrap on a removed body. This
> includes a tendon that only *crosses* the amputation level. See
> [`tendon_modifications`](#tendon_modifications).

## `actuator_removals` / `tendon_removals`

These sections remove the actuators and the tendons that you name.

```yaml
actuator_removals:
  default:
    - "soleus_r"
    - "tibant_r"
  myolegs:
    - "soleus_r"
    - "tibant_r"
    - "gaslat_r"        # 80-only equivalent

tendon_removals:
  default:
    - "soleus_r_tendon"
    - "tib_ant_r_tendon"
```

The two sections tolerate an absent element. The body cascade can remove the
element first, and a name that no longer exists causes no error. The sections
are mostly an explicit record of the muscles that the amputation removes.

The two sections support per-MSK overrides, with a top-level `default:` key and
MSK keys.

## `tendon_modifications`

This section moves the wrap points of a muscle onto the bone that remains after
an amputation. This is the myodesis step. Real surgery keeps a biarticular
muscle and attaches it again to the residual bone, where it continues to act at
the joint that remains.

> **This section runs before the removals.** The `spec.delete` cascade removes
> a muscle that has wrap points on a body that `body_removals` removes. The
> muscle remains only if you re-anchor it first.

The pipeline does not rewrite the tendon: a wrap holds its site or geom **by
name** and resolves that name at compile time. Thus a move of the named element
moves the wrap. `replace_*` adds the element on `new_body` with a placeholder
name, removes the original, then gives the free name to the replacement. The
replacement takes the class of the original and its visual and collision
attributes. The pipeline changes elements and does not rebuild tendons. Thus
the actuator objects stay in place and the `ctrl` order does not change.

There are four operations. The op key selects the operation, and its value
gives the name of the target element:

```yaml
tendon_modifications:
  default:
    - name: "rect_fem_r_tendon"      # hip flexion survives
      wraps:
        # Move a site on the body it already sits on.
        - reposition_site: "rect_fem_r_rect_fem_r-P2"
          pos: [0.045, -0.2, 0.005]
        # Move a site onto the residual bone.
        - replace_site: "rect_fem_r_rect_fem_r-P3"
          new_body: "femur_r"
          pos: [0.025, -0.275, 0.0075]
  myolegs:
    - name: "semimem_r_tendon"
      wraps:
        # A wrap cylinder and its sidesite move together.
        - replace_geom: "SM_at_condyles_wrap_r"
          new_body: "femur_r"
          pos: [0.01464, -0.270, 0.00916]
        - replace_site: "SM_at_condyles_site_semimem_r"
          new_body: "femur_r"
          pos: [0.01259, -0.270, 0.01207]
        - replace_site: "semimem-P2_r"
          new_body: "femur_r"
          pos: [0.01259, -0.28301, 0.01207]
```

| Op | Value | Required fields | Effect |
|---|---|---|---|
| `reposition_site` | site name | `pos` | Moves the site to a new xyz position on its *current* body. |
| `replace_site` | site name | `new_body`, `pos` | Moves the site onto `new_body`, at the given xyz position. |
| `reposition_geom` | geom name | `pos` | Moves the wrap geom to a new xyz position on its *current* body. |
| `replace_geom` | geom name | `new_body`, `pos` | Moves the wrap geom onto `new_body`, at the given xyz position. |

Each wrap entry has exactly one op. `pos` is in the local frame of the body
that receives the element: the current body for `reposition_*`, and `new_body`
for `replace_*`. An unknown tendon, element or `new_body` raises an error.

Rules for the author:

- Move **every** point at the cut plane or distal to it. This includes the
  points that are already on the body that remains. Use `reposition_*` for
  those points.
- **You must also move the sidesite of a wrap cylinder.** If one geom stays on
  a body that the pipeline removes, the cascade removes the tendon, whatever
  number of sites you moved. A sidesite that stays behind has the same result.
- Spread the new points along the residual bone, so that no tendon segment has
  zero length.
- A re-anchor changes the operating range of the muscle. Give the actuator a
  new `lengthrange` with [`actuator_overrides`](#actuator_overrides).

You cannot remove a wrap, because `MjsTendon` gives no editable list of wraps.
The obsolete `drop_site` op raises an error with that reason. To remove a
muscle, use `actuator_removals` and `tendon_removals`, or let the cascade
remove it.

This section supports per-MSK entries. If the block does not name an MSK key,
and the block has no `default:` entry, the resolver gives no modifications.

`OpenSourceLeg/KA_L1config.yaml` is the example. It is a transfemoral
amputation. It re-anchors two muscles on the 22-muscle and 26-muscle lineage,
and eight muscles on the 80-muscle lineage, which splits the same muscles.

## `mesh_replacements`

This section changes the mesh of a geom to a replacement mesh from the
`<asset>` section of the device XML file.

```yaml
mesh_replacements:
  default:
    - geom: "tibia_r_geom_1"
      mesh: "osl_tibia_fibula_trans_r"        # device-XML mesh name; prefix added automatically
  myolegs:
    - geom: "r_tibia"                          # different geom name in 80
      mesh: "osl_tibia_fibula_trans_r"
```

The `<asset>` section of the `model_xml` file of the device must specify the
replacement mesh. In the *combined* model, the mesh name has the prefix, for
example `OSL_A_L1_osl_tibia_fibula_trans_r`. The YAML file uses the name with
no prefix, and the pipeline adds the prefix at substitution time.

This section supports per-MSK entries. The usual reason is that different MSK
models use different geom names on the same body, for example
`tibia_r_geom_1` and `r_tibia`.

## `geom_removals`

This section removes a geom. Use it when `mesh_replacements` changes one geom
on a body, but you must also remove a second geom on the same body.

```yaml
geom_removals:
  default:
    - "tibia_r_geom_2"     # fibula geom (the residual stump mesh covers both bones)
  myolegs:
    - "r_fibula"
```

An example is a transtibial amputation where the residual mesh covers the tibia
and the fibula, but the MSK model has them as two separate geoms. The removal
also cascades to the cleanup of the contact pairs.

This section supports per-MSK entries.

## `body_overrides`

This section overrides the inertial properties of a body in the combined model.
The target is an MSK body, with no prefix on the name, or a device body, which
the pipeline resolves with the device prefix. Only the fields that you give
change.

```yaml
body_overrides:
  - name: "tibia_r"
    mass: 1.85375
    diaginertia: [0.0125, 0.0125, 0.00225]
    ipos: [0, -0.125, 0]
    iquat: [0.5, 0.5, -0.5, 0.5]
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `name` | string | yes | The body to change. Give the MSK name with no prefix, or the device name, which the pipeline resolves with the prefix. |
| `mass` | float | no | The new mass, in kg. |
| `diaginertia` | `[Ixx, Iyy, Izz]` | no | The principal moments, in the frame that `iquat` sets. Do not use it with `fullinertia`. |
| `fullinertia` | `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]` | no | The full tensor. Do not use it with `diaginertia`. |
| `ipos` | `[x, y, z]` | no | The origin of the inertial frame, in the body frame. |
| `iquat` | `[qw, qx, qy, qz]` | no | The orientation of the inertial frame. |

This section supports the per-MSK `default:` and `<msk_key>:` form.

This section is the mass equivalent of `mesh_replacements`. Amputation is the
primary use: `spec.delete` removes the distal subtree, but the parent body that
remains still has the mass of the *complete* segment. MuJoCo does not know that
the segment was cut. All three transtibial devices (`KFoot_L1`,
`OpenSourceLeg_A_L1` and `NEUankle_L1`) decrease `tibia_r` in this way. Without
this decrease, the prosthetic side has approximately 1.85 kg of unwanted mass.

> **A `mass` value alone raises an error on a body with compiler-derived
> inertia.** MuJoCo derives the inertia of a body from its geoms, unless the
> body is `explicitinertial`. If the pipeline made the body explicit and gave
> only a mass, the inertia would stay at zero with no warning. Therefore the
> pipeline rejects that combination. Give an inertia also.

## `actuator_overrides`

This section overrides the properties of an actuator in the combined model. The
target is an MSK actuator, with no prefix on the name, or a device actuator,
which the pipeline resolves with the device prefix. The pipeline applies this
section after the removals and the attachment, and before the final compile.
Thus the section can name a muscle that remains only because
`tendon_modifications` re-anchored it.

```yaml
actuator_overrides:
  myolegs26:
    - name: "rectfem_r"
      lengthrange: [0.226056, 0.330193]
    - name: "hamstrings_r"
      lengthrange: [0.238041, 0.343585]
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `name` | string | yes | The actuator to change. Give the MSK name with no prefix, or the device name, which the pipeline resolves with the prefix. |
| `lengthrange` | `[lo, hi]` | yes | The operating length range, in metres. `lo` must be less than `hi`. |

This section supports the per-MSK `default:` and `<msk_key>:` form. Per-MSK
entries are important here. A range that you derive again applies to the
geometry and the muscle names of one lineage only. The 80-muscle models divide
the muscles that the 22-muscle and 26-muscle models combine.

> **A re-anchored muscle keeps the incorrect `lengthrange`.** `lengthrange`
> normalises the force-length curve of a muscle, and the MuJoCo compiler keeps
> the value that the model authored, because `LRopt.useexisting` is 1. After a
> re-anchor, the muscle has the operating range of a path that it no longer
> has, and it gives incorrect forces with no warning. On `myolegs26`, the
> author gave `rectfem_r` the range `[0.321, 0.510]`. After the re-anchor,
> `rectfem_r` operates over approximately `[0.227, 0.329]`, which starts below
> its own lower limit.

> **Derive the values from a kinematic joint sweep.** Move the joints that the
> muscle still spans across their model ranges. Record the minimum and the
> maximum tendon length, and add a small margin. Do **not** use
> `mj_setLengthRange`. It ignores the joint limits, so it reports lengths for
> poses that the model cannot reach.

## `contact`

This section adds MuJoCo `<contact>` entries to the combined model. The
pipeline adds them after the attachment, so that the device geoms and bodies
exist. It resolves each name without a prefix first, then with the prefix.

```yaml
contact:
  pairs:
    - {geom1: "shank_r_main_geom", geom2: "femur_r_geom_1"}
  excludes:
    - ["foot_r", "calcn_r"]          # two-item list form
    - {body1: "link_gcf_l", body2: "calcn_l"}   # or explicit keys
```

The `pairs` fields are `geom1` and `geom2`, which are necessary, plus the
optional `condim`, `margin`, `gap`, `friction`, `solref` and `solimp`. Each
`excludes` entry takes two body names.

The two sub-sections support the per-MSK `default:` and `<msk_key>:` form. This
form is important, because the geom names are different between lineages. The
80-muscle models call the femur mesh geom `femur_r`, and the 26-muscle models
call it `femur_r_geom_1`. Also, some bodies exist only on some MSK models.

This section is necessary because `attach_body` copies a body subtree and not
the top-level sections. Thus the `<contact>` block of the device XML file never
moves to the combined model.

> **Use `contype` and `conaffinity` before you use excludes.** MuJoCo already
> ignores geoms in the same weld group and in parent-child body pairs. Thus a
> device that welds onto the MSK model, with no joints of its own, needs no
> excludes. A device with joints does need them: its links are separate weld
> groups, and the grandparent and sibling pairs are candidates for contact.
>
> Instead of one exclude for each pair, give every device geom
> `contype="2" conaffinity="1"`. A device-to-device test then gives `2 & 1 = 0`
> in both directions and makes no contact, but bone and ground (`contype=1`)
> continue to make contact. This method replaced approximately 30 excludes on
> each side of `STRIDE_L2`. The cost is that device parts make no contact with
> each other, and this includes left against right.

> **An `exclude` does not cancel a `pair`.** The two act at different stages. MuJoCo
> always evaluates a predefined `pair`; an `exclude` only filters the body pairs that
> the broadphase would otherwise generate. Measured on MuJoCo 3.4 and 3.11: two bodies
> with an explicit pair between their geoms give one contact whether or not an exclude
> names those bodies. So if a forced pair makes no contact, look at the geometry, the
> `margin` and the `condim` rather than at your excludes.
>
> An earlier version of this page said the opposite. If you removed an exclude to make
> a pair work, the exclude was not the cause.

> **A welded device body takes the weld group of its host.** Thus MuJoCo
> automatically excludes a part on the shank from the *femur*, which is the
> parent of its weld group. The part then goes into the thigh, and MuJoCo makes
> no contact. This condition needs an explicit `pair`, which ignores the
> `contype` and `conaffinity` filter completely.

## `sensors` / `sensor_removals`

These sections add sensors to the combined model and remove sensors from it.

```yaml
sensor_removals: ["r_foot", "r_toes"]

sensors:
  - {name: "r_foot", type: touch, site: "r_sole_touch"}
  - {name: "r_ankle_sensor", type: jointlimitfrc, joint: "neuankle_ankle_angle_r"}
```

Each sensor names exactly one target. The key selects the kind of target:
`site`, `joint`, `actuator`, `tendon`, `body` or `geom`. The optional `cutoff`
and `noise` fields pass through. The pipeline supports these `type` values: `touch`,
`force`, `torque`, `jointpos`, `jointvel`, `jointlimitpos`, `jointlimitvel`,
`jointlimitfrc`, `jointactuatorfrc`, `actuatorpos`, `actuatorvel`,
`actuatorfrc`, `tendonpos`, `tendonvel`, `framepos`, `framequat`,
`framelinvel`, `frameangvel`. A type that does not accept the kind of target
that you give raises an error.

The two sections support the per-MSK `default:` and `<msk_key>:` form. This
form is important here. `myolegs26` and `myolegs22` have twelve sensors, with a
`jointlimitfrc` sensor on every leg joint. `myolegs` and `myofullbody` have
only the four leg touch sensors. If you restore a `jointlimitfrc` sensor on
`myolegs` or `myofullbody`, it becomes the only sensor of that kind in the
model.

There are two different uses:

- **To restore what the cascade removed.** When you remove `talus_r`, the
  cascade also removes `r_foot` and `r_toes`, which are touch sensors on the
  removed sites, and `r_ankle_sensor` and `r_mtp_sensor`. The count of sensors
  decreases from the baseline twelve to eight. No sensor then reads the
  prosthetic side, but the intact side keeps all four equivalent sensors.
- **To point a sensor at a different target.** Remove the sensor, then add it
  again with the same name. A device with a shoe moves the ground contact from
  the bare foot onto the sole, and the baseline touch site is a *box*.
  `r_foot_touch` spans y from -0.022 to 0.018 on `calcn_r`, but the contact
  surface of a shoe sole is at y = -0.058, which is 35 mm below the bottom of
  the box. A touch sensor that stays on the baseline site reads exactly zero
  for the complete stance phase.

> **The pipeline adds sensors at the end of the list.** Thus the `sensordata`
> order of the combined model is different from the baseline order. Index by
> name, never by position.

> **Give a device site a different name if the MSK model keeps a site with that
> name.** The pipeline resolves a target without a prefix first. Thus the MSK
> site that remains hides a device site that is also called `r_foot_touch`, and
> the sensor continues to read the bare foot. The pipeline gives no warning.
> For this reason, `STRIDE_L2` names its sole sites `r_sole_touch` and
> `r_forefoot_touch`, but it keeps the *sensor* name `r_foot`, so that
> downstream consumers see no change.

## Per-MSK overrides: summary

These sections support the `default:` and `<msk_key>:` dispatch:

| Section | Per-MSK? | An MSK block... |
|---|---|---|
| `attachments` | ✓ | replaces `default` |
| `equality` | ✓ | replaces `default` |
| `joint_overrides` | ✓ | replaces `default` |
| `actuators` | flat form only | — |
| `keyframe_overrides` | ✓ | **merges onto `default`** |
| `keyframes` (legacy) | flat form only | — |
| `body_removals` | ✓ | replaces `default` |
| `mesh_replacements` | ✓ | replaces `default` |
| `actuator_removals` | ✓ | replaces `default` |
| `tendon_removals` | ✓ | replaces `default` |
| `tendon_modifications` | ✓ | replaces `default` |
| `geom_removals` | ✓ | replaces `default` |
| `body_overrides` | ✓ | replaces `default` |
| `actuator_overrides` | ✓ | replaces `default` |
| `contact` (`pairs` + `excludes`) | ✓ | replaces `default` |
| `sensors` | ✓ | replaces `default` |
| `sensor_removals` | ✓ | replaces `default` |

`keyframe_overrides` is the one section that **merges** rather than replaces, joint by joint.
It is already a patch rather than a definition, and a lineage usually needs one joint of one
pose changed -- most often the lunge knee, whose sign differs between the negative-flexion
(`myolegs22`, `myolegs26`) and positive-flexion (`myolegs`, `myofullbody`) models. Under
replace semantics that meant restating every pose per MSK. So this is enough:

```yaml
keyframe_overrides:
  default:
    lunge: {pelvis_ty: 0.675, knee_angle_l: -1.25}
  myolegs:
    lunge: {knee_angle_l: 1.25}      # pelvis_ty still comes from default
  myofullbody:
    lunge: {knee_angle_l: 1.25}
```

The trade-off is that a per-MSK block can add or change a joint but never drop one that
`default` sets.

The `actuators` section and the legacy `keyframes` section take the flat form
only. Handing `actuators` the dict form raises, naming the section.

A section in the per-MSK form with no `default:` entry resolves to nothing, for
each MSK key that it does not name. `attachments` is the one exception, because
the dict form needs a `default:` entry.

### The loader rejects what it cannot use

Every key in this reference is checked at load time. A config that names something the parser
does not read raises `ValueError` with a "did you mean", rather than loading clean and doing
nothing. That covers an unknown top-level section, an unknown key in the `device` block, an
unknown key inside an entry (`position` where `pos` was meant), an unknown key in a wrap edit,
a per-MSK block keyed by a name that is not an MSK key (`myolegz26`), and an actuator `type`
the pipeline does not implement. If you are editing an existing config, an error here means the
key was previously being ignored.

## General rules for authors

1. **The order in the YAML file is free, but the order in the pipeline is
   fixed.** You can list the sections in any order. The pipeline always runs
   `tendon_modifications` first, then the removals, then the attachment and the
   sections that add elements. It rebuilds the keyframes last, at compile time.
2. **The cascade removes a tendon that has a wrap on a removed body.** This
   includes a muscle that only *crosses* the amputation level. Thus
   `actuator_removals` and `tendon_removals` are mostly an explicit record of
   what the amputation removes. To keep a muscle, re-anchor it with
   `tendon_modifications`.
3. **Use the device prefix in `keyframe_overrides` for a joint that the device
   adds.** `keyframe_overrides` accepts a name with no prefix, for an MSK
   joint, or a name with the prefix, for a device joint such as
   `OSL_KA_L1_osl_knee_angle_r`. The resolver tries the name without the prefix
   first, then with the prefix. Thus you can usually omit the prefix.
4. **To remove a joint that has no name, you must remove its body.** The
   pipeline removes by body name, and the joints go with the body.
