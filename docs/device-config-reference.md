# Device Config Reference

Every device under `models/<DeviceDir>/` carries a YAML config (typically
named `L1config.yaml` or `<variant>_L1config.yaml`). This doc is the schema
reference -- every section, every field, with examples.

For walkthroughs see [how-to/add-a-device.md](how-to/add-a-device.md).

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
contact: ...
sensors: ...
sensor_removals: ...
```

Only `device` and `attachments` are required. Every other section defaults
to empty.

## `device`

```yaml
device:
  name: "DephyExoBoot_L1"
  model_xml: "L1model.xml"
  compatible_msk: ["myolegs22", "myolegs26"]   # optional
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `name` | string | yes | Namespace prefix applied to every body / site / mesh / joint / actuator / tendon imported from the device XML. Convention: PascalCase + `_L1` suffix (e.g. `DephyExoBoot_L1`, `OpenSourceLeg_A_L1`). |
| `model_xml` | path | yes | Path to the device's MuJoCo XML, relative to this YAML file. |
| `compatible_msk` | list | no | Restricts which MSKs this device may combine with. If absent, default to compatible with all. |

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
| `parent_body` | string | yes | Name of the MSK body the device body attaches under. The special value `world` (or `worldbody`) grafts the device body directly under worldbody, keeping its own `<freejoint>` -- see [Free-rooted attachment](#free-rooted-attachment-parent_body-world). |
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
  myolegs:
    - device_body: "hmedi_torso"
      parent_body: "pelvis"
      pos: [-0.105, 0.08, 0]
    # ...repeat any other attachments unchanged
```

The resolver returns the matching MSK key's list if present, else `default`.
Use this when even one attachment needs to differ per MSK.

### Free-rooted attachment (`parent_body: world`)

Most devices are *rigidly re-parented* onto a leg body. A device that is
physically a separate mechanism strapped to the leg -- e.g. the UT ankle exo,
a parallel linkage clamped at several points -- instead attaches to `world`
and is tied to the leg with [`equality`](#equality) constraints:

```yaml
attachments:
  - device_body: "part3_dx"      # a top-level body carrying its own <freejoint>
    parent_body: "world"
    pos: [-0.1574, 0.0345, -0.5832]   # world pose of the free root at qpos0
    quat: [0.1209, 0.0, 0.0, 0.9927]
```

The device body must declare a `<freejoint>` in the device XML (otherwise it
would be welded to the world, immobile). `pos` / `quat` are the world pose the
free root starts at -- place it so the exo sits on the leg, because MuJoCo's
`connect` constraints (below) pin the two bodies at *whatever* relative pose
holds at compile time. Per-MSK `attachments` are usually needed here, since
each baseline places the leg in a different world frame.

## `equality`

Add MuJoCo `<equality>` constraints tying a device body to an MSK body. This
is how a free-rooted device (attached to `world`) is fastened to the leg --
the counterpart to the rigid re-parenting that plain `attachments` perform.
Emitted after attachment, so both endpoints exist.

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
| `device_body` | string | connect/weld | Device body (namespaced with the device prefix at combine time). |
| `parent_body` | string | connect/weld | MSK body (left bare). |
| `joint1` / `joint2` | string | joint | Coupled joints, each resolved bare-first then prefixed. |
| `polycoef` | up to 5 floats | no (joint) | Quartic coefficients; unspecified trailing terms are zero. |
| `anchor` | `[x, y, z]` | connect | Connection point in the device body's local frame. |
| `relpose` | `[x, y, z, qw, qx, qy, qz]` | no (weld) | Relative pose; defaults to identity. |
| `torquescale` | float | no (weld) | Weld torque scale; defaults to 1.0. |
| `solref` / `solimp` | list | no | Constraint-solver knobs; default to MuJoCo's. |
| `active` | bool | no | Whether the constraint starts active (default `true`). |

Supports the per-MSK `default:` + `<msk_key>:` form, like `attachments`.

> **connect anchors record their coincidence at compile.** A `connect`
> introduces *no* initial violation regardless of placement -- it pins the two
> bodies at their qpos0 relative pose. Getting the exo to sit correctly is
> therefore about the *attachment* `pos`/`quat`, not the anchor.

### `type: joint` -- closing a kinematic loop

MuJoCo's body graph is strictly a tree, so a closed linkage cannot be expressed
by nesting alone. A joint equality couples two scalar joints by a quartic:

```
y - y0 = a0 + a1*(x - x0) + a2*(x - x0)^2 + a3*(x - x0)^3 + a4*(x - x0)^4
```

with `y` = `joint1`, `x` = `joint2`, and `x0`/`y0` their reference (qpos0)
values. Omitting `joint2` pins `joint1` to the constant in `polycoef[0]`.

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

`STRIDE_L2` is the worked example: five couplings per side, four closing the
six-bar and one tying its master hinge to `ankle_angle`.

> **Prefer a joint coupling over `connect` for a loop.** A `connect` imposes
> three constraints where two suffice, and the redundancy shows up as drift.
> Measured on STRIDE: 3.7 mm of link separation with `connect` versus 0.020 mm
> with joint couplings.

> **Loops want stiff solver settings.** With MuJoCo's defaults the links visibly
> drift through the range of motion. `solimp: [0.9999, 0.9999, 0.001, 0.5, 2]`
> with `solref: [0.002, 1]` holds the STRIDE loop to single-digit microradians.

> **Restrict the driven joint to the fit window.** A quartic fitted over a
> mechanism's travel extrapolates wildly outside it, and the extrapolated target
> then fights the linkage joints' own limits. Clamp the MSK joint with a
> `joint_overrides` range -- intersected with the range the MSK already declares,
> so the device never *widens* an anatomical limit.

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
only -- tendon-transmission actuators are authored in the device XML directly.

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
  myolegs:
    squat:
      osl_ankle_angle_r: 0.393
```

Use when joint names differ across MSKs (e.g. `pelvis_ty` doesn't exist in
80-muscle, which uses a freejoint root).

## `keyframes` (legacy)

Replace keyframes entirely with explicit qpos/qvel arrays. Model-specific
(must match `nq` / `nv` exactly). Avoid unless you really need to author
full arrays -- use `keyframe_overrides` instead for model-agnostic patches.

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
  myolegs:
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
  myolegs: []                      # explicitly no mods on 80
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

Per-MSK supported. Use `myolegs: []` to disable mods on 80 when the
default block references 22/26-specific tendon names.

## `mesh_replacements`

Swap a geom's mesh to a replacement mesh defined in the device XML's
`<asset>` section.

```yaml
mesh_replacements:
  default:
    - geom: "tibia_r_geom_1"
      mesh: "osl_tibia_fibula_trans_r"        # device-XML mesh name; prefix added automatically
  myolegs:
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

Geom removal -- for cases where `mesh_replacements` swaps one geom
on a body but a sibling geom needs to disappear too.

```yaml
geom_removals:
  default:
    - "tibia_r_geom_2"     # fibula geom (the residual stump mesh covers both bones)
  myolegs:
    - "r_fibula"
```

Use: transtibial amputation where the residual mesh covers
tibia + fibula but the MSK had them as two separate geoms. The strip
cascades into contact pair cleanup.

Per-MSK supported.

## `body_overrides`

Override the inertial properties of a body in the combined model. Targets an MSK
body (bare name) or a device body (resolved with the device prefix). Only the
fields given change.

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
| `name` | string | yes | Body to modify (bare MSK name, or device name resolved with the prefix). |
| `mass` | float | no | New mass, kg. |
| `diaginertia` | `[Ixx, Iyy, Izz]` | no | Principal moments, in the frame set by `iquat`. Mutually exclusive with `fullinertia`. |
| `fullinertia` | `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]` | no | Full tensor. Mutually exclusive with `diaginertia`. |
| `ipos` | `[x, y, z]` | no | Inertial-frame origin, in the body frame. |
| `iquat` | `[qw, qx, qy, qz]` | no | Inertial-frame orientation. |

Supports the per-MSK `default:` + `<msk_key>:` form.

This is the mass-side counterpart to `mesh_replacements`. Amputation is the
motivating case: `spec.delete` removes the distal subtree, but the surviving
parent body still carries the *whole* intact segment's mass -- MuJoCo has no
notion that the segment was transected. All three transtibial devices
(`KFoot_L1`, `OpenSourceLeg_A_L1`, `NEUankle_L1`) reduce `tibia_r` this way;
without it the prosthetic side carries roughly 1.85 kg of phantom mass.

> **Setting `mass` alone on a compiler-derived-inertia body raises.** MuJoCo
> derives a body's inertia from its geoms unless the body is `explicitinertial`.
> Marking it explicit while supplying only a mass would silently leave the inertia
> at zero, so that combination is rejected -- supply an inertia too.

## `contact`

Add MuJoCo `<contact>` entries to the combined model. Emitted after attachment,
so device geoms and bodies exist to reference; every name resolves bare-first
then prefixed.

```yaml
contact:
  pairs:
    - {geom1: "shank_r_main_geom", geom2: "femur_r_geom_1"}
  excludes:
    - ["foot_r", "calcn_r"]          # two-item list form
    - {body1: "link_gcf_l", body2: "calcn_l"}   # or explicit keys
```

`pairs` fields: `geom1`, `geom2` (required), plus optional `condim`, `margin`,
`gap`, `friction`, `solref`, `solimp`. `excludes` take two body names.

Both sub-sections support the per-MSK `default:` + `<msk_key>:` form -- which
matters, because geom names differ between lineages (the 80-muscle models call
the femur mesh geom `femur_r`, the 26-muscle ones `femur_r_geom_1`) and some
bodies only exist on some MSKs.

Needed because `attach_body` copies a body subtree, not top-level sections, so a
device XML's own `<contact>` block never migrates.

> **Reach for `contype`/`conaffinity` before reaching for excludes.** MuJoCo
> already skips geoms in the same weld group and in parent-child body pairs, so a
> device whose bodies are welded onto the MSK (no joints of its own) needs no
> excludes at all. A device with joints does: its links are separate weld groups,
> and grandparent/sibling pairs are live candidates. Rather than one exclude per
> pair, give every device geom `contype="2" conaffinity="1"` -- device-vs-device
> tests `2 & 1 = 0` both ways and never collides, while bone and ground
> (`contype=1`) still do. That replaced ~30 excludes per side on `STRIDE_L2`.
> The cost: device parts no longer collide with each other at all, including
> left-vs-right.

> **An `exclude` silently cancels a `pair`.** Excludes act at body level and win.
> If a forced pair stops generating contacts, check whether something excluded its
> owning bodies.

> **A welded device body inherits its host's weld group.** So a shank-mounted
> part is auto-excluded from the *femur* (its weld group's parent) and will
> interpenetrate the thigh with no contact generated. That needs an explicit
> `pair`, which bypasses `contype`/`conaffinity` filtering entirely.

## `sensors` / `sensor_removals`

Add sensors to, and remove sensors from, the combined model.

```yaml
sensor_removals: ["r_foot", "r_toes"]

sensors:
  - {name: "r_foot", type: touch, site: "r_sole_touch"}
  - {name: "r_ankle_sensor", type: jointlimitfrc, joint: "neuankle_ankle_angle_r"}
```

Each sensor names exactly one target, and the key used selects the target kind:
`site`, `joint`, `actuator`, `tendon`, `body` or `geom`. Optional `cutoff` and
`noise` pass through. Supported `type` values: `touch`, `force`, `torque`,
`jointpos`, `jointvel`, `jointlimitpos`, `jointlimitvel`, `jointlimitfrc`,
`jointactuatorfrc`, `actuatorpos`, `actuatorvel`, `actuatorfrc`, `tendonpos`,
`tendonvel`, `framepos`, `framequat`, `framelinvel`, `frameangvel`. A type that
does not accept the target kind you gave raises.

Both sections support the per-MSK `default:` + `<msk_key>:` form. That matters
more than it looks: `myolegs26` and `myolegs22` ship twelve sensors including
`jointlimitfrc` on every leg joint, while `myolegs` and `myofullbody` ship only
the four leg touch sensors. Restoring a `jointlimitfrc` on the latter would make
it the only sensor of its kind in the model.

Two distinct uses:

- **Restoring what surgery cascaded away.** Deleting `talus_r` also deletes
  `r_foot`/`r_toes` (touch sensors on the removed sites) and
  `r_ankle_sensor`/`r_mtp_sensor`, dropping the baseline's twelve sensors to
  eight and leaving nothing reading the prosthetic side while the intact side
  keeps all four counterparts.
- **Re-pointing a sensor**, via removal plus re-addition under the same name.
  A shod device moves ground contact from the bare foot onto the sole, and the
  baseline touch site is a *box*: `r_foot_touch` spans y in [-0.022, 0.018] on
  `calcn_r` while a shoe sole's contact surface sits at y = -0.058, 35 mm below
  the bottom of the box. A touch sensor left on the baseline site reads exactly
  zero for the whole stance phase.

> **Sensors are appended, not inserted.** The combined model's `sensordata`
> ordering therefore differs from the baseline's. Index by name, never position.

> **Name a device site distinctly if the MSK still has one by that name.**
> Targets resolve bare-first, so a device site called `r_foot_touch` is silently
> shadowed by the surviving MSK site of the same name and the sensor keeps reading
> the bare foot. `STRIDE_L2` names its sole sites `r_sole_touch` /
> `r_forefoot_touch` for exactly this reason, while keeping the *sensor* named
> `r_foot` so downstream consumers are unaffected.

## Per-MSK overrides -- summary

Sections that support the `default:` + `<msk_key>:` dispatch:

| Section | Per-MSK? |
|---|---|
| `attachments` | ✓ |
| `equality` | ✓ |
| `joint_overrides` | ✓ |
| `actuators` | (planned; currently default form only) |
| `keyframe_overrides` | ✓ |
| `body_removals` | (planned; currently default form only) |
| `mesh_replacements` | ✓ |
| `actuator_removals` | ✓ |
| `tendon_removals` | ✓ |
| `tendon_modifications` | ✓ |
| `geom_removals` | ✓ |
| `body_overrides` | ✓ |
| `contact` (`pairs` + `excludes`) | ✓ |
| `sensors` | ✓ |
| `sensor_removals` | ✓ |

Sections marked "planned" use the flat list form for now; per-MSK support
is incremental as configs need it.

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
   often omit the prefix -- the resolver tries both.
4. **For unnamed joint inside a body, you still must remove the body if
   you want it gone.** Removals are by body name; the joints come along.
