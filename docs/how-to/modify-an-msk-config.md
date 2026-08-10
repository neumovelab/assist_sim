# How To: Add or Modify a Per-MSK Override

The behavior of a device can differ between musculoskeletal (MSK) models.
The tendon names, the parent bodies or the joint values can differ. Use the
per-MSK override form for these cases. This guide describes the schema and
gives a worked example.

## When to use per-MSK overrides

Use them when:

- **The tendon or actuator names differ between MSK models.** Models 22 and
  26 share the names. Model 80 uses a different scheme.
- **The attachment topology differs between MSK models.** The parent body
  for a device part is not the same in 22 and in 80.
- **The keyframe joints differ.** Models 22 and 26 have `pelvis_ty`. Model
  80 does not have it.
- **The mesh replacement geoms differ.** The geom names differ between MSK
  models.
- **The removals differ.** Only the 80-muscle models have a `patella_r`,
  and it is a sibling of `tibia_r`. A transfemoral `body_removals` must
  therefore name it.
- **The muscles to re-anchor differ.** Model 80 splits the lumped muscles
  of 22 and 26. `tendon_modifications` and the `actuator_overrides` that
  follow it are therefore both per-MSK.

Do not use them when:

- A single block is correct for all MSK models. Keep the flat-list form.
- The difference is small enough for the device XML itself. For example,
  the `pos` and `quat` of a body in the device XML seldom need a per-MSK
  variation.

## Sections that support per-MSK overrides

| Section | Per-MSK |
|---|---|
| `attachments` | ✓ |
| `equality` | ✓ |
| `joint_overrides` | ✓ |
| `keyframe_overrides` | ✓ |
| `body_removals` | ✓ |
| `geom_removals` | ✓ |
| `actuator_removals` | ✓ |
| `tendon_removals` | ✓ |
| `sensor_removals` | ✓ |
| `tendon_modifications` | ✓ |
| `actuator_overrides` | ✓ |
| `body_overrides` | ✓ |
| `mesh_replacements` | ✓ |
| `contact` (`pairs` + `excludes`) | ✓ |
| `sensors` | ✓ |
| `actuators` | flat list only |
| `keyframes` (legacy) | flat list only |

Each section with a ✓ uses the same parser (`_parse_per_msk_list` in
`assist_sim/config.py`). The two forms below therefore apply to all of
them. See
[device-config-reference.md](../device-config-reference.md#per-msk-overrides-summary)
for the text about each section.

## Schema shape

YAML has two forms.

**Flat list** (applies to all MSK models):

```yaml
actuator_removals:
  - "soleus_r"
  - "tibant_r"
```

**Per-MSK** (`default:` plus one or more MSK keys):

```yaml
actuator_removals:
  default:
    - "soleus_r"
    - "tibant_r"
  myolegs:
    - "soleus_r"
    - "tibant_r"
    - "perbrev_r"     # 80 splits the lumped muscles, so it names more
    - "perlong_r"
```

The resolver selects the block whose key agrees with the MSK model that it
builds. If there is no such block, the resolver selects the `default`
block. `load_combined("myolegs26", ...)` gives the key for you.
`load_combined_model` accepts the key as `msk_key=`.

An MSK model with no block of its own gets `default`. If you also omit
`default:`, that MSK model gets nothing at all from the section.
(`attachments` is the exception. In the dict form it needs a `default:`
entry.)

The fallback causes a usual problem: `myofullbody` shares the 80-muscle leg
names, so it needs the `myolegs` block, not the default block. Give it a
YAML alias:

```yaml
  myolegs: &r80
    - "bfsh_r"
  myofullbody: *r80
```

## Worked example: OSL_KA transfemoral re-anchoring

`tendon_modifications` is the re-anchor step (myodesis). It is the one
section that runs **before** every removal, and this order is necessary.

The OSL_KA prosthetic removes `tibia_r` and all the bodies below it.
`spec.delete` cascades: it also removes each tendon and actuator whose wrap
points were on a removed body. A real transfemoral amputation does not
destroy those muscles. The surgeon re-attaches the biarticular muscles to
the residual femur, where they continue to act at the hip.
`tendon_modifications` does the same in the model. It must do this while
the wrap points still exist, because nothing remains to move after the
cascade.

From `assist_sim/models/OpenSourceLeg/KA_L1config.yaml`:

```yaml
tendon_modifications:
  default:                              # 22/26-muscle lineage
    - name: "rect_fem_r_tendon"         # hip flexion survives
      wraps:
        - reposition_site: "rect_fem_r_rect_fem_r-P2"
          pos: [0.045, -0.2, 0.005]     # already on femur_r, moved proximal
        - replace_site: "rect_fem_r_rect_fem_r-P3"
          new_body: "femur_r"           # was on a body the cascade removes
          pos: [0.025, -0.275, 0.0075]
    - name: "hamstrings_r_tendon"       # hip extension survives
      wraps:
        - replace_site: "hamstrings_r_semimem_r-P2"
          new_body: "femur_r"
          pos: [0.01259, -0.265, 0.01207]
        - replace_site: "hamstrings_r_semimem_r-P3"
          new_body: "femur_r"
          pos: [0.01259, -0.28301, 0.01207]

  myolegs: &r80_reanchor                # 80 splits the lumped muscles: 8 blocks
    - name: "semimem_r_tendon"
      wraps:
        - replace_geom: "SM_at_condyles_wrap_r"   # the wrap cylinder moves too
          new_body: "femur_r"
          pos: [0.01464, -0.270, 0.00916]
        - replace_site: "SM_at_condyles_site_semimem_r"
          new_body: "femur_r"
          pos: [0.01259, -0.270, 0.01207]
        - replace_site: "semimem-P2_r"
          new_body: "femur_r"
          pos: [0.01259, -0.28301, 0.01207]
    # ...bflh_r, semiten_r, grac_r, sart_r, tfl_r, addmagIsch_r, recfem_r
  myofullbody: *r80_reanchor            # its leg is identical to myolegs's
```

### The four ops

| Op | Effect | Required keys |
|---|---|---|
| `reposition_site` | moves a wrap site on the body where it already is | `pos` |
| `replace_site` | moves a wrap site onto `new_body` | `pos`, `new_body` |
| `reposition_geom` | moves a wrap cylinder on its current body | `pos` |
| `replace_geom` | moves a wrap cylinder onto `new_body` | `pos`, `new_body` |

`drop_site` is retired and gives a `ValueError`. MuJoCo has no editable
wrap list, so you cannot remove a wrap. To remove a muscle, use
`actuator_removals` plus `tendon_removals`.

### How the wrap follows the element

A wrap stores its site or geom by *name* and resolves that name at compile
time. `reposition_*` sets `pos` on the element. `replace_*` builds the
element again on the new body, removes the original, then gives the free
name to the replacement. In both cases the wrap follows the element, and
the pipeline makes no extra site. The tendon objects and the actuator
objects stay in position, so the `ctrl` indices keep their order.

### Four rules that the example obeys

1. **Move every wrap point at the cut plane or distal to it.** Include the
   points that are already on the bone that stays. `sart-P2_r` sits on
   `femur_r`, but at y = -0.357. This position is distal to the cut at
   y = -0.283012, so the example repositions the point.
2. **Move the wrap cylinder, not only the sites.** One geom on a removed
   body still removes the whole tendon. `replace_geom` exists for this
   problem (see
   `tests/test_tendon_reanchor.py::test_a_wrap_geom_left_behind_still_cascades_the_tendon`).
3. **Put the moved points at different positions along the residual bone.**
   Two points at one position give a tendon segment of zero length.
4. **Give the muscle a new `lengthrange`.** The compiler keeps the authored
   value, because `LRopt.useexisting` is 1. A re-anchored muscle otherwise
   describes a path that it no longer has, and it generates incorrect
   forces. Put the new values in `actuator_overrides`, which is per-MSK for
   the same reason:

```yaml
actuator_overrides:
  myolegs26:
    - name: "rectfem_r"
      lengthrange: [0.226056, 0.330193]
    - name: "hamstrings_r"
      lengthrange: [0.238041, 0.343585]
```

Derive the pair of values from a kinematic sweep of the joints along the
re-anchored path. Do not use `mj_setLengthRange`, which ignores the joint
limits.

### How to opt an MSK model out

An empty list opts one MSK model out of a section and does not change the
other MSK models:

```yaml
tendon_modifications:
  default:
    - name: "rect_fem_r_tendon"        # a 22/26-only tendon name
      # ...
  myolegs: []                          # 80 has no tendon of that name
```

If you omit the key, `myolegs` gets the default block, and the unknown
tendon name gives an error.

## Worked example: HMEDI per-MSK attachment

The `hmedi_torso` part of HMEDI attaches differently on each MSK model. In
22 and 26 the torso body is a child of the pelvis, so the config attaches
the part directly to `torso`. In 80 the torso is below a yaw-rotated `root`
body, so an attachment to `torso` gives a misaligned mesh. The legacy
80-muscle HMEDI prevents this problem completely. It attaches `hmedi_torso`
directly to `pelvis` with a different offset.

Caution: the resolver returns the whole list, not a difference. In the
per-MSK form for attachments, **list every attachment in each block**. If
an attachment is absent from the per-MSK block, that part does not attach
on that MSK model.

```yaml
attachments:
  default:
    - device_body: "hmedi_torso"
      parent_body: "torso"
    - device_body: "hmedi femurflap_r"
      parent_body: "femur_r"
    # ...
  myolegs:
    - device_body: "hmedi_torso"
      parent_body: "pelvis"           # different parent
      pos: [-0.105, 0.08, 0]          # frame offset to compensate
    - device_body: "hmedi femurflap_r"
      parent_body: "femur_r"
    # ...repeat the rest unchanged
```

## Test the per-MSK overrides

In Python:

```python
from assist_sim import DeviceConfig

config = DeviceConfig.from_yaml("assist_sim/models/MyDevice/L1config.yaml")
default_atts = config.resolve_attachments()
msk80_atts = config.resolve_attachments("myolegs")
assert default_atts != msk80_atts
```

Every section has a `resolve_*` method with the same shape, for example
`resolve_tendon_modifications("myolegs")` and
`resolve_actuator_overrides("myolegs26")`.

End to end:

```bash
python examples/quickstart.py myolegs MyDevice_L1
python examples/quickstart.py myolegs26 MyDevice_L1
```

Both commands must compile and give a correct model. For a change to a
re-anchor, also run the re-anchor test suite. It checks that the muscles
stay in the model, and that they start inside their operating range:

```bash
pytest tests/test_tendon_reanchor.py -v
```

## Usual problems

1. **No `myolegs: []` block, although the default block uses names from 22
   and 26.** Result: `ValueError: unknown tendon` on 80.
2. **No `myofullbody` key.** Result: `myofullbody` gets the `default` block
   in place of the 80-muscle block, and no message tells you.
3. **An attachment is absent from the per-MSK block.** Result: that device
   part floats free in the compiled model.
4. **A prefix in `keyframe_overrides` for a joint of the MSK model.** Write
   the bare MSK joint name (`pelvis_ty`), not the prefixed device name. The
   resolver tries the bare name first.
5. **A re-anchored muscle keeps its `lengthrange`.** Result: the muscle
   compiles, but its rest length can be outside its own range, and the
   forces are incorrect. Add an `actuator_overrides` entry.
6. **The sites move, but the wrap cylinder stays.** Result: the muscle
   disappears from the combined model, and no message tells you.

## See also

- [device-config-reference.md](../device-config-reference.md): the full
  schema, and the list of the sections that support per-MSK overrides
- [how-to/debug-a-combined-model.md](debug-a-combined-model.md)
