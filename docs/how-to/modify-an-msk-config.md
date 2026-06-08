# How To: Add or Modify a Per-MSK Override

When a device's behavior must differ between MSKs (different tendon names,
different parent bodies, different joint values), use the per-MSK override
form. This guide walks through the schema and a worked example.

## When to reach for per-MSK overrides

Use them when:

- **Tendon / actuator names differ across MSKs** (22/26 share names, 80
  has a different scheme).
- **Attachment topology differs across MSKs** (the parent body for a
  device part isn't the same in 22 vs 80).
- **Keyframe joints differ** (22/26 have `pelvis_ty`; 80 doesn't).
- **Mesh replacement geoms differ** (geom names differ across MSKs).

Don't use them when:

- A single block works for all MSKs -- keep it as the flat-list form.
- The difference is small enough to handle in the device XML itself
  (e.g. a body's `pos`/`quat` in the device XML rarely needs per-MSK
  variation).

## Sections that support per-MSK overrides

| Section | Per-MSK |
|---|---|
| `attachments` | ✓ |
| `keyframe_overrides` | ✓ |
| `mesh_replacements` | ✓ |
| `actuator_removals` | ✓ |
| `tendon_removals` | ✓ |
| `tendon_modifications` | ✓ |
| `geom_removals` | ✓ |

See [device-config-reference.md](../device-config-reference.md#per-msk-overrides-summary)
for the running list -- sections are migrated as real configs demand them.

## Schema shape

Two forms in YAML:

**Flat list** (applies to all MSKs):

```yaml
actuator_removals:
  - "soleus_r"
  - "tibant_r"
```

**Per-MSK** (`default:` + one or more MSK keys):

```yaml
actuator_removals:
  default:
    - "soleus_r"
    - "tibant_r"
  myoLeg80:
    - "soleus_r"
    - "tibant_r"
    - "gaslat_r"      # 80 splits gastroc into gaslat + gasmed
    - "gasmed_r"
```

The resolver picks the matching MSK key's list when `msk_key=...` is
passed to `load_combined_model`, else falls back to `default`. If neither
matches, the section is empty.

## Worked example: OSL_KA transfemoral tendon repositioning

The OSL_KA prosthetic removes `tibia_r` and everything below. Two tendons
that survive (rect_fem_r, vasti_r) have wrap sites that lived on bodies
under `tibia_r`; they're auto-pruned, but the legacy 22-muscle OSL_KA
also *re-anchors* them onto the residual femur for anatomical accuracy.

In `models/OpenSourceLeg/KA_L1config.yaml`:

```yaml
tendon_modifications:
  myoLeg80: []           # 80 uses different tendon names; nothing to do here
  default:               # 22/26 only
    - name: "rect_fem_r_tendon"
      wraps:
        - reposition_site: "rect_fem_r_rect_fem_r-P2"
          pos: [0.045, -0.2, 0.005]              # on femur_r, anatomical
        - replace_site: "rect_fem_r_rect_fem_r-P3"
          new_body: "femur_r"                    # re-anchor onto femur
          pos: [0.025, -0.275, 0.0075]
    - name: "vasti_r_tendon"
      wraps:
        - reposition_site: "vasti_r_vas_int_r-P3"
          pos: [0.03, -0.275, 0.0095]
```

Why `myoLeg80: []` (empty list) instead of just omitting it?

- If only `default:` is provided, every MSK gets that block.
- For 80, the tendon names `rect_fem_r_tendon` / `vasti_r_tendon` don't
  exist (80 uses `recfem_r_tendon` / `vasint_r_tendon` etc.) -- applying
  the default would raise `unknown tendon`.
- The empty `myoLeg80: []` opts 80 out cleanly without affecting 22/26.

## Worked example: HMEDI per-MSK attachment

HMEDI's `hmedi_torso` part attaches differently on each MSK. In 22/26
the torso body is a child of pelvis, so we attach directly to `torso`.
In 80 the torso lives under a yaw-rotated `root` body, so attaching to
`torso` produces a misaligned mesh. The legacy 80-muscle HMEDI bypasses
this entirely by attaching `hmedi_torso` to `pelvis` directly with a
different offset.

```yaml
attachments:
  default:
    - device_body: "hmedi_torso"
      parent_body: "torso"
    - device_body: "hmedi femurflap_r"
      parent_body: "femur_r"
    # ...
  myoLeg80:
    - device_body: "hmedi_torso"
      parent_body: "pelvis"           # different parent
      pos: [-0.105, 0.08, 0]          # frame offset to compensate
    - device_body: "hmedi femurflap_r"
      parent_body: "femur_r"
    # ...repeat the rest unchanged
```

When using the per-MSK form for attachments, **list every attachment in
each block** -- the resolver returns the whole list, not a diff. If you
forget an attachment in the per-MSK block, that part won't be attached
on that MSK.

## Testing per-MSK overrides

In Python:

```python
from assist_sim import DeviceConfig

config = DeviceConfig.from_yaml("models/MyDevice/L1config.yaml")
default_atts = config.resolve_attachments()
msk80_atts = config.resolve_attachments("myoLeg80")
assert default_atts != msk80_atts
```

End-to-end:

```bash
python examples/quickstart.py myoLeg80 MyDevice_L1
python examples/quickstart.py myoLeg22_2D MyDevice_L1
```

Both should compile and look right.

## Common pitfalls

1. **Forgetting `myoLeg80: []`** when the default block uses 22/26-only
   names → `ValueError: unknown tendon` on 80.
2. **Forgetting an attachment in the per-MSK block** → that device part
   floats free in the compiled model.
3. **Using a prefix in `keyframe_overrides`** for an MSK-side joint:
   write the bare MSK joint name (`pelvis_ty`), not the prefixed device
   name. The resolver tries the bare name first.

## See also

- [device-config-reference.md](../device-config-reference.md) -- the full
  schema, including which sections support per-MSK
- [how-to/debug-a-combined-model.md](debug-a-combined-model.md)
