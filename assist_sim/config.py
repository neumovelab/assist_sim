"""Device configuration dataclasses and YAML loader for the mjSpec combination pipeline."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml

_WRAP_OPS = ("drop_site", "reposition_site", "replace_site")


def _parse_per_msk_list(raw_value, parse_item):
    """Parse a section that may be a plain list (default) or a per-MSK dict.

    Returns ``(default_list, by_msk)`` where ``by_msk`` always contains a
    ``"default"`` entry.  A dict value is treated as the per-MSK form::

        section:
          default: [...]
          myolegs: [...]
    """
    if isinstance(raw_value, dict):
        by_msk = {key: [parse_item(x) for x in (items or [])] for key, items in raw_value.items()}
        default = by_msk.get("default", [])
        return default, by_msk
    default = [parse_item(x) for x in (raw_value or [])]
    return default, {"default": default}


def _kf_overrides_from_map(mapping: dict) -> "Dict[str, KeyframeOverride]":
    return {kf: KeyframeOverride(joint_values=jv) for kf, jv in mapping.items() if isinstance(jv, dict)}


def _is_per_msk_keyframe_overrides(raw: dict) -> bool:
    """A per-MSK keyframe_overrides block nests one level deeper than default."""
    for value in raw.values():
        if isinstance(value, dict) and any(isinstance(inner, dict) for inner in value.values()):
            return True
    return False


def _parse_keyframe_overrides(raw_value):
    """Parse keyframe_overrides as default form or per-MSK form.

    Returns ``(default_map, by_msk)`` with a ``"default"`` entry guaranteed.
    """
    if isinstance(raw_value, dict) and _is_per_msk_keyframe_overrides(raw_value):
        by_msk = {key: _kf_overrides_from_map(v) for key, v in raw_value.items()}
        default = by_msk.get("default", {})
        return default, by_msk
    default = _kf_overrides_from_map(raw_value or {})
    return default, {"default": default}


def _parse_wrap_edit(raw: dict) -> "WrapEdit":
    """Parse one wrap-edit dict; exactly one op key selects the operation."""
    ops_present = [op for op in _WRAP_OPS if op in raw]
    if len(ops_present) != 1:
        raise ValueError(f"each wrap edit must have exactly one of {_WRAP_OPS}; got {raw}")
    op = ops_present[0]
    site = raw[op]
    new_body = raw.get("new_body")
    pos = raw.get("pos")
    if op == "replace_site" and not new_body:
        raise ValueError(f"replace_site '{site}' requires 'new_body'")
    if op in ("reposition_site", "replace_site") and pos is None:
        raise ValueError(f"{op} '{site}' requires 'pos'")
    return WrapEdit(op=op, site=site, new_body=new_body, pos=pos)


_EQUALITY_TYPES = ("connect", "weld", "joint")


def _parse_equality(raw: dict) -> "EqualityConstraint":
    """Parse one equality-constraint dict from the ``equality:`` section.

    Two shapes, selected by ``type``.  ``connect`` / ``weld`` are body-to-body and
    take ``device_body`` + ``parent_body``; ``joint`` is joint-to-joint and takes
    ``joint1`` + ``joint2`` + ``polycoef``.
    """
    eq_type = raw.get("type", "connect")
    if eq_type not in _EQUALITY_TYPES:
        raise ValueError(f"equality 'type' must be one of {_EQUALITY_TYPES}; got {eq_type!r}")

    if eq_type == "joint":
        joint1 = raw.get("joint1")
        joint2 = raw.get("joint2")
        if not joint1:
            raise ValueError(f"joint equality must have 'joint1'; got {raw}")
        polycoef = raw.get("polycoef")
        if polycoef is not None and len(polycoef) > 5:
            raise ValueError(f"joint equality 'polycoef' takes at most 5 coefficients; got {len(polycoef)} in {raw}")
        return EqualityConstraint(
            type=eq_type,
            joint1=joint1,
            joint2=joint2,
            polycoef=polycoef,
            solref=raw.get("solref"),
            solimp=raw.get("solimp"),
            active=raw.get("active", True),
        )

    device_body = raw.get("device_body")
    parent_body = raw.get("parent_body")
    if not device_body or not parent_body:
        raise ValueError(f"each {eq_type} equality must have 'device_body' and 'parent_body'; got {raw}")
    if eq_type == "connect" and raw.get("anchor") is None:
        raise ValueError(f"connect equality on '{device_body}' requires 'anchor' (len-3)")
    return EqualityConstraint(
        type=eq_type,
        device_body=device_body,
        parent_body=parent_body,
        anchor=raw.get("anchor"),
        relpose=raw.get("relpose"),
        torquescale=raw.get("torquescale"),
        solref=raw.get("solref"),
        solimp=raw.get("solimp"),
        active=raw.get("active", True),
    )


# Sensor types the ``sensors:`` section can emit, mapped to the target kinds each
# accepts.  The MuJoCo enum is resolved in ``combine`` (config.py stays
# mujoco-free); the value here is the ``mjtSensor`` member name.
_SENSOR_TYPES: Dict[str, tuple] = {
    "touch": ("mjSENS_TOUCH", ("site",)),
    "force": ("mjSENS_FORCE", ("site",)),
    "torque": ("mjSENS_TORQUE", ("site",)),
    "jointpos": ("mjSENS_JOINTPOS", ("joint",)),
    "jointvel": ("mjSENS_JOINTVEL", ("joint",)),
    "jointlimitpos": ("mjSENS_JOINTLIMITPOS", ("joint",)),
    "jointlimitvel": ("mjSENS_JOINTLIMITVEL", ("joint",)),
    "jointlimitfrc": ("mjSENS_JOINTLIMITFRC", ("joint",)),
    "jointactuatorfrc": ("mjSENS_JOINTACTFRC", ("joint",)),
    "actuatorpos": ("mjSENS_ACTUATORPOS", ("actuator",)),
    "actuatorvel": ("mjSENS_ACTUATORVEL", ("actuator",)),
    "actuatorfrc": ("mjSENS_ACTUATORFRC", ("actuator",)),
    "tendonpos": ("mjSENS_TENDONPOS", ("tendon",)),
    "tendonvel": ("mjSENS_TENDONVEL", ("tendon",)),
    "framepos": ("mjSENS_FRAMEPOS", ("body", "site", "geom")),
    "framequat": ("mjSENS_FRAMEQUAT", ("body", "site", "geom")),
    "framelinvel": ("mjSENS_FRAMELINVEL", ("body", "site", "geom")),
    "frameangvel": ("mjSENS_FRAMEANGVEL", ("body", "site", "geom")),
}

# Every key that can name a sensor's target, in the order they're probed.
_SENSOR_TARGET_KINDS = ("site", "joint", "actuator", "tendon", "body", "geom")


def _parse_sensor(raw: dict) -> "SensorDef":
    """Parse one sensor dict; exactly one target key selects what it reads."""
    name = raw.get("name")
    sensor_type = raw.get("type")
    if not name or not sensor_type:
        raise ValueError(f"each sensor must have 'name' and 'type'; got {raw}")
    if sensor_type not in _SENSOR_TYPES:
        raise ValueError(f"sensor '{name}' has unsupported type {sensor_type!r}; supported: {sorted(_SENSOR_TYPES)}")

    present = [k for k in _SENSOR_TARGET_KINDS if raw.get(k)]
    if len(present) != 1:
        raise ValueError(f"sensor '{name}' must name exactly one target from {_SENSOR_TARGET_KINDS}; got {present or 'none'}")
    kind = present[0]

    allowed = _SENSOR_TYPES[sensor_type][1]
    if kind not in allowed:
        raise ValueError(f"sensor '{name}' of type '{sensor_type}' reads a {' or '.join(allowed)}, not a {kind}")

    return SensorDef(
        name=name,
        type=sensor_type,
        target_kind=kind,
        target=raw[kind],
        cutoff=raw.get("cutoff"),
        noise=raw.get("noise"),
    )


def _parse_body_override(raw: dict) -> "BodyOverride":
    """Parse one body-override dict from the ``body_overrides:`` section."""
    name = raw.get("name")
    if not name:
        raise ValueError(f"each body override must have 'name'; got {raw}")
    override = BodyOverride(
        name=name,
        mass=raw.get("mass"),
        diaginertia=raw.get("diaginertia"),
        fullinertia=raw.get("fullinertia"),
        ipos=raw.get("ipos"),
        iquat=raw.get("iquat"),
    )
    if override.diaginertia is not None and override.fullinertia is not None:
        raise ValueError(f"body override '{name}' sets both 'diaginertia' and 'fullinertia'; pick one")
    if not any(
        v is not None for v in (override.mass, override.diaginertia, override.fullinertia, override.ipos, override.iquat)
    ):
        raise ValueError(f"body override '{name}' sets nothing; give at least one of mass/diaginertia/fullinertia/ipos/iquat")
    return override


def _parse_exclude(raw) -> "ContactExclude":
    """Parse one contact exclude, as a mapping or a two-item ``[b1, b2]`` list."""
    if isinstance(raw, (list, tuple)):
        if len(raw) != 2:
            raise ValueError(f"a contact exclude list must have exactly two body names; got {raw}")
        return ContactExclude(body1=raw[0], body2=raw[1])
    body1, body2 = raw.get("body1"), raw.get("body2")
    if not body1 or not body2:
        raise ValueError(f"each contact exclude must have 'body1' and 'body2'; got {raw}")
    return ContactExclude(body1=body1, body2=body2)


def _parse_pair(raw: dict) -> "ContactPair":
    """Parse one explicit contact pair from ``contact.pairs``."""
    geom1, geom2 = raw.get("geom1"), raw.get("geom2")
    if not geom1 or not geom2:
        raise ValueError(f"each contact pair must have 'geom1' and 'geom2'; got {raw}")
    return ContactPair(
        geom1=geom1,
        geom2=geom2,
        condim=raw.get("condim"),
        margin=raw.get("margin"),
        gap=raw.get("gap"),
        friction=raw.get("friction"),
        solref=raw.get("solref"),
        solimp=raw.get("solimp"),
    )


@dataclass
class Attachment:
    """Maps a device body to a parent body in the human model.

    Optional ``pos`` (length 3) and ``quat`` (length 4) offset the device body
    on its parent via the attachment frame.  Both default to ``None`` (identity
    frame -- the device XML's own frame is used unchanged).
    """

    device_body: str
    parent_body: str
    pos: Optional[List[float]] = None
    quat: Optional[List[float]] = None


@dataclass
class EqualityConstraint:
    """A MuJoCo ``<equality>`` tying the device to the human model.

    Emitted onto the combined spec *after* attachment, so it can reference both
    device elements (namespaced with the device prefix at combine time) and
    human ones (left bare).  This is how a constraint-clamped device -- e.g. a
    free-floating exo strapped to the leg at several points, or a closed
    linkage whose loop MuJoCo's body tree cannot express -- integrates without
    the rigid re-parenting :class:`Attachment` performs.

    Body-to-body forms (``device_body`` + ``parent_body``):

    - ``type="connect"``: point-to-point (ball) constraint.  ``anchor`` (len 3)
      is the connection point, in ``device_body``'s local frame -- matching
      MJCF ``<connect body1=device body2=human anchor=...>``.
    - ``type="weld"``: fixes the full relative pose.  ``relpose`` (len 7,
      ``pos`` + ``quat``) and ``torquescale`` are optional.

    Joint-to-joint form (``joint1`` + ``joint2`` + ``polycoef``):

    - ``type="joint"``: couples two scalar joints by a quartic,
      ``y - y0 = a0 + a1*(x - x0) + ... + a4*(x - x0)^4`` where ``y`` is
      ``joint1`` and ``x`` is ``joint2``.  Each name is resolved bare-first then
      prefixed, so one entry can couple two device joints (both prefixed) or a
      device joint to a human joint (mixed).  Omitting ``joint2`` pins
      ``joint1`` to a constant.  This is how a closed kinematic loop is closed
      and how a linkage is tied to the biological joint it spans -- two
      constraints where a ``connect`` would impose three.

    ``solref`` / ``solimp`` / ``active`` are optional constraint-solver knobs;
    when ``None`` the MuJoCo defaults are used.  Closed loops generally want a
    stiff pair (e.g. ``solimp: [0.9999, 0.9999, 0.001, 0.5, 2]``) or the links
    visibly drift through the range of motion.
    """

    type: str
    device_body: Optional[str] = None
    parent_body: Optional[str] = None
    joint1: Optional[str] = None
    joint2: Optional[str] = None
    polycoef: Optional[List[float]] = None
    anchor: Optional[List[float]] = None
    relpose: Optional[List[float]] = None
    torquescale: Optional[float] = None
    solref: Optional[List[float]] = None
    solimp: Optional[List[float]] = None
    active: bool = True


@dataclass
class BodyOverride:
    """Overrides the inertial properties of a body in the combined model.

    Targets a human body (bare name) or a device body (resolved with the device
    prefix).  Only the fields given are changed; the rest keep their authored
    values.  This is the counterpart to :class:`MeshReplacement` for the *mass*
    side of prosthetic surgery: amputating a segment leaves the surviving parent
    body carrying the whole intact segment's mass, which has to be reduced to the
    residual limb's.

    ``diaginertia`` (len 3, principal moments in the frame set by ``iquat``) and
    ``fullinertia`` (len 6, ``Ixx Iyy Izz Ixy Ixz Iyz``) are mutually exclusive.
    ``ipos`` / ``iquat`` move the inertial frame.  The target body must already
    carry an explicit ``<inertial>`` unless an inertia is supplied here --
    otherwise setting mass alone would silently zero the compiler-derived
    inertia.
    """

    name: str
    mass: Optional[float] = None
    diaginertia: Optional[List[float]] = None
    fullinertia: Optional[List[float]] = None
    ipos: Optional[List[float]] = None
    iquat: Optional[List[float]] = None


@dataclass
class ContactExclude:
    """A MuJoCo ``<contact><exclude>`` between two bodies in the combined model.

    Each name is resolved bare-first then prefixed, so an exclude can pair two
    device bodies, or a device body with a human one.

    Only needed where geometry overlaps *by design*.  MuJoCo already skips geoms
    in the same weld group and in parent-child body pairs, so a device whose
    bodies are welded onto the human (no joints of its own) needs none of these.
    Prefer a ``contype`` / ``conaffinity`` bit split for blanket
    "device parts never touch each other" filtering -- that costs two attributes
    in the device XML instead of one exclude per pair.
    """

    body1: str
    body2: str


@dataclass
class ContactPair:
    """An explicit MuJoCo ``<contact><pair>`` in the combined model.

    Forces a collision check between two geoms regardless of their
    ``contype`` / ``conaffinity``, and lets the contact parameters be tuned.
    Names are resolved bare-first then prefixed.

    Beware the interaction with :class:`ContactExclude`: an ``exclude`` on the
    owning bodies cancels a ``pair`` between their geoms, silently.
    """

    geom1: str
    geom2: str
    condim: Optional[int] = None
    margin: Optional[float] = None
    gap: Optional[float] = None
    friction: Optional[List[float]] = None
    solref: Optional[List[float]] = None
    solimp: Optional[List[float]] = None


@dataclass
class SensorDef:
    """A sensor to add to the combined model.

    ``target_kind`` records which YAML key named the target (``site`` / ``joint``
    / ``actuator`` / ``tendon`` / ``body`` / ``geom``); ``target`` is that name,
    resolved bare-first then prefixed.

    Needed because surgery cascades: deleting ``talus_r`` also deletes the
    ``r_foot`` / ``r_toes`` touch sensors and the ``r_ankle_sensor`` /
    ``r_mtp_sensor`` limit sensors that referenced the removed subtree, leaving
    the model asymmetric with nothing reading the prosthetic side.  Sensors are
    *appended*, so the combined model's ``sensordata`` ordering differs from the
    baseline's -- index by name, not position.
    """

    name: str
    type: str
    target_kind: str
    target: str
    cutoff: Optional[float] = None
    noise: Optional[float] = None


@dataclass
class JointOverride:
    """Overrides properties of an existing joint in the human model."""

    name: str
    range: Optional[List[float]] = None
    damping: Optional[float] = None
    axis: Optional[List[float]] = None
    pos: Optional[List[float]] = None


@dataclass
class ActuatorDef:
    """Defines a new actuator to add to the combined model."""

    name: str
    type: str
    joint: str
    gaintype: str = "fixed"
    gainprm: List[float] = field(default_factory=lambda: [1, 0, 0])
    biastype: str = "none"
    biasprm: List[float] = field(default_factory=lambda: [0, 0, 0])
    dyntype: str = "none"
    dynprm: List[float] = field(default_factory=lambda: [1, 0, 0])
    ctrlrange: Optional[List[float]] = None
    ctrllimited: bool = False
    gear: List[float] = field(default_factory=lambda: [1.0])


@dataclass
class MeshReplacement:
    """Swap a geom's mesh on the human model (e.g. residual-limb bone)."""

    geom: str
    mesh: str


@dataclass
class WrapEdit:
    """A single edit to one wrap-site on a spatial tendon.

    ``op`` is one of ``drop_site`` / ``reposition_site`` / ``replace_site``.
    ``site`` is the existing wrap-site name being edited.

    - ``drop_site``: remove the wrap entirely.
    - ``reposition_site``: synthesize a new site (named ``{site}__mod``) at
      ``pos`` on the *same* body the original site sits on; the wrap is
      rewritten to reference it.
    - ``replace_site``: same, but the new site is created on ``new_body``.
    """

    op: str
    site: str
    new_body: Optional[str] = None
    pos: Optional[List[float]] = None


@dataclass
class TendonModification:
    """In-place edits to a tendon's wrap path (e.g. after amputation).

    By default, wraps whose sites live on removed bodies are auto-dropped in
    the preprocess layer; ``wraps`` is only needed to re-anchor or reposition
    surviving wraps, or to drop a specific wrap explicitly.
    """

    name: str
    wraps: List[WrapEdit] = field(default_factory=list)


@dataclass
class KeyframeDef:
    """Defines a keyframe for the combined model (legacy full-array mode)."""

    time: float = 0.0
    qpos: Optional[List[float]] = None
    qvel: Optional[List[float]] = None


@dataclass
class KeyframeOverride:
    """Per-joint keyframe patches (model-agnostic mode).

    Only the joints listed are modified; all others keep their baseline
    values from the human model.  Works with any human model that has
    the referenced joint names, regardless of nq.
    """

    joint_values: Dict[str, float] = field(default_factory=dict)


@dataclass
class DeviceConfig:
    """Complete device configuration loaded from a YAML file.

    Bundles the device model XML path with all metadata needed to
    integrate the device into a musculoskeletal model.
    """

    name: str
    model_xml: str
    attachments: List[Attachment]
    compatible_msk: Optional[List[str]] = None
    equalities: List[EqualityConstraint] = field(default_factory=list)
    joint_overrides: List[JointOverride] = field(default_factory=list)
    actuators: List[ActuatorDef] = field(default_factory=list)
    keyframe_overrides: Dict[str, KeyframeOverride] = field(default_factory=dict)
    keyframes: Dict[str, KeyframeDef] = field(default_factory=dict)
    body_removals: List[str] = field(default_factory=list)
    mesh_replacements: List[MeshReplacement] = field(default_factory=list)
    actuator_removals: List[str] = field(default_factory=list)
    tendon_removals: List[str] = field(default_factory=list)
    tendon_modifications: List[TendonModification] = field(default_factory=list)
    geom_removals: List[str] = field(default_factory=list)
    body_overrides: List[BodyOverride] = field(default_factory=list)
    contact_excludes: List[ContactExclude] = field(default_factory=list)
    contact_pairs: List[ContactPair] = field(default_factory=list)
    sensors: List[SensorDef] = field(default_factory=list)
    sensor_removals: List[str] = field(default_factory=list)

    # Per-MSK override maps (each guaranteed a "default" entry). Populated by
    # from_yaml; resolve_* methods select the matching MSK key or fall back.
    _tendon_modifications_by_msk: Dict[str, List["TendonModification"]] = field(default_factory=dict, repr=False)
    _actuator_removals_by_msk: Dict[str, List[str]] = field(default_factory=dict, repr=False)
    _keyframe_overrides_by_msk: Dict[str, Dict[str, "KeyframeOverride"]] = field(default_factory=dict, repr=False)
    _mesh_replacements_by_msk: Dict[str, List["MeshReplacement"]] = field(default_factory=dict, repr=False)
    _tendon_removals_by_msk: Dict[str, List[str]] = field(default_factory=dict, repr=False)
    _attachments_by_msk: Dict[str, List["Attachment"]] = field(default_factory=dict, repr=False)
    _geom_removals_by_msk: Dict[str, List[str]] = field(default_factory=dict, repr=False)
    _equalities_by_msk: Dict[str, List["EqualityConstraint"]] = field(default_factory=dict, repr=False)
    _body_overrides_by_msk: Dict[str, List["BodyOverride"]] = field(default_factory=dict, repr=False)
    _contact_excludes_by_msk: Dict[str, List["ContactExclude"]] = field(default_factory=dict, repr=False)
    _contact_pairs_by_msk: Dict[str, List["ContactPair"]] = field(default_factory=dict, repr=False)
    _sensors_by_msk: Dict[str, List["SensorDef"]] = field(default_factory=dict, repr=False)
    _sensor_removals_by_msk: Dict[str, List[str]] = field(default_factory=dict, repr=False)
    _joint_overrides_by_msk: Dict[str, List["JointOverride"]] = field(default_factory=dict, repr=False)

    # Resolved at load time -- absolute path to the device XML
    _config_dir: Path = field(default=Path("."), repr=False)

    @property
    def model_xml_path(self) -> Path:
        """Absolute path to the device model XML, resolved relative to the config file."""
        return (self._config_dir / self.model_xml).resolve()

    # ------------------------------------------------------------------
    # Per-MSK resolution
    # ------------------------------------------------------------------

    def resolve_body_removals(self, msk_key: Optional[str] = None) -> List[str]:
        """Body removals for the given MSK (default form returns the list)."""
        return self.body_removals

    @staticmethod
    def _resolve(by_msk: dict, msk_key: Optional[str], fallback):
        """Pick the per-MSK entry, else the 'default' entry, else fallback."""
        if msk_key is not None and msk_key in by_msk:
            return by_msk[msk_key]
        if "default" in by_msk:
            return by_msk["default"]
        return fallback

    def resolve_actuator_removals(self, msk_key: Optional[str] = None) -> List[str]:
        """Actuator removals for the given MSK (per-MSK override or default)."""
        return self._resolve(self._actuator_removals_by_msk, msk_key, self.actuator_removals)

    def resolve_tendon_modifications(self, msk_key: Optional[str] = None) -> List["TendonModification"]:
        """Tendon modifications for the given MSK (per-MSK override or default)."""
        return self._resolve(self._tendon_modifications_by_msk, msk_key, self.tendon_modifications)

    def resolve_keyframe_overrides(self, msk_key: Optional[str] = None) -> Dict[str, "KeyframeOverride"]:
        """Keyframe overrides for the given MSK (per-MSK override or default)."""
        return self._resolve(self._keyframe_overrides_by_msk, msk_key, self.keyframe_overrides)

    def resolve_mesh_replacements(self, msk_key: Optional[str] = None) -> List["MeshReplacement"]:
        """Mesh replacements for the given MSK (per-MSK override or default)."""
        return self._resolve(self._mesh_replacements_by_msk, msk_key, self.mesh_replacements)

    def resolve_tendon_removals(self, msk_key: Optional[str] = None) -> List[str]:
        """Tendon removals for the given MSK (per-MSK override or default)."""
        return self._resolve(self._tendon_removals_by_msk, msk_key, self.tendon_removals)

    def resolve_geom_removals(self, msk_key: Optional[str] = None) -> List[str]:
        """Geom removals for the given MSK (per-MSK override or default).

        Used for surgical geom removal (e.g. dropping the fibula geom that
        survives the tibia mesh-replacement on transtibial amputation).
        """
        return self._resolve(self._geom_removals_by_msk, msk_key, self.geom_removals)

    def resolve_attachments(self, msk_key: Optional[str] = None) -> List["Attachment"]:
        """Attachments for the given MSK (per-MSK override or default).

        Lets a device declare a different attachment frame (``pos`` / ``quat``)
        per MSK, e.g. when the parent body's local frame differs across MSKs
        (myolegs's ``torso`` sits under a yaw-rotated ``root``).
        """
        return self._resolve(self._attachments_by_msk, msk_key, self.attachments)

    def resolve_equalities(self, msk_key: Optional[str] = None) -> List["EqualityConstraint"]:
        """Equality constraints for the given MSK (per-MSK override or default).

        Lets a device pin its constraint anchors to different human bodies (or
        skip some) per MSK, mirroring :meth:`resolve_attachments`.
        """
        return self._resolve(self._equalities_by_msk, msk_key, self.equalities)

    def resolve_body_overrides(self, msk_key: Optional[str] = None) -> List["BodyOverride"]:
        """Body inertial overrides for the given MSK (per-MSK override or default).

        Per-MSK matters here: the 26/22-muscle lineage and the 80-muscle /
        full-body lineage give the same anatomical segment different masses and
        inertial frames, so a residual-limb override is not always transferable.
        """
        return self._resolve(self._body_overrides_by_msk, msk_key, self.body_overrides)

    def resolve_contact_excludes(self, msk_key: Optional[str] = None) -> List["ContactExclude"]:
        """Contact excludes for the given MSK (per-MSK override or default).

        Per-MSK matters here too: bodies that can collide only exist on some MSKs
        (e.g. the arms, which only the full-body model has).
        """
        return self._resolve(self._contact_excludes_by_msk, msk_key, self.contact_excludes)

    def resolve_contact_pairs(self, msk_key: Optional[str] = None) -> List["ContactPair"]:
        """Explicit contact pairs for the given MSK (per-MSK override or default)."""
        return self._resolve(self._contact_pairs_by_msk, msk_key, self.contact_pairs)

    def resolve_sensor_removals(self, msk_key: Optional[str] = None) -> List[str]:
        """Sensor removals for the given MSK (per-MSK override or default).

        Removal + re-add is how a sensor gets *re-pointed*: a shod device moves
        ground contact from the bare foot down onto the sole, so the baseline
        touch site (a box on ``calcn_r``) no longer contains the contact points
        and the sensor silently reads zero.  Drop it in surgery, then re-declare
        it in ``sensors:`` against the device's own site.
        """
        return self._resolve(self._sensor_removals_by_msk, msk_key, self.sensor_removals)

    def resolve_joint_overrides(self, msk_key: Optional[str] = None) -> List["JointOverride"]:
        """Joint overrides for the given MSK (per-MSK override or default).

        Per-MSK matters because joint *ranges* differ between lineages: a device
        that clamps a joint to its own mechanical travel has to intersect that
        travel with whatever range the host model already declares, rather than
        widening it past the anatomical limit.
        """
        return self._resolve(self._joint_overrides_by_msk, msk_key, self.joint_overrides)

    def resolve_sensors(self, msk_key: Optional[str] = None) -> List["SensorDef"]:
        """Sensors for the given MSK (per-MSK override or default).

        Per-MSK matters here as well: the full-body model ships no
        ``jointlimitfrc`` sensors on the legs, so backfilling one for a
        prosthetic ankle would make it the only sensor of its kind.
        """
        return self._resolve(self._sensors_by_msk, msk_key, self.sensors)

    # ------------------------------------------------------------------
    # YAML loading
    # ------------------------------------------------------------------

    @classmethod
    def from_yaml(cls, yaml_path: str) -> "DeviceConfig":
        """Load and validate a device configuration from a YAML file.

        Args:
            yaml_path: Path to the device config.yaml file.

        Returns:
            A fully-populated DeviceConfig instance.

        Raises:
            FileNotFoundError: If the YAML or referenced model XML doesn't exist.
            ValueError: If required fields are missing or malformed.
        """
        yaml_path = Path(yaml_path).resolve()
        if not yaml_path.exists():
            raise FileNotFoundError(f"Device config not found: {yaml_path}")

        with open(yaml_path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)

        if raw is None:
            raise ValueError(f"Empty config file: {yaml_path}")

        config_dir = yaml_path.parent

        # --- device section ---
        device_section = raw.get("device", {})
        name = device_section.get("name")
        model_xml = device_section.get("model_xml")
        if not name or not model_xml:
            raise ValueError("config.yaml must contain device.name and device.model_xml")

        model_xml_abs = (config_dir / model_xml).resolve()
        if not model_xml_abs.exists():
            raise FileNotFoundError(f"Device model XML not found: {model_xml_abs} (referenced from {yaml_path})")

        # --- attachments ---
        # Accept either a flat list (legacy form) or a per-MSK dict where
        # ``default:`` is the fallback and any other key is an MSK-specific
        # override (used for e.g. myolegs body-frame differences).
        raw_attachments = raw.get("attachments", [])
        if not raw_attachments:
            raise ValueError("config.yaml must contain at least one attachment")

        def _parse_attachment_list(items):
            return [
                Attachment(
                    device_body=a["device_body"],
                    parent_body=a["parent_body"],
                    pos=a.get("pos"),
                    quat=a.get("quat"),
                )
                for a in items
            ]

        attachments_by_msk: Dict[str, List[Attachment]] = {}
        if isinstance(raw_attachments, dict):
            if "default" not in raw_attachments:
                raise ValueError("attachments dict form must include a 'default' entry")
            for msk_key, items in raw_attachments.items():
                attachments_by_msk[msk_key] = _parse_attachment_list(items)
            attachments = attachments_by_msk["default"]
        else:
            attachments = _parse_attachment_list(raw_attachments)

        # --- equality constraints (default list or per-MSK dict) ---
        equalities, equalities_by_msk = _parse_per_msk_list(raw.get("equality", []), _parse_equality)

        # --- joint overrides (default list or per-MSK dict) ---
        def _parse_joint_override(j):
            return JointOverride(
                name=j["name"],
                range=j.get("range"),
                damping=j.get("damping"),
                axis=j.get("axis"),
                pos=j.get("pos"),
            )

        joint_overrides, joint_overrides_by_msk = _parse_per_msk_list(raw.get("joint_overrides", []), _parse_joint_override)

        # --- actuators ---
        actuators = [
            ActuatorDef(
                name=a["name"],
                type=a.get("type", "general"),
                joint=a["joint"],
                gaintype=a.get("gaintype", "fixed"),
                gainprm=a.get("gainprm", [1, 0, 0]),
                biastype=a.get("biastype", "none"),
                biasprm=a.get("biasprm", [0, 0, 0]),
                dyntype=a.get("dyntype", "none"),
                dynprm=a.get("dynprm", [1, 0, 0]),
                ctrlrange=a.get("ctrlrange"),
                ctrllimited=a.get("ctrllimited", False),
                gear=a.get("gear", [1.0]),
            )
            for a in raw.get("actuators", [])
        ]

        # --- prosthetic: body removals ---
        body_removals: List[str] = raw.get("body_removals", [])

        # --- prosthetic: mesh replacements (default or per-MSK) ---
        def _parse_mesh_rep(m):
            return MeshReplacement(geom=m["geom"], mesh=m["mesh"])

        mesh_replacements, mesh_replacements_by_msk = _parse_per_msk_list(raw.get("mesh_replacements", []), _parse_mesh_rep)

        # --- prosthetic: actuator removals (default or per-MSK) ---
        actuator_removals, actuator_removals_by_msk = _parse_per_msk_list(raw.get("actuator_removals", []), lambda s: s)

        # --- prosthetic: tendon removals (default or per-MSK) ---
        tendon_removals, tendon_removals_by_msk = _parse_per_msk_list(raw.get("tendon_removals", []), lambda s: s)

        # --- prosthetic: geom removals (default or per-MSK) ---
        geom_removals, geom_removals_by_msk = _parse_per_msk_list(raw.get("geom_removals", []), lambda s: s)

        # --- prosthetic: tendon modifications (WrapEdit schema, default/per-MSK) ---
        def _parse_tendon_mod(t):
            return TendonModification(
                name=t["name"],
                wraps=[_parse_wrap_edit(w) for w in t.get("wraps", [])],
            )

        tendon_modifications, tendon_modifications_by_msk = _parse_per_msk_list(
            raw.get("tendon_modifications", []), _parse_tendon_mod
        )

        # --- body inertial overrides (default or per-MSK) ---
        body_overrides, body_overrides_by_msk = _parse_per_msk_list(raw.get("body_overrides", []), _parse_body_override)

        # --- sensors: additions + removals, each default or per-MSK ---
        sensors, sensors_by_msk = _parse_per_msk_list(raw.get("sensors", []), _parse_sensor)
        sensor_removals, sensor_removals_by_msk = _parse_per_msk_list(raw.get("sensor_removals", []), lambda s: s)

        # --- contact: excludes + explicit pairs, each default or per-MSK ---
        raw_contact = raw.get("contact", {}) or {}
        if not isinstance(raw_contact, dict):
            raise ValueError(
                f"'contact' must be a mapping with 'excludes' and/or 'pairs' keys; got {type(raw_contact).__name__}"
            )
        unknown = set(raw_contact) - {"excludes", "pairs"}
        if unknown:
            raise ValueError(f"unknown key(s) in 'contact': {sorted(unknown)}; expected 'excludes' and/or 'pairs'")
        contact_excludes, contact_excludes_by_msk = _parse_per_msk_list(raw_contact.get("excludes", []), _parse_exclude)
        contact_pairs, contact_pairs_by_msk = _parse_per_msk_list(raw_contact.get("pairs", []), _parse_pair)

        # --- keyframe overrides (model-agnostic, default or per-MSK) ---
        keyframe_overrides, keyframe_overrides_by_msk = _parse_keyframe_overrides(raw.get("keyframe_overrides", {}))

        # --- keyframes (legacy full-array mode, backward compat) ---
        keyframes: Dict[str, KeyframeDef] = {}
        for kf_name, kf_data in raw.get("keyframes", {}).items():
            if isinstance(kf_data, dict):
                keyframes[kf_name] = KeyframeDef(
                    time=kf_data.get("time", 0.0),
                    qpos=kf_data.get("qpos"),
                    qvel=kf_data.get("qvel"),
                )
            else:
                keyframes[kf_name] = KeyframeDef()

        return cls(
            name=name,
            model_xml=model_xml,
            attachments=attachments,
            compatible_msk=device_section.get("compatible_msk"),
            equalities=equalities,
            _equalities_by_msk=equalities_by_msk,
            joint_overrides=joint_overrides,
            actuators=actuators,
            keyframe_overrides=keyframe_overrides,
            keyframes=keyframes,
            body_removals=body_removals,
            mesh_replacements=mesh_replacements,
            actuator_removals=actuator_removals,
            tendon_removals=tendon_removals,
            _tendon_removals_by_msk=tendon_removals_by_msk,
            _attachments_by_msk=attachments_by_msk,
            geom_removals=geom_removals,
            _geom_removals_by_msk=geom_removals_by_msk,
            tendon_modifications=tendon_modifications,
            _tendon_modifications_by_msk=tendon_modifications_by_msk,
            _actuator_removals_by_msk=actuator_removals_by_msk,
            _keyframe_overrides_by_msk=keyframe_overrides_by_msk,
            _mesh_replacements_by_msk=mesh_replacements_by_msk,
            body_overrides=body_overrides,
            _body_overrides_by_msk=body_overrides_by_msk,
            contact_excludes=contact_excludes,
            _contact_excludes_by_msk=contact_excludes_by_msk,
            contact_pairs=contact_pairs,
            _contact_pairs_by_msk=contact_pairs_by_msk,
            sensors=sensors,
            _sensors_by_msk=sensors_by_msk,
            sensor_removals=sensor_removals,
            _sensor_removals_by_msk=sensor_removals_by_msk,
            _joint_overrides_by_msk=joint_overrides_by_msk,
            _config_dir=config_dir,
        )
