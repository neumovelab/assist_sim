"""Device configuration dataclasses and YAML loader for the mjSpec combination pipeline."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

_WRAP_OPS = ("drop_site", "reposition_site", "replace_site")


def _parse_per_msk_list(raw_value, parse_item):
    """Parse a section that may be a plain list (default) or a per-MSK dict.

    Returns ``(default_list, by_msk)`` where ``by_msk`` always contains a
    ``"default"`` entry.  A dict value is treated as the per-MSK form::

        section:
          default: [...]
          myoLeg80: [...]
    """
    if isinstance(raw_value, dict):
        by_msk = {
            key: [parse_item(x) for x in (items or [])]
            for key, items in raw_value.items()
        }
        default = by_msk.get("default", [])
        return default, by_msk
    default = [parse_item(x) for x in (raw_value or [])]
    return default, {"default": default}


def _kf_overrides_from_map(mapping: dict) -> "Dict[str, KeyframeOverride]":
    return {
        kf: KeyframeOverride(joint_values=jv)
        for kf, jv in mapping.items()
        if isinstance(jv, dict)
    }


def _is_per_msk_keyframe_overrides(raw: dict) -> bool:
    """A per-MSK keyframe_overrides block nests one level deeper than default."""
    for value in raw.values():
        if isinstance(value, dict) and any(
            isinstance(inner, dict) for inner in value.values()
        ):
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
        raise ValueError(
            f"each wrap edit must have exactly one of {_WRAP_OPS}; got {raw}"
        )
    op = ops_present[0]
    site = raw[op]
    new_body = raw.get("new_body")
    pos = raw.get("pos")
    if op == "replace_site" and not new_body:
        raise ValueError(f"replace_site '{site}' requires 'new_body'")
    if op in ("reposition_site", "replace_site") and pos is None:
        raise ValueError(f"{op} '{site}' requires 'pos'")
    return WrapEdit(op=op, site=site, new_body=new_body, pos=pos)


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

    # Per-MSK override maps (each guaranteed a "default" entry). Populated by
    # from_yaml; resolve_* methods select the matching MSK key or fall back.
    _tendon_modifications_by_msk: Dict[str, List["TendonModification"]] = field(
        default_factory=dict, repr=False
    )
    _actuator_removals_by_msk: Dict[str, List[str]] = field(
        default_factory=dict, repr=False
    )
    _keyframe_overrides_by_msk: Dict[str, Dict[str, "KeyframeOverride"]] = field(
        default_factory=dict, repr=False
    )
    _mesh_replacements_by_msk: Dict[str, List["MeshReplacement"]] = field(
        default_factory=dict, repr=False
    )
    _tendon_removals_by_msk: Dict[str, List[str]] = field(
        default_factory=dict, repr=False
    )
    _attachments_by_msk: Dict[str, List["Attachment"]] = field(
        default_factory=dict, repr=False
    )
    _geom_removals_by_msk: Dict[str, List[str]] = field(
        default_factory=dict, repr=False
    )

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
        return self._resolve(
            self._actuator_removals_by_msk, msk_key, self.actuator_removals
        )

    def resolve_tendon_modifications(
        self, msk_key: Optional[str] = None
    ) -> List["TendonModification"]:
        """Tendon modifications for the given MSK (per-MSK override or default)."""
        return self._resolve(
            self._tendon_modifications_by_msk, msk_key, self.tendon_modifications
        )

    def resolve_keyframe_overrides(
        self, msk_key: Optional[str] = None
    ) -> Dict[str, "KeyframeOverride"]:
        """Keyframe overrides for the given MSK (per-MSK override or default)."""
        return self._resolve(
            self._keyframe_overrides_by_msk, msk_key, self.keyframe_overrides
        )

    def resolve_mesh_replacements(
        self, msk_key: Optional[str] = None
    ) -> List["MeshReplacement"]:
        """Mesh replacements for the given MSK (per-MSK override or default)."""
        return self._resolve(
            self._mesh_replacements_by_msk, msk_key, self.mesh_replacements
        )

    def resolve_tendon_removals(self, msk_key: Optional[str] = None) -> List[str]:
        """Tendon removals for the given MSK (per-MSK override or default)."""
        return self._resolve(
            self._tendon_removals_by_msk, msk_key, self.tendon_removals
        )

    def resolve_geom_removals(self, msk_key: Optional[str] = None) -> List[str]:
        """Geom removals for the given MSK (per-MSK override or default).

        Used for surgical geom removal (e.g. dropping the fibula geom that
        survives the tibia mesh-replacement on transtibial amputation).
        """
        return self._resolve(
            self._geom_removals_by_msk, msk_key, self.geom_removals
        )

    def resolve_attachments(
        self, msk_key: Optional[str] = None
    ) -> List["Attachment"]:
        """Attachments for the given MSK (per-MSK override or default).

        Lets a device declare a different attachment frame (``pos`` / ``quat``)
        per MSK, e.g. when the parent body's local frame differs across MSKs
        (myoLeg80's ``torso`` sits under a yaw-rotated ``root``).
        """
        return self._resolve(
            self._attachments_by_msk, msk_key, self.attachments
        )

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
            raise FileNotFoundError(
                f"Device model XML not found: {model_xml_abs} "
                f"(referenced from {yaml_path})"
            )

        # --- attachments ---
        # Accept either a flat list (legacy form) or a per-MSK dict where
        # ``default:`` is the fallback and any other key is an MSK-specific
        # override (used for e.g. myoLeg80 body-frame differences).
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
                raise ValueError(
                    "attachments dict form must include a 'default' entry"
                )
            for msk_key, items in raw_attachments.items():
                attachments_by_msk[msk_key] = _parse_attachment_list(items)
            attachments = attachments_by_msk["default"]
        else:
            attachments = _parse_attachment_list(raw_attachments)

        # --- joint overrides ---
        joint_overrides = [
            JointOverride(
                name=j["name"],
                range=j.get("range"),
                damping=j.get("damping"),
                axis=j.get("axis"),
                pos=j.get("pos"),
            )
            for j in raw.get("joint_overrides", [])
        ]

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

        mesh_replacements, mesh_replacements_by_msk = _parse_per_msk_list(
            raw.get("mesh_replacements", []), _parse_mesh_rep
        )

        # --- prosthetic: actuator removals (default or per-MSK) ---
        actuator_removals, actuator_removals_by_msk = _parse_per_msk_list(
            raw.get("actuator_removals", []), lambda s: s
        )

        # --- prosthetic: tendon removals (default or per-MSK) ---
        tendon_removals, tendon_removals_by_msk = _parse_per_msk_list(
            raw.get("tendon_removals", []), lambda s: s
        )

        # --- prosthetic: geom removals (default or per-MSK) ---
        geom_removals, geom_removals_by_msk = _parse_per_msk_list(
            raw.get("geom_removals", []), lambda s: s
        )

        # --- prosthetic: tendon modifications (WrapEdit schema, default/per-MSK) ---
        def _parse_tendon_mod(t):
            return TendonModification(
                name=t["name"],
                wraps=[_parse_wrap_edit(w) for w in t.get("wraps", [])],
            )

        tendon_modifications, tendon_modifications_by_msk = _parse_per_msk_list(
            raw.get("tendon_modifications", []), _parse_tendon_mod
        )

        # --- keyframe overrides (model-agnostic, default or per-MSK) ---
        keyframe_overrides, keyframe_overrides_by_msk = _parse_keyframe_overrides(
            raw.get("keyframe_overrides", {})
        )

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
            _config_dir=config_dir,
        )
