"""Core model combination logic using the MuJoCo mjSpec API.

Two-phase flow (runs on ``mujoco==3.3.3``, which has no ``MjSpec.delete``):

1. **Preprocess** (:mod:`assist_sim.preprocess`): an ElementTree pass applies
   every removal/cascade op to the human XML and writes a temp file.
2. **MjSpec phase** (here): load the preprocessed human + device specs,
   attach device bodies, edit attributes in place, add actuators, and rebuild
   keyframes after compile.  No element is ever deleted from a spec.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional, Tuple

import mujoco as mj

from .config import ActuatorDef, DeviceConfig
from .errors import unknown_reference
from .preprocess import KeyframeData, preprocess_human_xml, prepare_device_xml

# Maps from string names used in YAML to mujoco enum values
_GAINTYPE_MAP = {
    "fixed": mj.mjtGain.mjGAIN_FIXED,
    "affine": mj.mjtGain.mjGAIN_AFFINE,
    "muscle": mj.mjtGain.mjGAIN_MUSCLE,
    "user": mj.mjtGain.mjGAIN_USER,
}

_BIASTYPE_MAP = {
    "none": mj.mjtBias.mjBIAS_NONE,
    "affine": mj.mjtBias.mjBIAS_AFFINE,
    "muscle": mj.mjtBias.mjBIAS_MUSCLE,
    "user": mj.mjtBias.mjBIAS_USER,
}

_DYNTYPE_MAP = {
    "none": mj.mjtDyn.mjDYN_NONE,
    "integrator": mj.mjtDyn.mjDYN_INTEGRATOR,
    "filter": mj.mjtDyn.mjDYN_FILTER,
    "filterexact": mj.mjtDyn.mjDYN_FILTEREXACT,
    "muscle": mj.mjtDyn.mjDYN_MUSCLE,
    "user": mj.mjtDyn.mjDYN_USER,
}


class ModelCombiner:
    """Combines a musculoskeletal model with a device model using mjSpec.

    Removals are handled by the preprocess layer; this class only performs
    additive/in-place operations so it runs on MuJoCo 3.3.3.
    """

    def combine(
        self,
        human_xml: str,
        device_config: DeviceConfig,
        export_xml: Optional[str] = None,
        msk_key: Optional[str] = None,
        keep_temp: bool = False,
    ) -> Tuple[mj.MjModel, mj.MjData]:
        """Combine a human musculoskeletal model with a device.

        Args:
            human_xml: Path to the baseline musculoskeletal model XML.
            device_config: Loaded DeviceConfig describing the device.
            export_xml: If provided, save the combined model XML to this path.
            msk_key: Optional MSK key for per-MSK config overrides.
            keep_temp: If True, leave the preprocess temp files on disk
                (debugging aid).

        Returns:
            Tuple of (MjModel, MjData) ready for simulation.
        """
        human_xml = str(Path(human_xml).resolve())
        device_xml = str(device_config.model_xml_path)
        prefix = device_config.name + "_"

        pp = preprocess_human_xml(human_xml, device_config, msk_key=msk_key)
        device_full = prepare_device_xml(device_xml, strip_meshes=False)
        device_stripped = prepare_device_xml(device_xml, strip_meshes=True)
        temps = [pp.path, device_full, device_stripped]

        try:
            human_spec = mj.MjSpec.from_file(pp.path)
            device_full_spec = mj.MjSpec.from_file(device_full)
            device_stripped_spec = mj.MjSpec.from_file(device_stripped)

            self._attach_bodies(
                human_spec,
                device_full_spec,
                device_stripped_spec,
                device_config,
                prefix,
                msk_key=msk_key,
            )
            self._replace_meshes(human_spec, device_config, prefix, msk_key)
            self._apply_joint_overrides(human_spec, device_config)
            self._add_actuators(human_spec, device_config, prefix)
            self._import_device_tendons_actuators(human_spec, device_xml, prefix)

            # First compile gives the final qpos/dof layout used to rebuild
            # keyframes by joint name (no MjSpec.delete needed).
            model = human_spec.compile()
            self._rebuild_keyframes(
                human_spec, model, pp.keyframes, device_config, prefix, msk_key
            )
            model = human_spec.compile()
            data = mj.MjData(model)

            if export_xml:
                from .utils import export_combined_xml

                mesh_dirs = [
                    (
                        Path(human_spec.modelfiledir),
                        getattr(human_spec, "meshdir", "") or "",
                    ),
                    (
                        Path(device_full_spec.modelfiledir),
                        getattr(device_full_spec, "meshdir", "") or "",
                    ),
                ]
                export_combined_xml(
                    human_spec,
                    export_xml,
                    mesh_dirs=mesh_dirs,
                    terrain_paths=pp.terrain_paths,
                )

            return model, data
        finally:
            if not keep_temp:
                for t in temps:
                    Path(t).unlink(missing_ok=True)

    # ------------------------------------------------------------------
    # Attachment
    # ------------------------------------------------------------------

    @staticmethod
    def _attach_bodies(
        human_spec: mj.MjSpec,
        device_full_spec: mj.MjSpec,
        device_stripped_spec: mj.MjSpec,
        config: DeviceConfig,
        prefix: str,
        msk_key: Optional[str] = None,
    ) -> None:
        """Attach each device body to its human parent body.

        The first attach uses the full device spec, which copies the entire
        device asset library (meshes, defaults) into the human spec with the
        namespace prefix.  Every subsequent attach uses a mesh-stripped device
        spec so duplicate meshes are not re-added -- the device geoms still
        resolve to the already-copied prefixed meshes.  This replaces the 3.5+
        "delete device meshes after first attach" trick.
        """
        human_spec.copy_during_attach = True

        attachments = config.resolve_attachments(msk_key)
        for i, att in enumerate(attachments):
            parent_body = human_spec.body(att.parent_body)
            if parent_body is None:
                raise unknown_reference(
                    att.parent_body,
                    [b.name for b in human_spec.bodies],
                    section="attachments.parent_body",
                    kind="body",
                )

            src_spec = device_full_spec if i == 0 else device_stripped_spec
            device_body = src_spec.body(att.device_body)
            if device_body is None:
                raise unknown_reference(
                    att.device_body,
                    [b.name for b in device_full_spec.bodies],
                    section="attachments.device_body",
                    kind="device body",
                )

            frame = ModelCombiner._make_frame(parent_body, att)
            frame.attach_body(device_body, prefix=prefix)

    @staticmethod
    def _make_frame(parent_body, att):
        """Create an attachment frame, with optional pose offset (phase 05)."""
        kwargs = {}
        if getattr(att, "pos", None) is not None:
            kwargs["pos"] = att.pos
        if getattr(att, "quat", None) is not None:
            kwargs["quat"] = att.quat
        return parent_body.add_frame(**kwargs)

    @staticmethod
    def _replace_meshes(
        human_spec: mj.MjSpec,
        config: DeviceConfig,
        prefix: str,
        msk_key: Optional[str] = None,
    ) -> None:
        """Swap a geom's mesh reference to a device-provided replacement mesh."""
        for mr in config.resolve_mesh_replacements(msk_key):
            geom = human_spec.geom(mr.geom)
            if geom is None:
                raise unknown_reference(
                    mr.geom,
                    [g.name for g in human_spec.geoms],
                    section="mesh_replacements",
                    kind="geom",
                )
            geom.meshname = prefix + mr.mesh

    @staticmethod
    def _apply_joint_overrides(
        human_spec: mj.MjSpec,
        config: DeviceConfig,
    ) -> None:
        """Override joint properties (range, damping) on the human model."""
        for override in config.joint_overrides:
            joint = human_spec.joint(override.name)
            if joint is None:
                raise unknown_reference(
                    override.name,
                    [j.name for j in human_spec.joints],
                    section="joint_overrides",
                    kind="joint",
                )
            if override.range is not None:
                joint.range = override.range
            if override.damping is not None:
                joint.damping = override.damping

    @staticmethod
    def _add_actuators(
        human_spec: mj.MjSpec,
        config: DeviceConfig,
        prefix: str = "",
    ) -> None:
        """Add device actuators to the combined spec (auto-prefix device joints)."""
        for act_def in config.actuators:
            kwargs = _build_actuator_kwargs(act_def, human_spec, prefix)
            human_spec.add_actuator(**kwargs)

    # ------------------------------------------------------------------
    # Device-side tendon + tendon-transmission actuator import
    # ------------------------------------------------------------------

    @staticmethod
    def _import_device_tendons_actuators(
        human_spec: mj.MjSpec,
        device_xml_path: str,
        prefix: str,
    ) -> None:
        """Copy ``<tendon>/<spatial>`` and tendon-driven ``<actuator>`` entries
        from the device XML into the human spec, with name + site prefixed.

        ``MjSpec.attach_body`` only copies the body subtree (and its referenced
        assets) -- top-level tendon and actuator sections do not migrate.  This
        routine reads them from the device XML directly and re-creates each
        spatial tendon (with all attributes) plus any actuator whose
        ``tendon=`` references it.  Joint-transmission actuators are authored
        in YAML and skipped here.
        """
        tree = ET.parse(device_xml_path)
        root = tree.getroot()

        tendon_root = root.find("tendon")
        if tendon_root is not None:
            for spatial in tendon_root.findall("spatial"):
                src_name = spatial.get("name", "")
                if not src_name:
                    continue
                new_name = prefix + src_name
                tendon = human_spec.add_tendon(name=new_name)
                _apply_tendon_attrs(tendon, spatial)
                for child in spatial:
                    if child.tag == "site":
                        tendon.wrap_site(prefix + child.get("site", ""))

        actuator_root = root.find("actuator")
        if actuator_root is not None:
            for act in actuator_root:
                tendon_target = act.get("tendon")
                if not tendon_target:
                    continue  # joint-transmission actuators belong in YAML
                kwargs = _xml_actuator_kwargs(act, prefix)
                human_spec.add_actuator(**kwargs)

    # ------------------------------------------------------------------
    # Keyframes (rebuilt after compile, no MjSpec.delete)
    # ------------------------------------------------------------------

    @staticmethod
    def _rebuild_keyframes(
        human_spec: mj.MjSpec,
        model: mj.MjModel,
        parsed_keyframes,
        config: DeviceConfig,
        prefix: str,
        msk_key: Optional[str],
    ) -> None:
        """Re-add keyframes to the spec using the final compiled layout.

        Surviving joints are restored to their authored values by name;
        device-added joints take their model default (``qpos0``); then
        ``keyframe_overrides`` are applied on top.  This preserves pose
        fidelity across body removals without depending on array length.

        Compiling an attached spec leaves a stray empty-named keyframe in the
        spec (the 3.5 path used ``spec.delete`` to drop it).  Since 3.3.3 has
        no delete, those pre-existing keys are *repurposed* to host the first
        rebuilt keyframes, which avoids emitting an extra empty key.
        """
        # A small emitter that reuses pre-existing (empty) keys before adding.
        existing_keys = list(human_spec.keys)
        cursor = {"i": 0}

        def emit(name: str, time: float, qpos, qvel) -> None:
            if cursor["i"] < len(existing_keys):
                key = existing_keys[cursor["i"]]
                key.name = name
                key.time = time
                key.qpos = qpos
                key.qvel = qvel
                cursor["i"] += 1
            else:
                human_spec.add_key(name=name, time=time, qpos=qpos, qvel=qvel)

        # Legacy full-array mode: explicit qpos/qvel arrays per keyframe.
        if config.keyframes and not config.keyframe_overrides:
            has_full = any(kf.qpos is not None for kf in config.keyframes.values())
            if has_full:
                for kf_name, kf_def in config.keyframes.items():
                    if kf_def.qpos is None:
                        continue
                    qvel = kf_def.qvel if kf_def.qvel is not None else [0.0] * model.nv
                    emit(kf_name, kf_def.time, kf_def.qpos, qvel)
                return

        if not parsed_keyframes:
            return

        overrides = config.resolve_keyframe_overrides(msk_key)

        for kf_name, kf_data in parsed_keyframes.items():
            qpos = list(model.qpos0)
            qvel = [0.0] * model.nv

            ModelCombiner._restore_joint_slices(
                model, kf_data.qpos_by_joint, qpos, model.jnt_qposadr
            )
            ModelCombiner._restore_joint_slices(
                model, kf_data.qvel_by_joint, qvel, model.jnt_dofadr
            )

            override = overrides.get(kf_name)
            if override is not None:
                ModelCombiner._apply_overrides(model, override, prefix, qpos)

            emit(kf_name, kf_data.time, qpos, qvel)

    @staticmethod
    def _restore_joint_slices(model, by_joint, target, adr_array) -> None:
        """Write each named joint's stored slice into the target array."""
        limit = len(target)
        for joint_name, values in by_joint.items():
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint_name)
            if jid < 0:
                continue  # joint was removed; nothing to restore
            start = int(adr_array[jid])
            for k, v in enumerate(values):
                if start + k < limit:
                    target[start + k] = v

    @staticmethod
    def _apply_overrides(model, override, prefix, qpos) -> None:
        """Apply per-joint keyframe overrides onto a qpos vector."""
        for joint_name, value in override.joint_values.items():
            jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint_name)
            if jid < 0 and prefix:
                jid = mj.mj_name2id(
                    model, mj.mjtObj.mjOBJ_JOINT, prefix + joint_name
                )
            if jid < 0:
                # MSKs differ (e.g. myoLeg80 has freejoint root, not pelvis_ty).
                continue
            qpos[int(model.jnt_qposadr[jid])] = value


def _pad(values: list, size: int) -> list:
    """Pad a list with zeros to the required MuJoCo array size."""
    if len(values) >= size:
        return values[:size]
    return values + [0.0] * (size - len(values))


def _floats(text: Optional[str]) -> Optional[List[float]]:
    """Parse a whitespace-separated float list ('0 0 0.5') or None."""
    if text is None:
        return None
    return [float(x) for x in text.split()]


def _bool(text: Optional[str]) -> Optional[bool]:
    if text is None:
        return None
    return text.lower() in ("true", "1")


def _apply_tendon_attrs(tendon, spatial_elem: ET.Element) -> None:
    """Copy supported attributes from a device-XML <spatial> onto a MjSpec tendon.

    Only attributes that round-trip cleanly via MjSpec properties are set.
    Unknown attributes are ignored (with no warning) since device authors may
    use class-driven defaults.
    """
    rgba = _floats(spatial_elem.get("rgba"))
    if rgba is not None and len(rgba) == 4:
        tendon.rgba = rgba

    width = spatial_elem.get("width")
    if width is not None:
        tendon.width = float(width)

    limited = _bool(spatial_elem.get("limited"))
    if limited is not None:
        # MjSpec stores limited as int (0/1) on 3.3.3; assign defensively.
        try:
            tendon.limited = mj.mjtLimited.mjLIMITED_TRUE if limited else mj.mjtLimited.mjLIMITED_FALSE
        except AttributeError:
            tendon.limited = 1 if limited else 0

    rng = _floats(spatial_elem.get("range"))
    if rng is not None and len(rng) == 2:
        tendon.range = rng

    sl = _floats(spatial_elem.get("springlength"))
    if sl is not None:
        # springlength may be 1 or 2 floats in XML; MjSpec expects 2.
        if len(sl) == 1:
            sl = [sl[0], sl[0]]
        tendon.springlength = sl

    for attr in ("stiffness", "damping", "frictionloss", "margin"):
        v = spatial_elem.get(attr)
        if v is not None:
            setattr(tendon, attr, float(v))

    material = spatial_elem.get("material")
    if material:
        tendon.material = material


def _xml_actuator_kwargs(act_elem: ET.Element, prefix: str) -> dict:
    """Convert a device-XML <actuator>/<general|motor|...> with tendon target
    into ``MjSpec.add_actuator`` kwargs (prefixed name + tendon)."""
    kwargs: dict = {"name": prefix + act_elem.get("name", "")}
    kwargs["trntype"] = mj.mjtTrn.mjTRN_TENDON
    kwargs["target"] = prefix + act_elem.get("tendon", "")

    gaintype = act_elem.get("gaintype")
    if gaintype:
        enum = _GAINTYPE_MAP.get(gaintype.lower())
        if enum is not None:
            kwargs["gaintype"] = enum
    gainprm = _floats(act_elem.get("gainprm"))
    if gainprm is not None:
        kwargs["gainprm"] = _pad(gainprm, 10)

    biastype = act_elem.get("biastype")
    if biastype:
        enum = _BIASTYPE_MAP.get(biastype.lower())
        if enum is not None:
            kwargs["biastype"] = enum
    biasprm = _floats(act_elem.get("biasprm"))
    if biasprm is not None:
        kwargs["biasprm"] = _pad(biasprm, 10)

    dyntype = act_elem.get("dyntype")
    if dyntype:
        enum = _DYNTYPE_MAP.get(dyntype.lower())
        if enum is not None:
            kwargs["dyntype"] = enum
    dynprm = _floats(act_elem.get("dynprm"))
    if dynprm is not None:
        kwargs["dynprm"] = _pad(dynprm, 10)

    rng = _floats(act_elem.get("ctrlrange"))
    if rng is not None and len(rng) == 2:
        kwargs["ctrlrange"] = rng
    if _bool(act_elem.get("ctrllimited")):
        kwargs["ctrllimited"] = 1

    gear = _floats(act_elem.get("gear"))
    if gear is not None:
        kwargs["gear"] = _pad(gear, 6)

    return kwargs


def _build_actuator_kwargs(
    act_def: ActuatorDef,
    spec: Optional[mj.MjSpec] = None,
    prefix: str = "",
) -> dict:
    """Convert an ActuatorDef into kwargs for MjSpec.add_actuator().

    If spec and prefix are given, resolves the joint target: use bare name
    if it exists in the spec, otherwise try prefix + joint (for device joints).
    """
    kwargs: dict = {"name": act_def.name}

    target = act_def.joint
    if spec is not None and prefix:
        if spec.joint(target) is not None:
            pass
        else:
            prefixed = prefix + target
            if spec.joint(prefixed) is not None:
                target = prefixed

    kwargs["trntype"] = mj.mjtTrn.mjTRN_JOINT
    kwargs["target"] = target

    gaintype_enum = _GAINTYPE_MAP.get(act_def.gaintype.lower())
    if gaintype_enum is not None:
        kwargs["gaintype"] = gaintype_enum
    kwargs["gainprm"] = _pad(act_def.gainprm, 10)

    biastype_enum = _BIASTYPE_MAP.get(act_def.biastype.lower())
    if biastype_enum is not None:
        kwargs["biastype"] = biastype_enum
    kwargs["biasprm"] = _pad(act_def.biasprm, 10)

    dyntype_enum = _DYNTYPE_MAP.get(act_def.dyntype.lower())
    if dyntype_enum is not None:
        kwargs["dyntype"] = dyntype_enum
    kwargs["dynprm"] = _pad(act_def.dynprm, 10)

    if act_def.ctrlrange is not None:
        kwargs["ctrlrange"] = act_def.ctrlrange
    if act_def.ctrllimited:
        kwargs["ctrllimited"] = 1

    kwargs["gear"] = _pad(act_def.gear, 6)

    return kwargs
