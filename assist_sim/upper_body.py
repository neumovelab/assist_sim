"""Composed upper-body environments (wheelchair, ...).

Unlike the lower-limb devices (modular MSK x device compositions resolved through
``registry`` / ``load_combined``), the upper-body environments are single composed
models: a myo_sim human model + device hardware, attached via ``MjSpec.attach``.

Mesh sourcing follows the 1.0 rule -- anatomical/MSK meshes come from the myo_sim
import (``myo_sim.build_spec``); assist_sim houses only the device-specific hardware
meshes under ``models/<Name>/mesh``. These envs are intentionally not made as modular
as the lower-limb devices; a single composed model + its utilities is the contract.
"""

from __future__ import annotations

from importlib.resources import files as _files

import mujoco

import myo_sim

_WHEELCHAIR_XML = str(_files("assist_sim").joinpath("models", "Wheelchair", "wheelchair.xml"))

# Chair placement relative to the human worldbody (identity orientation keeps the
# chair upright). Tuned so the seat pan sits just under the sacrum; see
# models/Wheelchair/TODO.md for the remaining pose/grip tuning.
_CHAIR_SEAT_OFFSET = (0.22, 0.36, 0.55)


def build_wheelchair_spec(arm_model: str = "myoarms") -> "mujoco.MjSpec":
    """Compose a seated wheelchair env: myo_sim ``arm_model`` + the chair hardware.

    The human is the **base** spec on purpose: attaching the human *as a child*
    drops 16 cross-body muscles (TRIlong/PECM/LAT/CORB/... -- the shoulder+elbow
    muscles that power propulsion) because ``MjSpec.attach`` mishandles their
    spanning tendons. Attaching the rigid chair *into* the human preserves all
    126 actuators. The chair's own freejoint is removed so it is rigidly seated;
    a freejoint on the human root lets the whole system roll.

    Returns the uncompiled ``MjSpec`` so callers can edit before compiling.
    """
    human = myo_sim.build_spec(arm_model)

    # Drop myo_sim scene decor (ground plane, backdrop meshes, light). The human
    # geoms live inside the 'Full Body' body, not at worldbody level.
    for geom in list(human.worldbody.geoms):
        human.delete(geom)
    for light in list(human.worldbody.lights):
        human.delete(light)
    human.body("Full Body").add_freejoint()

    chair = mujoco.MjSpec.from_file(_WHEELCHAIR_XML)
    for joint in list(chair.joints):  # rigid seat: drop the chair's own freejoint
        if joint.type == mujoco.mjtJoint.mjJNT_FREE:
            chair.delete(joint)

    frame = human.worldbody.add_frame()
    frame.pos = _CHAIR_SEAT_OFFSET
    human.attach(chair, prefix="wc_", frame=frame)
    return human


def build_wheelchair(arm_model: str = "myoarms"):
    """Compile the wheelchair env. Returns ``(MjModel, MjData)``."""
    model = build_wheelchair_spec(arm_model).compile()
    return model, mujoco.MjData(model)
