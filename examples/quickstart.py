"""Quickstart: combine an MSK + device and inspect the result in a viewer.

Resolves an MSK key and a device key through ``assist_sim.registry``, compiles
the combined model in memory, and opens a paused MuJoCo viewer window so you
can rotate, pan, and visually inspect the model.  Nothing is written to disk
(use ``load_combined_model(..., export_xml=...)`` if you want the combined XML).

Usage:

    python examples/quickstart.py                                  # defaults
    python examples/quickstart.py myoLeg22_2D DephyExoBoot_L1      # explicit pair
    python examples/quickstart.py --list                           # show all keys

Defaults to ``myoLeg22_2D`` + ``DephyExoBoot_L1``.

Requires the ``myo_sim`` package to be installed (it ships the MSK model files).
If it's missing, this script prints an install hint and exits.
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path

# Allow running from a fresh clone without `pip install -e .`: put the repo
# root on sys.path so `import assist_sim` works.
_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

DEFAULT_MSK = "myoLeg22_2D"
DEFAULT_DEVICE = "DephyExoBoot_L1"


def _abort_no_myo_sim() -> None:
    print(
        "ERROR: the `myo_sim` package is not installed in this environment.\n"
        "       assist_sim resolves MSK model files (myoLeg22_2D, myoLeg26_3D, "
        "myoLeg80) through that package.\n"
        "\n"
        "Once it is published to PyPI:\n"
        "    pip install myo_sim\n"
        "\n"
        "In the meantime, install from a git tag:\n"
        "    pip install git+https://github.com/MyoHub/myo_sim.git@<tag>\n"
        "\n"
        "Without myo_sim, no combined model can be compiled.",
        file=sys.stderr,
    )
    sys.exit(1)


def _list_combinations() -> None:
    from assist_sim.registry import (
        _COMPATIBLE_MSK_KEYS,
        DEVICE_CONFIGS,
        get_available_combinations,
    )

    print("Compatible MSKs (resolved through myo_sim):")
    for key in sorted(_COMPATIBLE_MSK_KEYS):
        pkg, fname = _COMPATIBLE_MSK_KEYS[key]
        print(f"  {key:14s}  <- {pkg}/{fname}")

    print("\nDevice configs (autodiscovered from models/):")
    for key in sorted(DEVICE_CONFIGS):
        print(f"  {key}")

    if importlib.util.find_spec("myo_sim"):
        print("\nResolvable combinations (msk x device):")
        for msk, devs in get_available_combinations().items():
            print(f"  {msk}:")
            for d in devs:
                print(f"      {d}")
    else:
        print(
            "\n(myo_sim not installed -- no combinations resolvable yet. "
            "See `python examples/quickstart.py --help` for install hints.)"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "msk",
        nargs="?",
        default=DEFAULT_MSK,
        help=f"MSK key (default: {DEFAULT_MSK})",
    )
    parser.add_argument(
        "device",
        nargs="?",
        default=DEFAULT_DEVICE,
        help=f"Device key (default: {DEFAULT_DEVICE})",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available MSK + device keys and exit.",
    )
    args = parser.parse_args()

    if args.list:
        _list_combinations()
        return

    if importlib.util.find_spec("myo_sim") is None:
        _abort_no_myo_sim()

    import mujoco as mj
    import mujoco.viewer

    from assist_sim import load_combined_model
    from assist_sim.registry import resolve

    print(f"Resolving {args.msk} + {args.device} ...")
    msk_path, device_path = resolve(args.msk, args.device)
    print(f"  MSK:    {msk_path}")
    print(f"  Device: {device_path}")

    print("Compiling combined model ...")
    model, data = load_combined_model(
        human_xml=str(msk_path),
        device_config=str(device_path),
        msk_key=args.msk,
    )
    print(
        f"  nq={model.nq}  nu={model.nu}  nbody={model.nbody}  "
        f"nmesh={model.nmesh}  nkey={model.nkey}"
    )

    # Load the first keyframe if any, so the model isn't sitting at qpos0.
    if model.nkey > 0:
        first_kf = mj.mj_id2name(model, mj.mjtObj.mjOBJ_KEY, 0)
        mj.mj_resetDataKeyframe(model, data, 0)
        mj.mj_forward(model, data)
        print(f"  Loaded keyframe: {first_kf!r}")

    # Cleaner default inspection scene: disable shadow map (effectively
    # disables shadow rendering).  Reflections are off by default because
    # no material in the model carries reflectance > 0 after the terrain
    # strip.  Both can still be toggled at runtime from the viewer's
    # Rendering panel.
    model.vis.quality.shadowsize = 0

    print(
        "\nOpening paused viewer.  Drag to rotate, scroll to zoom, "
        "ctrl-drag to pan.\n"
        "Press Enter to exit.\n"
    )

    # launch_passive() returns a Handle in a separate window; the simulation
    # does NOT step (paused).  Keep the script alive until the user signals exit.
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Initial free-camera pose at torso height, ~2.5m away.  Per-MSK
        # azimuth so the model faces toward the camera given each MSK's
        # world orientation (22/26 face +X, 80 faces -Y).
        viewer.cam.lookat[:] = [0.0, 0.0, 0.81]
        viewer.cam.distance = 2.5
        if args.msk == "myoLeg80":
            # <camera pos="-0.995 -2.282 1.032"
            #  xyaxes="0.917 -0.400 0 0.036 0.081 0.996"/>
            viewer.cam.azimuth = 66.4
            viewer.cam.elevation = 5.1
        else:
            # 22/26: <camera pos="2.276 -1.060 0.917"
            #  xyaxes="0.398 0.917 0 -0.039 0.017 0.999"/>
            viewer.cam.azimuth = 157.0
            viewer.cam.elevation = 2.4
        viewer.sync()
        try:
            input()
        except (KeyboardInterrupt, EOFError):
            pass
        viewer.close()


if __name__ == "__main__":
    main()
