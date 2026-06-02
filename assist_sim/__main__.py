"""Command-line interface for assist_sim.

    python -m assist_sim list                      # show discoverable combinations
    python -m assist_sim validate MSK DEVICE       # check a pair resolves
    python -m assist_sim combine MSK DEVICE [-o OUT.xml] [--cache-dir DIR]

Also available as the ``assist-sim`` script after install.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from . import (
    __version__,
    get_available_combinations,
    load_combined,
    resolve_model_path,
    validate_combination,
)


def _cmd_list(_: argparse.Namespace) -> int:
    from .registry import DEVICE_CONFIGS, MSK_MODELS, _COMPATIBLE_MSK_KEYS

    combos = get_available_combinations()
    if combos:
        for msk_key, devices in combos.items():
            print(f"{msk_key}:")
            for device in devices:
                print(f"    - {device}")
        return 0

    # No MSKs resolved -- either myo_sim isn't installed or doesn't have any
    # of the listed MSK files yet.  Report what we DID find so the user can
    # tell which half is missing.
    print("No MSK x device combinations resolvable.")
    print()
    print(f"  Compatible MSK keys (curated): {sorted(_COMPATIBLE_MSK_KEYS)}")
    print(f"  MSK files resolvable now:      {sorted(MSK_MODELS) or '(none -- is `myo_sim` installed?)'}")
    print(f"  Device configs discovered:     {sorted(DEVICE_CONFIGS)}")
    print()
    print("Install myo_sim (pip install myo_sim, or a git+http URL until the")
    print("PyPI wheel ships) to enable MSK resolution.")
    return 0


def _cmd_validate(args: argparse.Namespace) -> int:
    ok = validate_combination(args.msk, args.device)
    if ok:
        human, config = resolve_model_path(args.msk, args.device)
        print(f"OK: {args.msk} x {args.device}")
        print(f"    human:  {human}")
        print(f"    config: {config}")
        return 0
    print(f"INVALID: {args.msk} x {args.device}", file=sys.stderr)
    return 1


def _cmd_combine(args: argparse.Namespace) -> int:
    model, _ = load_combined(
        args.msk,
        args.device,
        export_xml=args.output,
        cache_dir=args.cache_dir,
    )
    print(
        f"Combined {args.msk} x {args.device}: "
        f"nq={model.nq} nu={model.nu} nbody={model.nbody} nmesh={model.nmesh}"
    )
    if args.output:
        print(f"Exported to {Path(args.output).resolve()}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="assist-sim", description=__doc__)
    parser.add_argument("--version", action="version", version=f"assist-sim {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)

    p_list = sub.add_parser("list", help="list discoverable combinations")
    p_list.set_defaults(func=_cmd_list)

    p_val = sub.add_parser("validate", help="check that a combination resolves")
    p_val.add_argument("msk")
    p_val.add_argument("device")
    p_val.set_defaults(func=_cmd_validate)

    p_comb = sub.add_parser("combine", help="combine and optionally export")
    p_comb.add_argument("msk")
    p_comb.add_argument("device")
    p_comb.add_argument("-o", "--output", help="export the combined XML here")
    p_comb.add_argument("--cache-dir", help="enable caching in this directory")
    p_comb.set_defaults(func=_cmd_combine)

    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
