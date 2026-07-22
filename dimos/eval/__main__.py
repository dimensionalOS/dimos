# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Command-line : ``python -m dimos.eval <blueprint>``.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from dimos.eval.metrics import render_report
from dimos.eval.runner import run_eval


def _default_output(blueprint: str) -> Path:
    import socket

    host = socket.gethostname().split(".")[0]
    return Path.cwd() / f"eval_{blueprint}_{host}.json"


def _list_blueprints() -> None:
    from dimos.robot.all_blueprints import all_blueprints

    print("Available blueprints:")
    for name in sorted(all_blueprints):
        print(f"  {name}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m dimos.eval",
        description="Benchmark a DimOS blueprint's resource footprint on this machine.",
    )
    parser.add_argument("blueprint", nargs="?", help="Blueprint name (see --list).")
    parser.add_argument("--list", action="store_true", help="List available blueprints and exit.")

    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--replay", action="store_true", help="Run from recorded data (default).")
    mode.add_argument("--simulation", action="store_true", help="Run in simulation.")
    mode.add_argument("--real", action="store_true", help="Run against real hardware.")

    parser.add_argument("--simulator", choices=("mujoco", "dimsim"), default="mujoco",
                        help="Simulation backend for --simulation (default: mujoco).")

    parser.add_argument("--duration", type=float, default=15.0,
                        help="Steady-state sampling window in seconds (default: 15).")
    parser.add_argument("--warmup", type=float, default=3.0,
                        help="Seconds to run before sampling starts (default: 3).")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="Seconds between samples (default: 1).")
    parser.add_argument("--output", "-o", type=Path, default=None,
                        help="JSON artifact path (default: eval_<blueprint>_<host>.json).")
    parser.add_argument("--no-output", action="store_true", help="Do not write a JSON artifact.")
    parser.add_argument("--quiet", "-q", action="store_true", help="Skip the terminal report.")

    args = parser.parse_args(argv)

    if args.list:
        _list_blueprints()
        return 0
    if not args.blueprint:
        parser.error("a blueprint name is required (or use --list)")

    run_mode = "simulation" if args.simulation else "real" if args.real else "replay"

    result = run_eval(
        args.blueprint,
        run_mode=run_mode,
        simulator=args.simulator,
        duration=args.duration,
        warmup=args.warmup,
        interval=args.interval,
    )

    if not args.quiet:
        render_report(result)

    if not args.no_output:
        out = args.output or _default_output(args.blueprint)
        out.write_text(result.to_json())
        print(f"Wrote artifact: {out}")

    return 0 if result.status == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
