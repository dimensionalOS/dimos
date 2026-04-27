# Copyright 2025-2026 Dimensional Inc.
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

"""CLI: overlay the cmd/meas traces of multiple same-type runs.

Writes a single SVG to ``--out`` (default ``compare.svg`` in the current
working directory). All runs must have the same ``test_type``.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

from dimos.utils.characterization.scripts.analyze_run import (
    _channel_arrays,
    _channel_unit,
    _dominant_channel,
    _reconstruct_or_empty,
    load_run,
)

logger = logging.getLogger(__name__)

_PALETTE = [
    "#1f77b4",
    "#d62728",
    "#2ca02c",
    "#ff7f0e",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#17becf",
]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Overlay cmd/meas traces of multiple runs.")
    parser.add_argument("run_dirs", nargs="+", help="Two or more run directories.")
    parser.add_argument("--out", default="compare.svg", help="Output SVG path (default: compare.svg)")
    parser.add_argument(
        "--channel",
        choices=("vx", "vy", "wz", "auto"),
        default="auto",
        help=(
            "Body-frame channel to compare. 'auto' (default) picks the "
            "dominant commanded channel from the first run."
        ),
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    from dimos.memory2.vis.plot.elements import Series, Style
    from dimos.memory2.vis.plot.plot import Plot, TimeAxis

    runs = [load_run(Path(p)) for p in args.run_dirs]
    types = {r.test_type for r in runs}
    if len(types) != 1:
        print(
            f"ERROR: runs have different test_types: {types}. Cannot compare.",
            file=sys.stderr,
        )
        return 2

    channel = args.channel if args.channel != "auto" else _dominant_channel(runs[0])
    unit = _channel_unit(channel)

    plot = Plot(time_axis=TimeAxis.raw)
    for i, run in enumerate(runs):
        color = _PALETTE[i % len(_PALETTE)]
        cmd_arr, meas_arr = _channel_arrays(run, channel)
        plot.add(
            Series(
                ts=run.cmd_ts_rel.tolist(),
                values=cmd_arr.tolist(),
                label=f"{run.name}: cmd_{channel} [{unit}]",
                color=color,
                style=Style.dashed,
            )
        )
        if run.meas_ts_rel.size:
            plot.add(
                Series(
                    ts=run.meas_ts_rel.tolist(),
                    values=meas_arr.tolist(),
                    label=f"{run.name}: meas_{channel} [{unit}]",
                    color=color,
                )
            )

    out_path = Path(args.out).expanduser().resolve()
    out_path.write_text(plot.to_svg())
    print(f"compared {len(runs)} runs → {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
