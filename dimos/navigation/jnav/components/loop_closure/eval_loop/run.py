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

"""Download, prep, benchmark and render every dataset (or the named ones), in priority order.

usage: run.py [--tables] [<dataset> ...]      datasets: see recordings.py
       --tables: only print the tables from existing summaries
"""

import json
from pathlib import Path
import subprocess
import sys

from paths import OUT, PGO_EVAL
from recordings import BY_NAME, DATASETS

HERE = Path(__file__).resolve().parent


def step(script: str, *args: str) -> None:
    subprocess.run([sys.executable, str(HERE / script), *args], check=True)


# Ivan's PGO only searches keyframes more than loop_time_thresh (20 s) apart (go2/loop_closure/pgo.py)
IVAN_MIN_LOOP_GAP_S = 20.0


def gsc_loops_over(name: str, key: str, seconds: float) -> int:
    """A gsc run's kept loop closures whose two keyframes are more than `seconds` apart."""
    output = OUT / (f"{name}.gsc.json" if key == "gsc" else f"{name}.{key}.json")
    result = json.loads(output.read_text())
    times = [row[0] for row in result["graph"]]
    return sum(
        1
        for edge in result["committed_loop_edges"]
        if edge["gnc_weight"] > 0
        and abs(times[edge["source_id"]] - times[edge["target_id"]]) > seconds
    )


def cell(row: dict, method: str) -> str:
    stats = row[method]
    voxels = f"{(stats['voxels'] / row['voxels_raw'] - 1) * 100:+.1f}%"
    crashed = " (CRASH unguarded)" if stats.get("crashed_unguarded") else ""
    loops = str(stats["loops"])
    if method.startswith("gsc"):
        loops += f" ({gsc_loops_over(row['dataset'], method, IVAN_MIN_LOOP_GAP_S)} >20 s)"
    return f"{loops} / {voxels}{crashed}"


# Ivan's TOTAL_SPREAD sums his own evaluation set (go2/loop_closure/eval.py DEFAULT_DATASETS)
IVAN_EVAL_SET = [f"hk_village{i}" for i in range(1, 7)]


# summary keys shown in the tables; gsc_ivancfg comes from `variant.py gsc_ivancfg "$(cat ivancfg.json)"`
COLUMNS = [
    ("ivan", "Ivan PGO()"),
    ("ivan_live", "Ivan LIVE_PGO"),
    ("gsc_ivancfg", "gsc, Ivan-equivalent config"),
    ("gsc", "gsc, defaults"),
]


def print_tables(datasets) -> None:
    rows = [json.loads((OUT / f"{d.name}.summary.json").read_text()) for d in datasets]
    columns = [(key, label) for key, label in COLUMNS if any(key in row for row in rows)]
    print(
        "\n| dataset | scans | "
        + " | ".join(f"{label} loops / voxels" for _, label in columns)
        + " |"
    )
    print("|---|---|" + "---|" * len(columns))
    for row in rows:
        cells = [cell(row, key) if key in row else "-" for key, _ in columns]
        print(f"| {row['dataset']} | {row['scans']} | " + " | ".join(cells) + " |")
    for row in rows:
        if row.get("tag_spread_raw_m") is None:
            print(f"tag metric not run on {row['dataset']}: {row.get('tag_note')}")
    tagged = [row for row in rows if row.get("tag_spread_raw_m") is not None]
    if not tagged:
        return
    print("\nIvan's AprilTag spread (his eval.py metric, m, lower = tighter):\n")
    print("| dataset | tracks | raw | " + " | ".join(label for _, label in columns) + " |")
    print("|---|---|---|" + "---|" * len(columns))

    def spread(row: dict, key: str) -> str:
        return f"{row[key]['tag_spread_m']:.2f}" if key in row else "-"

    for row in tagged:
        cells = [spread(row, key) for key, _ in columns]
        print(
            f"| {row['dataset']} | {row['tag_tracks']} | {row['tag_spread_raw_m']:.2f} | "
            + " | ".join(cells)
            + " |"
        )
    his = [row for row in tagged if row["dataset"] in IVAN_EVAL_SET]
    if len(his) == len(IVAN_EVAL_SET):
        totals = [
            f"{sum(row[key]['tag_spread_m'] for row in his):.2f}"
            if all(key in row for row in his)
            else "-"
            for key, _ in columns
        ]
        raw_total = sum(row["tag_spread_raw_m"] for row in his)
        print(f"| TOTAL hk_village1-6 | | {raw_total:.2f} | " + " | ".join(totals) + " |")


def main() -> None:
    args = [arg for arg in sys.argv[1:] if arg != "--tables"]
    unknown = [name for name in args if name not in BY_NAME]
    if unknown:
        raise SystemExit(f"unknown dataset(s) {unknown}; choose from {list(BY_NAME)}")
    datasets = [BY_NAME[name] for name in args] or DATASETS
    if "--tables" not in sys.argv:
        if not PGO_EVAL.exists():
            raise SystemExit(f"no pgo-eval at {PGO_EVAL}; build it first (README, Setup step 1)")
        for dataset in datasets:
            source = dataset.source()
            if dataset.odom and not (OUT / f"{dataset.name}.src.db").exists():
                step(
                    "prep.py",
                    dataset.name,
                    str(source),
                    dataset.lidar,
                    dataset.odom,
                    *([dataset.odom_pose_of] if dataset.odom_pose_of else []),
                )
            step("bench.py", dataset.name if dataset.odom else f"{dataset.name}={source}")
            step("render.py", dataset.name)
    print_tables(datasets)
    print(f"\nmaps: {OUT}/<dataset>.topdown.png")


if __name__ == "__main__":
    main()
