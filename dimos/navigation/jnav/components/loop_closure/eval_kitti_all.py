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

"""Run every PGO on KITTI db(s) and render an official-error comparison table.

Parallels eval_all.py. For each KITTI db (one sequence) it evaluates every PGO
module in its own subprocess (they share LCM) via eval_kitti.py, then renders
translational (%) / rotational (deg/m) error per module to kitti_comparison.md,
sorted best-first (lower error = better), alongside the raw-odometry baseline.

Usage:
    uv run python dimos/navigation/jnav/components/loop_closure/eval_kitti_all.py \
        --recordings-dir ~/datasets/kitti/sequences                   # all kitti_seq*/mem2.db
        [--db-path ~/datasets/kitti/sequences/kitti_seq07/mem2.db]    # or one
        [--only gsc_pgo,ivan_pgo]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Any

LOOP_CLOSURE_DIR = Path(__file__).resolve().parent
RESULTS_DIR = LOOP_CLOSURE_DIR / "eval_results"
TABLE_PATH = RESULTS_DIR / "kitti_comparison.md"
MODULES = ("gsc_pgo", "ivan_pgo", "ivan_pgo_transformer", "unrefined_pgo")


def run_one(db_path: Path, module_dir: str) -> bool:
    command = [
        sys.executable,
        "-u",
        str(LOOP_CLOSURE_DIR / "eval_kitti.py"),
        "--db-path",
        str(db_path),
        "--module-path",
        str(LOOP_CLOSURE_DIR / module_dir / "module.py"),
        "--module-name",
        "PGO",
    ]
    print(f"\n=== {db_path.parent.name} / {module_dir} ===", flush=True)
    return subprocess.run(command, check=False).returncode == 0


def render_table() -> Path:
    by_recording: dict[str, list[dict[str, Any]]] = {}
    for summary_path in sorted(RESULTS_DIR.glob("*/kitti_summary.json")):
        summary = json.loads(summary_path.read_text())
        recording = Path(summary["db"]).parent.name
        by_recording.setdefault(recording, []).append(
            {"module": summary["module"], **summary["scores"]}
        )

    lines = ["# KITTI official-error PGO comparison", ""]
    lines += [
        "Translational error (%) and rotational error (deg/m), the KITTI",
        "leaderboard metric (avg relative pose error over 100..800m sub-sequences).",
        "**Lower is better.** raw odometry = the scan-to-scan ICP input the PGO refines.",
        "",
    ]
    for recording in sorted(by_recording):
        rows = sorted(by_recording[recording], key=lambda r: r["translational_percent"])
        lines += [f"## {recording}", ""]
        lines.append("| module | transl % | rot deg/m | closures | keyframes |")
        lines.append("|---|---|---|---|---|")
        baseline = rows[0]
        lines.append(
            f"| _raw odometry_ | {baseline['baseline_translational_percent']:.2f} | "
            f"{baseline['baseline_rotational_deg_per_m']:.4f} | — | — |"
        )
        for row in rows:
            lines.append(
                f"| {row['module']} | {row['translational_percent']:.2f} | "
                f"{row['rotational_deg_per_m']:.4f} | {row['closures']} | {row['keyframes']} |"
            )
        lines.append("")
    RESULTS_DIR.mkdir(exist_ok=True)
    TABLE_PATH.write_text("\n".join(lines))
    return TABLE_PATH


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recordings-dir", type=Path, default=Path("~/datasets/kitti/sequences"))
    parser.add_argument(
        "--db-path", type=Path, default=None, help="a single kitti db (overrides dir)"
    )
    parser.add_argument("--only", help="comma-separated module dirs")
    args = parser.parse_args()

    if args.db_path:
        dbs = [args.db_path.expanduser()]
    else:
        dbs = sorted(args.recordings_dir.expanduser().glob("kitti_seq*/mem2.db"))
    if not dbs:
        raise SystemExit("no KITTI dbs found (run kitti_to_db.py first)")
    modules = [m.strip() for m in args.only.split(",")] if args.only else list(MODULES)

    for db_path in dbs:
        for module_dir in modules:
            run_one(db_path, module_dir)
        table = render_table()
        print(f"\ntable -> {table}")


if __name__ == "__main__":
    main()
