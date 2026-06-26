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

"""Resumable PGO benchmark over (environment x implementation/config).

Fills one big table: every environment (go2 recordings, hk_village; kitti once
converted) scored against every PGO column (gsc_pgo in several configs, the
basic unrefined_pgo, ivan_pgo, ivan_pgo_transformer). Each cell is one eval.py
subprocess (sequential — they share the isolated LCM bus).

CHECKPOINTED: a cell is skipped when its summary.json already exists and its
fingerprint still matches the db (size+mtime) and EVAL_VERSION. Kill it any
time and rerun — only missing/stale cells recompute. `--force` recomputes all.

The universal score is **voxel agreement** (re-anchoring scans onto the
corrected trajectory should collapse double walls — ground-truth-free and
needs no camera), so tagless environments (kitti, hk_village) still get a
real number. April-tag agreement is reported additionally wherever a camera +
intrinsics sidecar exists.

Usage:
    uv run python dimos/navigation/jnav/components/loop_closure/benchmark_table.py
    ... [--go2-root ~/datasets/go2_recordings] [--with-hk-village] [--force]
    ... [--only-env NAME] [--only-col NAME] [--table-only]
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

LOOP_CLOSURE_DIR = Path(__file__).resolve().parent
EVAL_PY = LOOP_CLOSURE_DIR / "eval.py"
RESULTS_DIR = LOOP_CLOSURE_DIR / "eval_results"
TABLE_PATH = RESULTS_DIR / "benchmark_table.md"

DEFAULT_GO2_ROOT = Path("~/datasets/go2_recordings").expanduser()
# loop_closure/modules/jnav/navigation/dimos/<repo_root> -> parents[4] is repo root.
LFS_DATA_DIR = LOOP_CLOSURE_DIR.parents[4] / "data"  # repo_root/data (hk_village*.db)

# Shared loop-closure thresholds for the gsc_pgo configs (mirrors recordings_eval).
_CMU_BASE: dict[str, Any] = {
    "loop_search_radius": 3.0,
    "loop_time_thresh": 5.0,
    "min_loop_detect_duration": 2.0,
    "key_pose_delta_trans": 0.5,
}


@dataclass(frozen=True)
class Column:
    """One implementation+config = one table column."""

    name: str  # column label + results-suffix (disambiguates same-class modules)
    module_dir: str  # subdir under loop_closure/ holding module.py (class PGO)
    overrides: dict[str, Any] = field(default_factory=dict)


COLUMNS: list[Column] = [
    Column("cmu_stock", "gsc_pgo", {}),
    Column("cmu_scan_context", "gsc_pgo", {**_CMU_BASE, "use_scan_context": True}),
    Column("cmu_radius", "gsc_pgo", {**_CMU_BASE, "use_scan_context": False}),
    Column(
        "cmu_scan_context_far",
        "gsc_pgo",
        {
            **_CMU_BASE,
            "use_scan_context": True,
            "loop_candidate_max_distance_m": 0.0,
            "loop_score_thresh": 10000.0,
        },
    ),
    Column("unrefined", "unrefined_pgo", {}),
    Column("ivan", "ivan_pgo", {"publish_global_map": False}),
    Column("ivan_transformer", "ivan_pgo_transformer", {}),
]


@dataclass(frozen=True)
class Environment:
    """One dataset = one table row."""

    name: str  # results-dir recording key
    db_path: Path
    odom_stream: str
    lidar_stream: str
    camera_stream: str | None = None
    intrinsics_json: Path | None = None


def discover_go2(root: Path) -> list[Environment]:
    environments = []
    for db_path in sorted(root.glob("*/mem2.db")):
        recording = db_path.parent
        sidecar = recording / "camera_intrinsics.json"
        environments.append(
            Environment(
                name=recording.name,
                db_path=db_path,
                odom_stream="fastlio_odometry",
                lidar_stream="fastlio_lidar",
                camera_stream="color_image",
                intrinsics_json=sidecar if sidecar.exists() else None,
            )
        )
    return environments


def discover_hk_village(data_dir: Path) -> list[Environment]:
    # hk_village LFS dbs publish world-frame lidar + PoseStamped odom; no camera
    # intrinsics sidecar, so they score on voxel agreement alone.
    environments = []
    for db_path in sorted(data_dir.glob("hk_village*.db")):
        environments.append(
            Environment(
                name=db_path.stem,
                db_path=db_path,
                odom_stream="odom",
                lidar_stream="lidar",
                camera_stream=None,
                intrinsics_json=None,
            )
        )
    return environments


def cell_dir(environment: Environment, column: Column) -> Path:
    # Mirrors eval.py's out_dir formula: <recording>__<package>.PGO[.<suffix>].
    return RESULTS_DIR / f"{environment.name}__{column.module_dir}.PGO.{column.name}"


def cell_is_fresh(environment: Environment, column: Column) -> bool:
    summary_path = cell_dir(environment, column) / "summary.json"
    if not summary_path.exists():
        return False
    try:
        summary = json.loads(summary_path.read_text())
    except (json.JSONDecodeError, OSError):
        return False
    fingerprint = summary.get("fingerprint", {})
    stat = environment.db_path.stat()
    return (
        fingerprint.get("db_bytes") == stat.st_size
        and fingerprint.get("db_mtime") == int(stat.st_mtime)
        and fingerprint.get("version") is not None
    )


def _kill_zombies() -> None:
    """Clear leftover native processes / workers that can wedge the next cell."""
    subprocess.run(
        "lsof -ti tcp:7766 2>/dev/null | xargs kill -9 2>/dev/null;"
        ' pkill -9 -f "bin/pgo|scene_lidar" 2>/dev/null',
        shell=True,
        check=False,
    )


def run_cell(environment: Environment, column: Column) -> bool:
    command = [
        sys.executable,
        "-u",
        str(EVAL_PY),
        "--db-path",
        str(environment.db_path),
        "--odom-stream",
        environment.odom_stream,
        "--lidar-stream",
        environment.lidar_stream,
        "--module-path",
        str(LOOP_CLOSURE_DIR / column.module_dir / "module.py"),
        "--module-name",
        "PGO",
        "--recording-name",
        environment.name,
        "--results-suffix",
        column.name,
        "--with-rrd",
        "false",
        "--lockstep",
        "true",
    ]
    if environment.camera_stream is not None:
        command += ["--camera-stream", environment.camera_stream]
    if environment.intrinsics_json is not None:
        command += ["--camera-intrinsics-json-path", str(environment.intrinsics_json)]
    if column.overrides:
        command += ["--pgo-config-json", json.dumps(column.overrides)]
    print(f"\n=== {environment.name} x {column.name} ===", flush=True)
    result = subprocess.run(command, check=False)
    print(f"=== {environment.name} x {column.name} exit: {result.returncode} ===", flush=True)
    return result.returncode == 0


def _fmt(value: float | None, places: int = 3, signed: bool = True) -> str:
    if value is None:
        return "—"
    return f"{value:+.{places}f}" if signed else f"{value:.{places}f}"


def render_table(environments: list[Environment]) -> Path:
    cells: dict[tuple[str, str], dict[str, Any]] = {}
    for summary_path in RESULTS_DIR.glob("*/summary.json"):
        recording, _, module_key = summary_path.parent.name.rpartition("__")
        try:
            summary = json.loads(summary_path.read_text())
        except (json.JSONDecodeError, OSError):
            continue
        cells[(recording, module_key)] = summary.get("scores", {})

    column_keys = [f"{column.module_dir}.PGO.{column.name}" for column in COLUMNS]
    header = "| environment | " + " | ".join(column.name for column in COLUMNS) + " |"
    sep = "|" + "---|" * (len(COLUMNS) + 1)
    lines = [
        "# PGO benchmark — environments x implementations",
        "",
        "Each cell: **voxel improvement** (fractional drop in occupied 0.2 m voxels "
        "after re-anchoring scans onto the corrected trajectory; the universal, "
        "ground-truth-free score) — then `tag:<april-tag improvement>` where a camera "
        "exists, and `cl<closures>`. Higher is better; `—` = not yet run / N/A.",
        "",
        header,
        sep,
    ]
    for environment in environments:
        row_cells = []
        for column_key in column_keys:
            scores = cells.get((environment.name, column_key))
            if scores is None:
                row_cells.append("—")
                continue
            voxel = _fmt(scores.get("voxel_improvement"))
            tag = scores.get("tag_improvement")
            closures = scores.get("closures")
            text = voxel
            if tag is not None:
                text += f" tag:{tag:+.2f}"
            if closures is not None:
                text += f" cl{closures}"
            row_cells.append(text)
        lines.append(f"| {environment.name} | " + " | ".join(row_cells) + " |")

    # Per-column mean voxel improvement (over environments that have a number).
    lines += ["", "## Mean voxel improvement per column", ""]
    lines.append("| " + " | ".join(column.name for column in COLUMNS) + " |")
    lines.append("|" + "---|" * len(COLUMNS))
    means = []
    for column_key in column_keys:
        values = [
            cells[(environment.name, column_key)]["voxel_improvement"]
            for environment in environments
            if (environment.name, column_key) in cells
            and cells[(environment.name, column_key)].get("voxel_improvement") is not None
        ]
        means.append(f"{sum(values) / len(values):+.3f}" if values else "—")
    lines.append("| " + " | ".join(means) + " |")
    lines.append("")

    RESULTS_DIR.mkdir(exist_ok=True)
    TABLE_PATH.write_text("\n".join(lines) + "\n")
    return TABLE_PATH


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--go2-root", type=Path, default=DEFAULT_GO2_ROOT)
    parser.add_argument("--with-hk-village", action="store_true")
    parser.add_argument("--data-dir", type=Path, default=LFS_DATA_DIR)
    parser.add_argument("--only-env", help="comma-separated environment names")
    parser.add_argument("--only-col", help="comma-separated column names")
    parser.add_argument("--force", action="store_true", help="recompute fresh cells too")
    parser.add_argument(
        "--attempts", type=int, default=2, help="retries per cell on transient RPC timeouts"
    )
    parser.add_argument("--table-only", action="store_true", help="render from cache, run nothing")
    args = parser.parse_args()

    environments = discover_go2(args.go2_root.expanduser())
    if args.with_hk_village:
        environments += discover_hk_village(args.data_dir.expanduser())
    if args.only_env:
        wanted = {name.strip() for name in args.only_env.split(",")}
        environments = [environment for environment in environments if environment.name in wanted]

    columns = COLUMNS
    if args.only_col:
        wanted = {name.strip() for name in args.only_col.split(",")}
        columns = [column for column in COLUMNS if column.name in wanted]

    if args.table_only:
        print(f"table -> {render_table(environments)}")
        return

    total = len(environments) * len(columns)
    print(f"benchmark: {len(environments)} environments x {len(columns)} columns = {total} cells")
    done = skipped = failed = 0
    for environment in environments:
        for column in columns:
            if not args.force and cell_is_fresh(environment, column):
                skipped += 1
                print(f"skip (fresh): {environment.name} x {column.name}", flush=True)
                continue
            # Retry transient macOS LCM startup-RPC timeouts; a fresh process
            # almost always gets past them. Kill zombies between attempts.
            ok = False
            for attempt in range(1, args.attempts + 1):
                ok = run_cell(environment, column)
                if ok:
                    break
                _kill_zombies()
                if attempt < args.attempts:
                    print(
                        f"retry {attempt + 1}/{args.attempts}: {environment.name} x {column.name}"
                    )
                    time.sleep(5)
            done += 1 if ok else 0
            failed += 0 if ok else 1
            render_table(environments)  # refresh after every cell — live + crash-safe

    table = render_table(environments)
    print(f"\ncells: {done} ran, {skipped} cached, {failed} failed")
    print(f"table -> {table}")


if __name__ == "__main__":
    main()
