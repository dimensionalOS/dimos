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

Fills one big table: every environment (go2 recordings, hk_village) scored
against every PGO column (gsc_pgo in its default config plus a couple of
one-feature ablations). Each cell is one eval.py subprocess (sequential — they
share the isolated LCM bus).

CHECKPOINTED: a cell is skipped when its summary.json already exists and its
fingerprint still matches the db (size+mtime) and EVAL_VERSION. Kill it any
time and rerun — only missing/stale cells recompute. `--force` recomputes all.

The universal score is **voxel agreement** (re-anchoring scans onto the
corrected trajectory should collapse double walls — ground-truth-free and
needs no camera), so tagless environments (kitti, hk_village) still get a
real number. April-tag agreement is reported additionally wherever a camera +
intrinsics file exists.

Usage:
    uv run python .../benchmark_table.py --dataset-dir ~/datasets/RECORDINGS_DIR
    ... [--with-hk-village] [--force] [--only-env NAME] [--only-col NAME] [--table-only]
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

LFS_DATA_DIR = LOOP_CLOSURE_DIR.parents[4] / "data"  # repo_root/data (hk_village*.db)


@dataclass(frozen=True)
class Environment:
    """One dataset = one table row."""

    db_path: str  # mem2.db path, relative to the dataset dir
    odom_stream: str
    lidar_stream: str
    camera_stream: str | None = None
    intrinsics_json: str | None = None  # camera-intrinsics file, relative to the dataset dir


@dataclass(frozen=True)
class LoopClosureModule:
    """One implementation (+ optional config override) = one table column."""

    module_name: str  # class name inside module.py (e.g. PGO)
    module_dir: str  # subdir under loop_closure/ holding module.py
    config: dict[str, Any] = field(default_factory=dict)


# Ordered; the dict key is the row name and the results-dir key. Bare keys use pointlio;
# a `_fastlio` suffix marks the fastlio-odom variant of the same recording. Paths are
# relative to --dataset-dir. grassy_field last (feature-poor field, lowest priority).
RECORDINGS: dict[str, Environment] = {
    "huge_loop_realsense": Environment(
        db_path="2026-06-04_12-57pm-PST__huge_loop_realsense/mem2.db",
        odom_stream="pointlio_odometry",
        lidar_stream="pointlio_lidar",
        camera_stream="color_image",
        intrinsics_json="2026-06-04_12-57pm-PST__huge_loop_realsense/camera_intrinsics.json",
    ),
    "huge_loop_realsense_fastlio": Environment(
        db_path="2026-06-04_12-57pm-PST__huge_loop_realsense/mem2.db",
        odom_stream="fastlio_odometry",
        lidar_stream="fastlio_lidar",
        camera_stream="color_image",
        intrinsics_json="2026-06-04_12-57pm-PST__huge_loop_realsense/camera_intrinsics.json",
    ),
    "huge_loop_go2": Environment(
        db_path="2026-06-04_12-56pm-PST__huge_loop_go2/mem2.db",
        odom_stream="pointlio_odometry",
        lidar_stream="pointlio_lidar",
        camera_stream="color_image",
        intrinsics_json="2026-06-04_12-56pm-PST__huge_loop_go2/camera_intrinsics.json",
    ),
    "china_office1": Environment(
        db_path="2026-06-12_03-26am-PST__china_office1/mem2.db",
        odom_stream="pointlio_odometry",
        lidar_stream="pointlio_lidar",
        camera_stream="color_image",
        intrinsics_json="2026-06-12_03-26am-PST__china_office1/camera_intrinsics.json",
    ),
    "gir_stairs2": Environment(
        db_path="2026-06-02_10-03pm-PST__gir_stairs2/mem2.db",
        odom_stream="pointlio_odometry",
        lidar_stream="pointlio_lidar",
        camera_stream="color_image",
        intrinsics_json="2026-06-02_10-03pm-PST__gir_stairs2/camera_intrinsics.json",
    ),
    "outdoor_small_loop": Environment(
        db_path="2026-05-28_10-54am-PST__outdoor_small_loop/mem2.db",
        odom_stream="fastlio_odometry",
        lidar_stream="fastlio_lidar",
        camera_stream="color_image",
        intrinsics_json="2026-05-28_10-54am-PST__outdoor_small_loop/camera_intrinsics.json",
    ),
    "gir_stairs1": Environment(
        db_path="2026-06-01_6-05pm-PST__gir_stairs1/mem2.db",
        odom_stream="fastlio_odometry",
        lidar_stream="fastlio_lidar",
        camera_stream="color_image",
        intrinsics_json="2026-06-01_6-05pm-PST__gir_stairs1/camera_intrinsics.json",
    ),
    "gir_park1": Environment(
        db_path="2026-06-02_11-12pm-PST__gir_park1/mem2.db",
        odom_stream="fastlio_odometry",
        lidar_stream="fastlio_lidar",
        camera_stream="color_image",
        intrinsics_json="2026-06-02_11-12pm-PST__gir_park1/camera_intrinsics.json",
    ),
    "gir_park1_2": Environment(
        db_path="2026-06-02_11-33pm-PST__gir_park1_2/mem2.db",
        odom_stream="fastlio_odometry",
        lidar_stream="fastlio_lidar",
        camera_stream="color_image",
        intrinsics_json="2026-06-02_11-33pm-PST__gir_park1_2/camera_intrinsics.json",
    ),
    "grassy_field": Environment(
        db_path="2026-06-01_05-32pm-PST__grassy_field/mem2.db",
        odom_stream="fastlio_odometry",
        lidar_stream="fastlio_lidar",
        camera_stream="color_image",
        intrinsics_json="2026-06-01_05-32pm-PST__grassy_field/camera_intrinsics.json",
    ),
}


LOOP_CLOSURE_MODULES: dict[str, LoopClosureModule] = {
    "gsc_pgo": LoopClosureModule(module_name="PGO", module_dir="gsc_pgo"),
    "gsc_pgo_no_sc": LoopClosureModule(
        module_name="PGO", module_dir="gsc_pgo", config={"use_scan_context": False}
    ),
    "gsc_pgo_sc_far": LoopClosureModule(
        module_name="PGO",
        module_dir="gsc_pgo",
        config={"loop_candidate_max_distance_m": 0.0, "loop_score_thresh": 10000.0},
    ),
    # "ivan_pgo": LoopClosureModule(
    #     module_name="PGO", module_dir="ivan_pgo", config={"publish_global_map": False}
    # ),
    # "ivan_pgo_transformer": LoopClosureModule(module_name="PGO", module_dir="ivan_pgo_transformer"),
    # "unrefined_pgo": LoopClosureModule(module_name="PGO", module_dir="unrefined_pgo"),
}


def cell_is_fresh(name: str, module_key: str, module: LoopClosureModule, db_path: Path) -> bool:
    # eval.py's out_dir formula: <recording>__<package>.<class>[.<suffix>].
    cell_dir = RESULTS_DIR / f"{name}__{module.module_dir}.{module.module_name}.{module_key}"
    summary_path = cell_dir / "summary.json"
    if not summary_path.exists():
        return False
    try:
        summary = json.loads(summary_path.read_text())
    except (json.JSONDecodeError, OSError):
        return False
    fingerprint = summary.get("fingerprint", {})
    stat = db_path.stat()
    return (
        fingerprint.get("db_bytes") == stat.st_size
        and fingerprint.get("db_mtime") == int(stat.st_mtime)
        and fingerprint.get("version") is not None
    )


def run_cell(
    name: str,
    module_key: str,
    module: LoopClosureModule,
    environment: Environment,
    dataset_dir: Path,
) -> bool:
    command = [
        sys.executable,
        "-u",
        str(EVAL_PY),
        "--db-path",
        str(dataset_dir / environment.db_path),
        "--odom-stream",
        environment.odom_stream,
        "--lidar-stream",
        environment.lidar_stream,
        "--module-path",
        str(LOOP_CLOSURE_DIR / module.module_dir / "module.py"),
        "--module-name",
        module.module_name,
        "--recording-name",
        name,
        "--results-suffix",
        module_key,
        "--with-rrd",
        "false",
        "--lockstep",
        "true",
    ]
    if environment.camera_stream is not None:
        command += ["--camera-stream", environment.camera_stream]
    if environment.intrinsics_json is not None:
        command += ["--camera-intrinsics-json-path", str(dataset_dir / environment.intrinsics_json)]
    if module.config:
        command += ["--pgo-config-json", json.dumps(module.config)]
    print(f"\n=== {name} x {module_key} ===", flush=True)
    result = subprocess.run(command, check=False)
    print(f"=== {name} x {module_key} exit: {result.returncode} ===", flush=True)
    return result.returncode == 0


def render_table(environments: dict[str, Environment]) -> Path:
    cells: dict[tuple[str, str], dict[str, Any]] = {}
    for summary_path in RESULTS_DIR.glob("*/summary.json"):
        recording, _, column_key = summary_path.parent.name.rpartition("__")
        try:
            summary = json.loads(summary_path.read_text())
        except (json.JSONDecodeError, OSError):
            continue
        cells[(recording, column_key)] = summary.get("scores", {})

    column_keys = [
        f"{module.module_dir}.{module.module_name}.{key}"
        for key, module in LOOP_CLOSURE_MODULES.items()
    ]
    header = "| environment | " + " | ".join(LOOP_CLOSURE_MODULES) + " |"
    sep = "|" + "---|" * (len(LOOP_CLOSURE_MODULES) + 1)
    lines = [
        "# PGO benchmark — environments x implementations",
        "",
        "Each cell: **voxel improvement** (fractional drop in occupied 0.2 m voxels "
        "after re-anchoring scans onto the corrected trajectory; the universal, "
        "ground-truth-free score) — then `tag:<april-tag improvement>` where a camera "
        "exists, and `cl<closures>`. Higher is better; `—` = not yet run / N/A.",
        "",
        "How to read a cell, e.g. `+0.016 tag:-0.73 cl94`: `+0.016` = voxel "
        "improvement (occupied 0.2 m voxels dropped 1.6% after re-anchoring onto the "
        "corrected trajectory; positive = tighter map, negative = smeared worse than "
        "raw odometry). `tag:-0.73` = april-tag improvement, the fractional drop in "
        "per-visit tag-position spread (`+1.00` = revisits collapse to a point, "
        "`0.00` = no change, negative = worse; here 73% worse). `cl94` = 94 loop "
        "closures accepted.",
        "",
        header,
        sep,
    ]
    for name in environments:
        row_cells = []
        for column_key in column_keys:
            scores = cells.get((name, column_key))
            if scores is None:
                row_cells.append("—")
                continue
            voxel_value = scores.get("voxel_improvement")
            voxel = "—" if voxel_value is None else f"{voxel_value:+.3f}"
            tag = scores.get("tag_improvement")
            closures = scores.get("closures")
            text = voxel
            if tag is not None:
                text += f" tag:{tag:+.2f}"
            if closures is not None:
                text += f" cl{closures}"
            row_cells.append(text)
        lines.append(f"| {name} | " + " | ".join(row_cells) + " |")

    # Per-column mean voxel improvement (over environments that have a number).
    lines += ["", "## Mean voxel improvement per column", ""]
    lines.append("| " + " | ".join(LOOP_CLOSURE_MODULES) + " |")
    lines.append("|" + "---|" * len(LOOP_CLOSURE_MODULES))
    means = []
    for column_key in column_keys:
        values = [
            cells[(name, column_key)]["voxel_improvement"]
            for name in environments
            if (name, column_key) in cells
            and cells[(name, column_key)].get("voxel_improvement") is not None
        ]
        means.append(f"{sum(values) / len(values):+.3f}" if values else "—")
    lines.append("| " + " | ".join(means) + " |")
    lines.append("")

    RESULTS_DIR.mkdir(exist_ok=True)
    TABLE_PATH.write_text("\n".join(lines) + "\n")
    return TABLE_PATH


EXAMPLE_USAGE = """\
examples:
  # run the whole table (every recording x every module/config), resuming cached cells
  uv run python .../benchmark_table.py --dataset-dir ~/datasets/RECORDINGS_DIR

  # only one recording row, only the stock gsc_pgo column
  ... --dataset-dir ~/datasets/RECORDINGS_DIR --only-env huge_loop_realsense --only-col gsc_pgo

  # recompute everything from scratch (ignore cached summaries)
  ... --dataset-dir ~/datasets/RECORDINGS_DIR --force

  # just re-render the markdown table from cached results, run nothing
  ... --dataset-dir ~/datasets/RECORDINGS_DIR --table-only

  # also run the hk_village recordings (always read from the repo LFS data dir)
  ... --dataset-dir ~/datasets/RECORDINGS_DIR --with-hk-village
"""


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog=EXAMPLE_USAGE,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--dataset-dir",
        type=Path,
        required=True,
        help="dir holding the go2 recordings (RECORDINGS paths are relative to this)",
    )
    parser.add_argument(
        "--with-hk-village",
        action="store_true",
        help="also run the hk_village recordings (always read from the repo LFS data dir)",
    )
    parser.add_argument("--only-env", help="comma-separated environment names")
    parser.add_argument("--only-col", help="comma-separated column names")
    parser.add_argument("--force", action="store_true", help="recompute fresh cells too")
    parser.add_argument(
        "--attempts", type=int, default=2, help="retries per cell on transient RPC timeouts"
    )
    parser.add_argument("--table-only", action="store_true", help="render from cache, run nothing")
    args = parser.parse_args()

    dataset_dir = args.dataset_dir.expanduser()
    environments = {
        name: environment
        for name, environment in RECORDINGS.items()
        if (dataset_dir / environment.db_path).exists()
    }
    if args.with_hk_village:
        # hk_village dbs live in the repo LFS data dir, never the recordings dataset dir.
        # They publish world-frame lidar + PoseStamped odom, no camera, so they score on
        # voxel agreement alone.
        for db_path in sorted(LFS_DATA_DIR.glob("hk_village*.db")):
            environments[db_path.stem] = Environment(
                db_path=str(db_path), odom_stream="odom", lidar_stream="lidar"
            )
    if args.only_env:
        wanted = {name.strip() for name in args.only_env.split(",")}
        environments = {name: env for name, env in environments.items() if name in wanted}

    modules = LOOP_CLOSURE_MODULES
    if args.only_col:
        wanted = {name.strip() for name in args.only_col.split(",")}
        modules = {key: module for key, module in LOOP_CLOSURE_MODULES.items() if key in wanted}

    if args.table_only:
        print(f"table -> {render_table(environments)}")
        return

    total = len(environments) * len(modules)
    print(f"benchmark: {len(environments)} environments x {len(modules)} columns = {total} cells")
    done = skipped = failed = 0
    for name, environment in environments.items():
        db_path = dataset_dir / environment.db_path
        for module_key, module in modules.items():
            if not args.force and cell_is_fresh(name, module_key, module, db_path):
                skipped += 1
                print(f"skip (fresh): {name} x {module_key}", flush=True)
                continue
            # Retry transient macOS LCM startup-RPC timeouts; a fresh process
            # almost always gets past them. Kill leftover native procs between attempts.
            ok = False
            for attempt in range(1, args.attempts + 1):
                ok = run_cell(name, module_key, module, environment, dataset_dir)
                if ok:
                    break
                subprocess.run(
                    "lsof -ti tcp:7766 2>/dev/null | xargs kill -9 2>/dev/null;"
                    ' pkill -9 -f "bin/pgo|scene_lidar" 2>/dev/null',
                    shell=True,
                    check=False,
                )
                if attempt < args.attempts:
                    print(f"retry {attempt + 1}/{args.attempts}: {name} x {module_key}")
                    time.sleep(5)
            done += 1 if ok else 0
            failed += 0 if ok else 1
            render_table(environments)  # refresh after every cell — live + crash-safe

    table = render_table(environments)
    print(f"\ncells: {done} ran, {skipped} cached, {failed} failed")
    print(f"table -> {table}")


if __name__ == "__main__":
    main()
