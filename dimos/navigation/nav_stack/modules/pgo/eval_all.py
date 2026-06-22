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

"""Cross-algorithm PGO eval: every recording x every loop-closure algorithm.

Fills one comparison table (recordings = rows, algorithms = columns). Each cell
is one eval.py subprocess with lockstep replay (sequential — they share the
isolated LCM bus). The table is re-rendered to eval_results/comparison.md after
every cell, so it grows live and survives a kill.

CHECKPOINTED: a cell is skipped when its summary.json already exists and its
fingerprint still matches the db (size+mtime) and EVAL_VERSION. Kill it any
time and rerun — only missing/stale cells recompute. `--force` recomputes all.

Algorithms compared:
  * pgo              — the nav-stack C++ PGO (GTSAM iSAM2 + PCL ICP)
  * ivan_pgo         — Ivan's pure-Python PGO (GTSAM + Open3D ICP)
  * ivan_transformer — Ivan's current incremental PGO core, online-wrapped
  * unrefined_pgo    — the frozen unrefined C++ baseline pgo was refined from

Runs china_office1 (on pointlio odometry) first, then the rest on fastlio.

Usage:
    uv run python dimos/navigation/nav_stack/modules/pgo/eval_all.py
    ... [--recordings-root ~/datasets/go2_recordings] [--force]
    ... [--only-env NAME] [--only-module NAME] [--table-only]
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

PGO_DIR = Path(__file__).resolve().parent
EVAL_PY = PGO_DIR / "eval.py"
RESULTS_DIR = PGO_DIR / "eval_results"
TABLE_PATH = RESULTS_DIR / "comparison.md"

# Each cell subprocess runs on its own LCM multicast port (see isolate_lcm in eval.py); set pre-launch so workers inherit it.
_LCM_GROUP = "239.255.76.67"
_LCM_BASE_PORT = 7800
_LCM_PORT_SPAN = 100

DEFAULT_RECORDINGS_ROOT = Path("~/datasets/go2_recordings").expanduser()
SIDECAR_NAME = "camera_intrinsics.json"

# Loop-closure thresholds shared across the native columns (eval.py applies
# these as DEFAULT_PGO_CONFIG too; unknown keys are dropped per-module).
_NATIVE_BASE: dict[str, Any] = {
    "loop_search_radius": 3.0,
    "loop_time_thresh": 5.0,
    "min_loop_detect_duration": 2.0,
    "key_pose_delta_trans": 0.5,
    "use_scan_context": True,
    # The global map is a big cloud nothing in the eval consumes; its publishes
    # congest the corrected_odometry ack channel the lockstep replay waits on.
    "global_map_publish_rate": 0.0,
    # Lockstep waits for one corrected_odometry ack per scan; if PGO drops the
    # scan as stale it never acks and the replay stalls out on ack timeouts.
    "drain_stale_scans": False,
}


@dataclass(frozen=True)
class Algorithm:
    """One loop-closure implementation = one table column."""

    name: str  # column label
    package: str  # results-dir module key segment (matches eval.py's out_dir)
    module_path: Path
    overrides: dict[str, Any] = field(default_factory=dict)


ALGORITHMS: list[Algorithm] = [
    Algorithm("pgo", "pgo", PGO_DIR / "pgo.py", dict(_NATIVE_BASE)),
    Algorithm(
        "ivan_pgo",
        "ivan_pgo",
        PGO_DIR / "ivan_pgo" / "module.py",
        {"publish_global_map": False},
    ),
    Algorithm(
        "ivan_transformer",
        "ivan_pgo_transformer",
        PGO_DIR / "ivan_pgo_transformer" / "module.py",
        {},
    ),
    Algorithm(
        "unrefined_pgo",
        "unrefined_pgo",
        PGO_DIR / "unrefined_pgo" / "module.py",
        dict(_NATIVE_BASE),
    ),
]


@dataclass(frozen=True)
class Environment:
    """One recording (+ odometry choice) = one table row."""

    name: str  # results-dir recording key (row label; encodes odom choice)
    db_path: Path
    odom_stream: str
    lidar_stream: str
    camera_stream: str | None = None
    intrinsics_json: Path | None = None
    ignore_tags: str = ""  # comma-separated dynamic/unreliable tag ids


# Per-recording dynamic/unreliable April tags to drop (motion looks like drift).
IGNORE_TAGS_BY_RECORDING: dict[str, str] = {
    "2026-06-04_12-57pm-PST__huge_loop_realsense": "17",
}

# Recording evaluated first, on pointlio odometry instead of fastlio.
POINTLIO_FIRST = "2026-06-12_03-26am-PST__china_office1"


def _environment(db_path: Path, *, pointlio: bool) -> Environment:
    recording = db_path.parent
    sidecar = recording / SIDECAR_NAME
    odom = "pointlio_odometry" if pointlio else "fastlio_odometry"
    lidar = "pointlio_lidar" if pointlio else "fastlio_lidar"
    return Environment(
        name=f"{recording.name}_pointlio" if pointlio else recording.name,
        db_path=db_path,
        odom_stream=odom,
        lidar_stream=lidar,
        camera_stream="color_image",
        intrinsics_json=sidecar if sidecar.exists() else None,
        ignore_tags=IGNORE_TAGS_BY_RECORDING.get(recording.name, ""),
    )


def discover_environments(root: Path) -> list[Environment]:
    """china_office1 (pointlio) first, then every other recording on fastlio."""
    dbs = sorted(root.glob("*/mem2.db"))
    china = [db for db in dbs if db.parent.name == POINTLIO_FIRST]
    rest = [db for db in dbs if db.parent.name != POINTLIO_FIRST]
    environments = [_environment(db, pointlio=True) for db in china]
    environments += [_environment(db, pointlio=False) for db in rest]
    return environments


def cell_dir(environment: Environment, algorithm: Algorithm) -> Path:
    # Mirrors eval.py's out_dir formula: <recording>__<package>.PGO.
    return RESULTS_DIR / f"{environment.name}__{algorithm.package}.PGO"


def cell_is_fresh(environment: Environment, algorithm: Algorithm) -> bool:
    summary_path = cell_dir(environment, algorithm) / "summary.json"
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
        ' pkill -9 -f "bin/pgo" 2>/dev/null',
        shell=True,
        check=False,
    )


def run_cell(environment: Environment, algorithm: Algorithm) -> bool:
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
        str(algorithm.module_path),
        "--module-name",
        "PGO",
        "--recording-name",
        environment.name,
        "--with-rrd",
        "false",
        "--lockstep",
        "true",
    ]
    if environment.camera_stream is not None:
        command += ["--camera-stream", environment.camera_stream]
    if environment.intrinsics_json is not None:
        command += ["--camera-intrinsics-json-path", str(environment.intrinsics_json)]
    if environment.ignore_tags:
        command += ["--ignore-tags", environment.ignore_tags]
    if algorithm.overrides:
        command += ["--pgo-config-json", json.dumps(algorithm.overrides)]
    print(f"\n=== {environment.name} x {algorithm.name} ===", flush=True)
    env = dict(os.environ)
    env.setdefault(
        "LCM_DEFAULT_URL",
        f"udpm://{_LCM_GROUP}:{_LCM_BASE_PORT + os.getpid() % _LCM_PORT_SPAN}",
    )
    result = subprocess.run(command, check=False, env=env)
    print(f"=== {environment.name} x {algorithm.name} exit: {result.returncode} ===", flush=True)
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

    column_keys = [f"{algorithm.package}.PGO" for algorithm in ALGORITHMS]
    header = "| recording | " + " | ".join(algorithm.name for algorithm in ALGORITHMS) + " |"
    sep = "|" + "---|" * (len(ALGORITHMS) + 1)
    lines = [
        "# PGO comparison — recordings x algorithms",
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
            text = _fmt(scores.get("voxel_improvement"))
            tag = scores.get("tag_improvement")
            closures = scores.get("closures")
            if tag is not None:
                text += f" tag:{tag:+.2f}"
            if closures is not None:
                text += f" cl{closures}"
            row_cells.append(text)
        lines.append(f"| {environment.name} | " + " | ".join(row_cells) + " |")

    # Per-column mean voxel improvement (over recordings that have a number).
    lines += ["", "## Mean voxel improvement per algorithm", ""]
    lines.append("| " + " | ".join(algorithm.name for algorithm in ALGORITHMS) + " |")
    lines.append("|" + "---|" * len(ALGORITHMS))
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
    parser.add_argument("--recordings-root", type=Path, default=DEFAULT_RECORDINGS_ROOT)
    parser.add_argument("--only-env", help="comma-separated recording row names")
    parser.add_argument("--only-module", help="comma-separated algorithm names")
    parser.add_argument("--force", action="store_true", help="recompute fresh cells too")
    parser.add_argument(
        "--attempts", type=int, default=2, help="retries per cell on transient RPC timeouts"
    )
    parser.add_argument("--table-only", action="store_true", help="render from cache, run nothing")
    args = parser.parse_args()

    environments = discover_environments(args.recordings_root.expanduser())
    if args.only_env:
        wanted = {name.strip() for name in args.only_env.split(",")}
        environments = [environment for environment in environments if environment.name in wanted]

    algorithms = ALGORITHMS
    if args.only_module:
        wanted = {name.strip() for name in args.only_module.split(",")}
        algorithms = [algorithm for algorithm in ALGORITHMS if algorithm.name in wanted]

    if args.table_only:
        print(f"table -> {render_table(environments)}")
        return

    total = len(environments) * len(algorithms)
    print(
        f"eval_all: {len(environments)} recordings x {len(algorithms)} algorithms = {total} cells"
    )
    done = skipped = failed = 0
    for environment in environments:
        for algorithm in algorithms:
            if not args.force and cell_is_fresh(environment, algorithm):
                skipped += 1
                print(f"skip (fresh): {environment.name} x {algorithm.name}", flush=True)
                continue
            # Retry transient LCM startup-RPC timeouts; a fresh process almost
            # always gets past them. Kill zombies between attempts.
            ok = False
            for attempt in range(1, args.attempts + 1):
                ok = run_cell(environment, algorithm)
                if ok:
                    break
                _kill_zombies()
                if attempt < args.attempts:
                    print(
                        f"retry {attempt + 1}/{args.attempts}: "
                        f"{environment.name} x {algorithm.name}"
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
