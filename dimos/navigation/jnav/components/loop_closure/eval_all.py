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

"""Run the loop-closure eval over every PGO module and render a comparison table.

Each module is evaluated in its own subprocess (sequentially — they share LCM)
via eval.py with lockstep replay, then all eval_results summaries are rendered
to eval_results/comparison.md. A failed module doesn't stop the rest.

Usage:
    uv run python dimos/navigation/jnav/components/loop_closure/eval_all.py \\
        --db-path ~/datasets/go2_recordings/<recording>/mem2.db \\
        [--camera-intrinsics-json-path <path>]   # default: sidecar next to db
        [--only gsc_pgo,ivan_pgo]                # subset by directory name
        [--with-rrd true] [--lockstep true]
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path
import subprocess
import sys
from typing import Any

LOOP_CLOSURE_DIR = Path(__file__).resolve().parent
RESULTS_DIR = LOOP_CLOSURE_DIR / "eval_results"
TABLE_PATH = RESULTS_DIR / "comparison.md"
SIDECAR_NAME = "camera_intrinsics.json"

# (directory, per-module config overrides). All classes are named PGO.
# global_map is disabled everywhere — nothing in the eval consumes it, and the
# native modules' big global_map clouds overflow LCM and congest the
# corrected_odometry ack channel the lockstep replay waits on (rate 0 = off
# for the native binaries; publish_global_map=False for the python ivan).
MODULES: list[tuple[str, dict[str, Any]]] = [
    ("gsc_pgo", {"use_scan_context": True, "global_map_publish_rate": 0.0}),
    ("ivan_pgo", {"publish_global_map": False}),
    ("ivan_pgo_transformer", {}),
    ("unrefined_pgo", {"global_map_publish_rate": 0.0}),
]


def _fmt_spread(s: dict[str, Any]) -> str:
    raw, corrected = s.get("raw_spread_m"), s.get("corrected_spread_m")
    if raw is None or corrected is None:  # tagless dataset (KITTI, bare lidar)
        return "—"
    return f"{raw:.2f} -> {corrected:.2f}"


TABLE_COLUMNS = [
    ("tag spread (m)", _fmt_spread),
    (
        "tag improvement",
        lambda s: f"{s['tag_improvement']:+.3f}" if s.get("tag_improvement") is not None else "—",
    ),
    (
        "voxel improvement",
        lambda s: f"{s['voxel_improvement']:+.3f}"
        if s.get("voxel_improvement") is not None
        else "—",
    ),
    (
        "drift recovery",  # ATE improvement vs un-drifted GT (only with --drift-per-sec)
        lambda s: f"{s['trajectory_improvement']:+.3f}"
        if s.get("trajectory_improvement") is not None
        else "—",
    ),
    ("closures", lambda s: str(s["closures"])),
    ("keyframes", lambda s: str(s["keyframes"])),
    ("runtime (s)", lambda s: str(s["runtime_s"])),
]


def run_one(
    module_dir: str,
    overrides: dict[str, Any],
    *,
    db_path: Path,
    odom_stream: str,
    camera_stream: str,
    lidar_stream: str,
    intrinsics_json: Path | None,
    with_rrd: str,
    lockstep: str,
    results_suffix: str = "",
    drift_per_sec: str = "",
    ignore_tags: str = "",
) -> bool:
    command = [
        sys.executable,
        "-u",
        str(LOOP_CLOSURE_DIR / "eval.py"),
        "--db-path",
        str(db_path),
        "--odom-stream",
        odom_stream,
        "--camera-stream",
        camera_stream,
        "--lidar-stream",
        lidar_stream,
        "--module-path",
        str(LOOP_CLOSURE_DIR / module_dir / "module.py"),
        "--module-name",
        "PGO",
        "--with-rrd",
        with_rrd,
        "--lockstep",
        lockstep,
    ]
    if intrinsics_json is not None:
        command += ["--camera-intrinsics-json-path", str(intrinsics_json)]
    if drift_per_sec:
        command += ["--drift-per-sec", drift_per_sec]
    if ignore_tags:
        command += ["--ignore-tags", ignore_tags]
    if results_suffix:
        command += ["--results-suffix", results_suffix]
    if overrides:
        command += ["--pgo-config-json", json.dumps(overrides)]
    print(f"\n=== {module_dir} ===", flush=True)
    result = subprocess.run(command, check=False)
    print(f"=== {module_dir} exit: {result.returncode} ===", flush=True)
    return result.returncode == 0


def _row_label(module_key: str, replay: dict[str, Any]) -> str:
    label = module_key
    scans = replay.get("scans_sent")
    timeouts = replay.get("timeouts")
    if scans is not None:
        label += f" ({scans} scans, {timeouts} ack-timeouts)" if timeouts else f" ({scans} scans)"
    if replay.get("hit_max_run_s"):
        label += " [hit run cap]"
    return label


def _section_for(recording: str, module_key: str) -> str:
    """Which breakdown section a result belongs to."""
    if recording.startswith("hk_village"):
        return "hk_village"
    if "drift" in module_key:
        return "drift"
    return "real"


def _tag_improvement_key(labelled_row: tuple[str, dict[str, Any]]) -> float:
    # Best first; fall back to drift-recovery for tagless rows, then sink to the
    # bottom if neither is present.
    scores = labelled_row[1]
    value = scores.get("tag_improvement")
    if value is None:
        value = scores.get("trajectory_improvement")
    return float(value) if value is not None else float("-inf")


def _render_section(
    by_recording: dict[str, list[tuple[str, dict[str, Any]]]], sort_reverse: bool
) -> list[str]:
    lines: list[str] = []
    for recording in sorted(by_recording):
        rows = sorted(by_recording[recording], key=_tag_improvement_key, reverse=sort_reverse)
        lines += [f"### {recording}", ""]
        lines.append("| module | " + " | ".join(name for name, _ in TABLE_COLUMNS) + " | replay |")
        lines.append("|" + "---|" * (len(TABLE_COLUMNS) + 2))
        for module_label, scores in rows:
            cells = [render(scores) for _, render in TABLE_COLUMNS]
            lines.append(f"| {module_label} | " + " | ".join(cells) + f" | {scores['_mode']} |")
        lines.append("")
    return lines


def _base_module(module_key: str) -> str:
    """`gsc_pgo.PGO.drift0p05_0_0` -> `gsc_pgo`."""
    return module_key.split(".")[0]


def _conclusion(real: dict[str, list[tuple[str, dict[str, Any]]]]) -> list[str]:
    """Data-driven summary: count per-recording tag-improvement wins per module."""
    wins: dict[str, int] = defaultdict(int)
    tagged_recordings = 0
    for rows in real.values():
        tagged = [(label, s) for label, s in rows if s.get("tag_improvement") is not None]
        if not tagged:
            continue
        tagged_recordings += 1
        best_label = max(tagged, key=lambda item: item[1]["tag_improvement"])[0]
        wins[_base_module(best_label)] += 1
    if not tagged_recordings:
        return []
    ranking = sorted(wins.items(), key=lambda item: item[1], reverse=True)
    leader = ranking[0][0] if ranking else "n/a"
    lines = ["## Conclusion", ""]
    lines.append(
        f"Across {tagged_recordings} tagged real recordings, the best per-recording "
        f"April-tag improvement was won by:"
    )
    lines.append("")
    for module, count in ranking:
        lines.append(f"- **{module}** — best on {count}/{tagged_recordings}")
    lines += [
        "",
        f"`{leader}` is the strongest loop-closure PGO overall on real recordings; "
        "`unrefined_pgo` (the pass-through baseline) is the floor. On tagless data "
        "(hk_village) modules are ranked by voxel improvement, and under artificial "
        "drift by drift-recovery (fraction of injected ATE removed).",
        "",
    ]
    return lines


def render_table() -> Path:
    buckets: dict[str, dict[str, list[tuple[str, dict[str, Any]]]]] = {
        "real": defaultdict(list),
        "hk_village": defaultdict(list),
        "drift": defaultdict(list),
    }
    for summary_path in sorted(RESULTS_DIR.glob("*/summary.json")):
        summary = json.loads(summary_path.read_text())
        # Skip non-PGO summaries (e.g. the cross-recording ground-truth tag eval,
        # which has a different shape and its own write-up).
        if "scores" not in summary:
            continue
        recording, _, module_key = summary_path.parent.name.rpartition("__")
        replay = summary.get("replay", {})
        row = {**summary["scores"], "_mode": replay.get("mode", "fixed-rate (pre-lockstep)")}
        section = _section_for(recording, module_key)
        buckets[section][recording].append((_row_label(module_key, replay), row))

    lines = [
        "# Loop-closure PGO comparison",
        "",
        "Higher improvement = better. **Tag improvement**: fractional drop in per-visit",
        "April-tag position spread (1.0 = perfect). **Voxel improvement**: fractional drop",
        "in occupied 0.2 m voxels after re-anchoring scans onto the corrected trajectory.",
        "**Drift recovery**: fraction of injected ATE removed vs the un-drifted ground truth.",
        "",
    ]
    if buckets["real"]:
        lines += ["## Real recordings (clean input)", ""]
        lines += _render_section(buckets["real"], sort_reverse=True)
    if buckets["hk_village"]:
        lines += ["## hk_village (tagless — voxel agreement only)", ""]
        lines += _render_section(buckets["hk_village"], sort_reverse=True)
    if buckets["drift"]:
        lines += ["## Artificial drift robustness", ""]
        lines += _render_section(buckets["drift"], sort_reverse=True)

    kitti_table = RESULTS_DIR / "kitti_comparison.md"
    if kitti_table.exists():
        lines += [
            "## KITTI (official odometry error)",
            "",
            "Scored with the official KITTI translational (%) / rotational (deg/m) error "
            "(lower = better); see [kitti_comparison.md](kitti_comparison.md).",
            "",
        ]

    lines += _conclusion(buckets["real"])
    RESULTS_DIR.mkdir(exist_ok=True)
    TABLE_PATH.write_text("\n".join(lines))
    return TABLE_PATH


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db-path", type=Path, required=True)
    parser.add_argument("--odom-stream", default="fastlio_odometry")
    parser.add_argument("--camera-stream", default="color_image")
    parser.add_argument("--lidar-stream", default="fastlio_lidar")
    parser.add_argument(
        "--camera-intrinsics-json-path",
        type=Path,
        help=f"default: {SIDECAR_NAME} next to the db",
    )
    parser.add_argument("--only", help="comma-separated module directory names")
    parser.add_argument("--with-rrd", default="true", choices=["true", "false"])
    parser.add_argument("--lockstep", default="true", choices=["true", "false"])
    parser.add_argument(
        "--results-suffix",
        default="",
        help="extra results-dir key for runs with different inputs (e.g. pointlio)",
    )
    parser.add_argument(
        "--drift-per-sec",
        default="",
        help="inject constant-velocity world drift 'x,y,z' (m/s) into odom+lidar; "
        "stress-tests how each PGO corrects known drift. Auto-tags the results dir.",
    )
    parser.add_argument(
        "--ignore-tags",
        default="",
        help="comma-separated April-tag ids to drop from scoring (e.g. '17' for the "
        "dynamic tag in huge_loop_realsense)",
    )
    args = parser.parse_args()

    # Keep drifted runs in their own results dirs so they don't clobber the
    # clean-input comparison (e.g. drift 0.1,0,0 -> suffix '...drift0p1x').
    results_suffix = args.results_suffix
    if args.drift_per_sec:
        drift_tag = "drift" + args.drift_per_sec.replace(".", "p").replace(",", "_")
        results_suffix = f"{results_suffix}.{drift_tag}" if results_suffix else drift_tag

    db_path = args.db_path.expanduser()
    if not db_path.exists():
        raise SystemExit(f"no such db: {db_path}")
    # Tagless datasets (KITTI, bare lidar): empty --camera-stream -> no intrinsics
    # needed, voxel-agreement scoring only.
    tagless = not args.camera_stream
    intrinsics_json: Path | None = None
    if not tagless:
        sidecar: Path = args.camera_intrinsics_json_path or db_path.parent / SIDECAR_NAME
        intrinsics_json = sidecar.expanduser()
        if not intrinsics_json.exists():
            raise SystemExit(f"no intrinsics json: {intrinsics_json}")

    selected = MODULES
    if args.only:
        wanted = {name.strip() for name in args.only.split(",")}
        selected = [(name, overrides) for name, overrides in MODULES if name in wanted]
        missing = wanted - {name for name, _ in selected}
        if missing:
            raise SystemExit(
                f"unknown modules: {sorted(missing)} (have: {[m for m, _ in MODULES]})"
            )

    outcomes = {}
    for module_dir, overrides in selected:
        outcomes[module_dir] = run_one(
            module_dir,
            overrides,
            db_path=db_path,
            odom_stream=args.odom_stream,
            camera_stream=args.camera_stream,
            lidar_stream=args.lidar_stream,
            intrinsics_json=intrinsics_json,
            with_rrd=args.with_rrd,
            lockstep=args.lockstep,
            results_suffix=results_suffix,
            drift_per_sec=args.drift_per_sec,
            ignore_tags=args.ignore_tags,
        )

    table = render_table()
    print(f"\ntable -> {table}")
    failed = [name for name, ok in outcomes.items() if not ok]
    if failed:
        raise SystemExit(f"failed modules: {failed}")


if __name__ == "__main__":
    main()
