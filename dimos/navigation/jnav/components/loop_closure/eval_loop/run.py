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

"""Download, prep and replay every recording (or the named ones) live through both PGOs, then print the table.

usage: run.py [--tables] [<dataset> ...]      datasets: see recordings.py
       --tables: only print the table from existing summaries
"""

import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from paths import OUT, PGO_EVAL
from recordings import BY_NAME, DATASETS

HERE = Path(__file__).resolve().parent
METHODS = [("gsc_live", "gsc_pgo"), ("ivan_live", "Ivan's live module")]


def step(script: str, *args: str) -> None:
    subprocess.run([sys.executable, str(HERE / script), *args], check=True)


def latency_rows(name: str, key: str) -> np.ndarray:
    """(arrival ts, ms from arrival to done) per processed scan."""
    path = OUT / (
        f"{name}.gsc_live.json" if key == "gsc_live" else f"{name}.ivan_live.latency.json"
    )
    rows = json.loads(path.read_text())
    return np.asarray(rows["latency"] if isinstance(rows, dict) else rows, float)


def rss_samples(name: str, key: str) -> np.ndarray | None:
    """(ts, resident MB) every 300 processed scans, or None for a run that didn't record it."""
    path = OUT / (
        f"{name}.gsc_live.json" if key == "gsc_live" else f"{name}.ivan_live.latency.json"
    )
    rows = json.loads(path.read_text())
    samples = rows.get("rss_samples") if isinstance(rows, dict) else None
    return np.asarray(samples, float) if samples else None


def latency(name: str, key: str) -> str:
    """Scan arrival -> done, ms: median / p99 / max."""
    values = latency_rows(name, key)[:, 1]
    return "/".join(f"{np.percentile(values, q):.0f}" for q in (50, 99, 100))


def print_speed_table(datasets) -> None:
    """How each module's latency moves over a recording: the mean in its first and last 10 minutes,
    its worst 10-minute mean, how many scans took over 100 ms or 1 s, and its resident memory at the
    first scan and at its peak (MB)."""
    print(
        "\n| recording | module | mean ms, first 10 min | mean ms, last 10 min | worst 10-min mean |"
        " worst ms | over 100 ms | over 1 s | dropped | RAM MB, start -> peak |"
    )
    print("|---|---|---|---|---|---|---|---|---|---|")
    for dataset in datasets:
        path = OUT / f"{dataset.name}.summary.json"
        if not path.exists():
            continue
        row = json.loads(path.read_text())
        if row["duration_s"] < 1200:
            continue
        for key, label in METHODS:
            if key not in row or not (OUT / f"{dataset.name}.gsc_live.json").exists():
                continue
            rows = latency_rows(dataset.name, key)
            minutes = (rows[:, 0] - rows[0, 0]) / 60.0
            ms = rows[:, 1]
            windows = [
                ms[(minutes >= start) & (minutes < start + 10)]
                for start in np.arange(0, minutes[-1] - 10, 1)
            ]
            worst = max(window.mean() for window in windows if len(window))
            rss = rss_samples(dataset.name, key)
            ram = "-" if rss is None else f"{rss[0, 1]:.0f} -> {rss[:, 1].max():.0f}"
            print(
                f"| {dataset.name} | {label} | {ms[minutes < 10].mean():.0f} | "
                f"{ms[minutes > minutes[-1] - 10].mean():.0f} | {worst:.0f} | {ms.max():.0f} | "
                f"{(ms > 100).mean() * 100:.1f}% | {(ms > 1000).sum()} | {dropped(dataset.name, key, row)} | {ram} |"
            )


def dropped(name: str, key: str, row: dict) -> int:
    if key == "ivan_live":
        return row[key]["scans_dropped"]
    return json.loads((OUT / f"{name}.gsc_live.json").read_text())["scans_dropped"]


def print_table(datasets) -> None:
    print(
        "\n| recording | scans | raw tags | "
        + " | ".join(f"{label} loops / voxels / tags / dropped / latency" for _, label in METHODS)
        + " |"
    )
    print("|---|---|---|" + "---|" * len(METHODS))
    for dataset in datasets:
        path = OUT / f"{dataset.name}.summary.json"
        if not path.exists():
            continue
        row = json.loads(path.read_text())
        raw_tags = row.get("tag_spread_raw_m")
        cells = []
        for key, _ in METHODS:
            live = (
                (OUT / f"{dataset.name}.gsc_live.json").exists()
                if key == "gsc_live"
                else row.get(key, {}).get("replay") == "live"
            )
            if key not in row or not live:
                cells.append("-")
                continue
            stats = row[key]
            voxels = f"{(stats['voxels'] / row['voxels_raw'] - 1) * 100:+.1f}%"
            tags = "-" if stats.get("tag_spread_m") is None else f"{stats['tag_spread_m']:.2f}"
            cells.append(
                f"{stats['loops']} / {voxels} / {tags} / {dropped(dataset.name, key, row)} / "
                f"{latency(dataset.name, key)}"
            )
        raw = "-" if raw_tags is None else f"{raw_tags:.2f}"
        print(f"| {dataset.name} | {row['scans']} | {raw} | " + " | ".join(cells) + " |")


def main() -> None:
    args = [arg for arg in sys.argv[1:] if arg != "--tables"]
    unknown = [name for name in args if name not in BY_NAME]
    if unknown:
        raise SystemExit(f"unknown dataset(s) {unknown}; choose from {list(BY_NAME)}")
    datasets = [BY_NAME[name] for name in args] or DATASETS
    if "--tables" not in sys.argv:
        if not PGO_EVAL.exists():
            raise SystemExit(f"no pgo-eval at {PGO_EVAL}; build it first (README, Reproduce)")
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
                    *([f"--voxel={dataset.voxel_m}"] if dataset.voxel_m else []),
                )
            step("bench.py", dataset.name if dataset.odom else f"{dataset.name}={source}")
            step("variant.py", "gsc_live", "{}", dataset.name)
            step("ivan_live.py", dataset.name)
    print_table(datasets)
    print_speed_table(datasets)


if __name__ == "__main__":
    main()
