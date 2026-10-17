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

"""The README's claim figures, from results already in $EVAL_LOOP_OUT -> ./figures/*.png

usage: figures.py
- claim0_planner_load.png: MLS planner seconds per hour of driving for each way of taking loop closures
  (mls_summary.json)
- claim1_latency.png: each live module's output latency over the drive (live_timing.json and gsc's
  pgo-eval frame_timing, through each module's own queue, as compare_timing.py)
- claim3_<dataset>.png: the top-down renders of the low-drift recordings (render.py's PNGs)
- claim4_efficiency.png: each live module's memory and cumulative compute over the drive
"""

from collections import deque
import json
from pathlib import Path
import shutil

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from paths import OUT

HERE = Path(__file__).resolve().parent / "figures"
NAME = "huge_loop_pointlio"
IVAN, GSC = "#c0392b", "#2471a3"


def planner_load() -> None:
    summary = json.loads((OUT / f"{NAME}.mls_summary.json").read_text())
    whole = summary["whole"]
    rows = [
        ("Ivan's map, every publish", whole["seconds_of_mls_per_second_of_driving"] * 3600, IVAN),
        ("Ivan's map, rebuilds only", whole["rebuilds_only"]["mls_seconds_per_hour"], IVAN),
        (
            "gsc, full rebuild per closure",
            summary["gsc_closure_full_rebuilds"]["mls_seconds_per_hour"],
            GSC,
        ),
        (
            "Ivan's corrections, patched in place",
            summary["ivan_patch_in_place"]["mls_seconds_per_hour"],
            IVAN,
        ),
        (
            "gsc's corrections, patched in place",
            summary["gsc_patch_in_place"]["mls_seconds_per_hour"],
            GSC,
        ),
    ]
    fig, ax = plt.subplots(figsize=(8, 3.2), dpi=110)
    labels, values, colors = zip(*rows, strict=True)
    ax.barh(labels[::-1], values[::-1], color=colors[::-1])
    ax.set_xscale("log")
    ax.axvline(3600, color="#555", linestyle="--", linewidth=1)
    ax.text(3600, -0.45, " real time: 3600 s per hour", fontsize=8, color="#555", va="center")
    for i, value in enumerate(values[::-1]):
        ax.text(value, i, f" {value:,.0f} s", va="center", fontsize=8)
    ax.set_xlabel("MLS planner seconds per hour of driving (log)")
    ax.set_title(f"{NAME}: what the MLS planner pays to take loop closures", fontsize=10)
    ax.set_xlim(5, max(values) * 6)
    fig.tight_layout()
    fig.savefig(HERE / "claim0_planner_load.png")


def latencies(seconds, ms, emits, cap):
    """(arrival s, latency s) for each published output, one worker, queue keeps at most `cap`."""
    out, waiting, free_at = [], deque(), -np.inf

    def drain(now):
        nonlocal free_at
        while waiting and free_at <= now:
            index = waiting.popleft()
            free_at += ms[index] / 1e3
            if emits[index]:
                out.append((seconds[index], free_at - seconds[index]))

    for i, arrival in enumerate(seconds):
        drain(arrival)
        if free_at <= arrival and not waiting:
            free_at = arrival + ms[i] / 1e3
            if emits[i]:
                out.append((arrival, free_at - arrival))
            continue
        waiting.append(i)
        while len(waiting) > cap:
            waiting.popleft()
    drain(np.inf)
    return np.array(out)


def live_latency() -> None:
    live = np.array(json.loads((OUT / f"{NAME}.live_timing.json").read_text()))
    gsc = np.array(
        json.loads((OUT / "timing" / f"{NAME}.gsc_default.json").read_text())["frame_timing"]
    )
    t0 = live[0, 0]
    ivan = latencies(live[:, 0] - t0, live[:, 1] + live[:, 2], live[:, 6] >= 0, 1)
    ours = latencies(gsc[:, 0] - t0, gsc[:, 1] - gsc[:, 3], np.ones(len(gsc), bool), 100)
    fig, ax = plt.subplots(figsize=(8, 3.4), dpi=110)
    for (x, y), color, label in (
        (ivan.T, IVAN, "Ivan's live module (global map out)"),
        (ours.T, GSC, "gsc's live module (corrected odometry out)"),
    ):
        keep = y > 1e-3
        ax.scatter(
            x[keep] / 60, y[keep] * 1e3, s=2, color=color, alpha=0.5, label=label, linewidths=0
        )
    ax.axhline(100, color="#555", linestyle="--", linewidth=1)
    ax.text(0.5, 110, "100 ms (one 10 Hz lidar frame)", fontsize=8, color="#555")
    ax.set_yscale("log")
    ax.set_xlabel("drive time (min)")
    ax.set_ylabel("output latency (ms, log)")
    ax.set_title(
        f"{NAME}: scan arrival to published output, each module's own queue (outputs under 1 ms not drawn)",
        fontsize=9,
    )
    ax.legend(loc="upper left", fontsize=8, markerscale=4)
    fig.tight_layout()
    fig.savefig(HERE / "claim1_latency.png")


def efficiency() -> None:
    live = np.array(json.loads((OUT / f"{NAME}.live_timing.json").read_text()))
    result = json.loads((OUT / "timing" / f"{NAME}.gsc_default.json").read_text())
    gsc, rss = np.array(result["frame_timing"]), np.array(result["rss_samples"])
    t0 = live[0, 0]
    fig, (memory, compute) = plt.subplots(1, 2, figsize=(10, 3.4), dpi=110)
    memory.plot(
        (live[:, 0] - t0) / 60,
        np.maximum.accumulate(live[:, 8]) / 1e3,
        color=IVAN,
        label="Ivan's live module",
    )
    memory.plot(
        (rss[:, 0] - t0) / 60,
        np.maximum.accumulate(rss[:, 1]) / 1e3,
        color=GSC,
        label="gsc's live module",
    )
    memory.set_xlabel("drive time (min)")
    memory.set_ylabel("peak memory so far (GB)")
    memory.legend(fontsize=8)
    compute.plot(
        (live[:, 0] - t0) / 60,
        np.cumsum(live[:, 1] + live[:, 2]) / 1e3,
        color=IVAN,
        label="Ivan's live module",
    )
    compute.plot(
        (gsc[:, 0] - t0) / 60, np.cumsum(gsc[:, 1]) / 1e3, color=GSC, label="gsc, all threads"
    )
    compute.plot(
        (gsc[:, 0] - t0) / 60,
        np.cumsum(gsc[:, 1] - gsc[:, 3]) / 1e3,
        color=GSC,
        linestyle="--",
        label="gsc, pipeline thread",
    )
    compute.set_xlabel("drive time (min)")
    compute.set_ylabel("compute so far (s)")
    compute.legend(fontsize=8)
    fig.suptitle(f"{NAME}: memory and compute of each live module (no planner)", fontsize=10)
    fig.tight_layout()
    fig.savefig(HERE / "claim4_efficiency.png")


def low_drift_topdowns() -> None:
    for name in ("hk_village3", "hk_village4", "go2_hongkong_office"):
        source = OUT / f"{name}.topdown.png"
        if source.exists():
            shutil.copy(source, HERE / f"claim3_{name}.png")


def main() -> None:
    HERE.mkdir(exist_ok=True)
    planner_load()
    live_latency()
    low_drift_topdowns()
    efficiency()
    print("\n".join(sorted(str(p) for p in HERE.glob("*.png"))))


if __name__ == "__main__":
    main()
