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

"""Chart how long each module's scan thread was blocked over each live replay, scaled per recording: responsiveness, not total compute.

usage: plot_busy.py [<dataset> ...]   reads the live runs' latency -> $EVAL_LOOP_OUT/busy.svg
"""

import json
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from paths import OUT
from recordings import BY_NAME, DATASETS
from run import METHODS, dropped, latency_rows

COLORS = {"gsc_live": "#58a6ff", "ivan_live": "#f0883e"}
BACKGROUND, TEXT, MUTED = "#0d1117", "#e6edf3", "#8b949e"


def busy_s(rows: np.ndarray) -> float:
    """Seconds the one scan thread spent working: from a scan's arrival (or the previous scan's
    finish, if later) to its own finish, summed."""
    order = np.argsort(rows[:, 0] + rows[:, 1] / 1e3)
    arrival = rows[order, 0]
    done = arrival + rows[order, 1] / 1e3
    previous_done = np.concatenate([[-np.inf], done[:-1]])
    return float(np.sum(done - np.maximum(arrival, previous_done)))


def main() -> None:
    datasets = [BY_NAME[name] for name in sys.argv[1:]] or DATASETS
    rows = []
    for dataset in datasets:
        summary = OUT / f"{dataset.name}.summary.json"
        if not summary.exists() or not (OUT / f"{dataset.name}.gsc_live.json").exists():
            continue
        row = json.loads(summary.read_text())
        if not all(key in row for key, _ in METHODS):
            continue
        gsc = json.loads((OUT / f"{dataset.name}.gsc_live.json").read_text())
        scans = gsc["scans_processed"] + gsc["scans_dropped"]
        modules = {
            key: (busy_s(latency_rows(dataset.name, key)), dropped(dataset.name, key, row), scans)
            for key, _ in METHODS
        }
        rows.append((dataset.name, row["duration_s"], modules))
    rows.sort(key=lambda r: r[1])

    plt.style.use("dark_background")
    plt.rcParams["svg.fonttype"] = "none"  # text as text, not glyph paths: keeps the svg small
    fig, ax = plt.subplots(facecolor=BACKGROUND, figsize=(12, 0.62 * len(rows) + 1.6))
    height = 0.36
    for index, (_name, duration, modules) in enumerate(rows):
        longest = (
            max(seconds for seconds, _, _ in modules.values()) / 0.6
        )  # leave room for the labels
        for offset, (key, label) in zip((-height / 2, height / 2), METHODS, strict=False):
            seconds, lost, scans = modules[key]
            ax.barh(
                index + offset,
                seconds / longest,
                height,
                color=COLORS[key],
                label=label if index == 0 else None,
            )
            share = (
                " (<1%)" if 0 < lost < 0.01 * scans else f" ({lost / scans:.0%})" if lost else ""
            )
            note = f" {seconds:,.0f} s, {lost:,} of {scans:,} scans dropped{share}"
            ax.text(seconds / longest, index + offset, note, va="center", fontsize=8, color=TEXT)
        ax.text(
            1.0,
            index,
            f"{duration / 60:.0f} min",
            va="center",
            ha="right",
            fontsize=8,
            color=MUTED,
        )
    ax.text(
        1.0,
        len(rows) - 0.4,
        "recording\n(= replay time)",
        va="bottom",
        ha="right",
        fontsize=8,
        color=MUTED,
    )
    ax.set_yticks(range(len(rows)), [name for name, _, _ in rows])
    ax.set_facecolor(BACKGROUND)
    for spine in ax.spines.values():
        spine.set_color("#30363d")
    ax.set_xlim(0, 1.0)
    ax.set_ylim(-0.6, len(rows) + 0.4)
    ax.set_xticks([])
    ax.set_xlabel(
        "seconds the scan thread (the one that takes each scan) was busy; scans that arrive meanwhile wait or are dropped\n"
        "gsc's background solve runs on another thread and isn't counted; bars scaled per recording, slower = full width"
    )
    ax.set_title(
        "Time each module's scan thread was blocked on a real-time replay (not total compute)",
        loc="left",
    )
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.06), ncol=2, fontsize=8, frameon=False)
    ax.grid(axis="x", which="major", alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUT / "busy.svg", metadata={"Date": None}, facecolor=BACKGROUND)
    print(OUT / "busy.svg")


if __name__ == "__main__":
    main()
