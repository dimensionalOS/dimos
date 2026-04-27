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

"""dtop-plot — Plot resource stats from a dtop JSONL log file.

Usage:
    dtop-plot <log.jsonl> [--metrics cpu_percent,pss] [--out plot.png]
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import pandas as pd

_COORDINATOR = "coordinator"

_METRIC_LABELS: dict[str, str] = {
    "cpu_percent": "CPU %",
    "pss": "PSS (MB)",
    "num_threads": "Threads",
    "num_children": "Children",
    "num_fds": "File Descriptors",
    "cpu_time_user": "User CPU Time (s)",
    "cpu_time_system": "Sys CPU Time (s)",
    "cpu_time_iowait": "IO Wait Time (s)",
    "io_read_bytes": "IO Read (MB)",
    "io_write_bytes": "IO Write (MB)",
}

_SCALE: dict[str, float] = {
    "pss": 1 / 1048576,
    "io_read_bytes": 1 / 1048576,
    "io_write_bytes": 1 / 1048576,
}


def _load(path: str) -> tuple[pd.DataFrame, dict[str, str]]:
    import pandas as pd

    raw = pd.read_json(path, lines=True)

    rows = []
    for _, msg in raw.iterrows():
        ts = msg["ts"]
        rows.append({"ts": ts, "role": _COORDINATOR, **msg[_COORDINATOR]})
        for w in msg.get("workers", []):
            wid = w.get("worker_id", 0)
            rows.append({"ts": ts, "role": f"worker_{wid}", **w})

    df = pd.DataFrame(rows)
    df["ts"] = pd.to_datetime(df["ts"], unit="s")

    labels: dict[str, str] = {_COORDINATOR: _COORDINATOR}
    for role, group in df.groupby("role"):
        role = str(role)
        if role == _COORDINATOR:
            continue
        mods = next((m for m in group.get("modules", []) if m), None)
        labels[role] = ", ".join(mods) if mods else role

    df["label"] = df["role"].map(labels)
    return df, labels


def _plot(
    df: pd.DataFrame, labels: dict[str, str], metrics: list[str], out: str, show: bool = False
) -> None:
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(len(metrics), 1, figsize=(12, 3 * len(metrics)), sharex=True)
    if len(metrics) == 1:
        axes = [axes]

    for ax, metric in zip(axes, metrics, strict=False):
        if metric not in df.columns:
            ax.set_visible(False)
            continue
        scale = _SCALE.get(metric, 1.0)
        for role, group in df.groupby("role"):
            ax.plot(group["ts"], group[metric] * scale, label=labels[str(role)])
        ax.set_ylabel(_METRIC_LABELS.get(metric, metric))
        ax.legend(fontsize=8, loc="center left", bbox_to_anchor=(1.01, 0.5), borderaxespad=0)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time")
    fig.tight_layout()

    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Saved to {out}")
    if show:
        plt.show()


def _default_out(log_path: str) -> str:
    base = log_path.removesuffix(".ignore.jsonl")
    return f"{base}.ignore.png"


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(
        prog="dtop-plot", description="Plot resource stats from a dtop JSONL log file."
    )
    parser.add_argument("log", metavar="LOG", help="Path to a dtop JSONL log file.")
    parser.add_argument(
        "--metrics",
        default="cpu_percent,pss,num_threads",
        help="Comma-separated list of metrics to plot (default: cpu_percent,pss,num_threads).",
    )
    parser.add_argument(
        "--out", metavar="PATH", help="Output image path (default: <log>.ignore.png)."
    )
    parser.add_argument(
        "--show", action="store_true", help="Open the plot interactively after saving."
    )
    args = parser.parse_args()

    out = args.out or _default_out(args.log)
    metrics = [m.strip() for m in args.metrics.split(",")]
    df, labels = _load(args.log)
    _plot(df, labels, metrics, out, args.show)


if __name__ == "__main__":
    main()
