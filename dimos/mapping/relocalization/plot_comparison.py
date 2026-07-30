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

"""Parse two `eval.py` run logs (--mode global vs --mode tracking) into a
comparison table + plots. Doesn't re-run anything itself -- see `compare.sh`
for the end-to-end pipeline (run both modes, then call this).

Usage:
    uv run python -m dimos.mapping.relocalization.plot_comparison \
        --global-log /tmp/reloc_comparison/global.log \
        --tracking-log /tmp/reloc_comparison/tracking.log \
        --out-dir /tmp/reloc_comparison
"""

from __future__ import annotations

import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
import typer  # noqa: E402

LINE_RE = re.compile(
    r"t=\s*(?P<ts>[\d.]+)\s+"
    r"n_pts=\s*(?P<n_pts>\d+)\s+"
    r"mode=(?P<mode>\S+)\s+"
    r"fitness=(?P<fitness>[\d.]+)\s+"
    r"wall=\s*(?P<wall_s>[\d.]+)s\s+"
    r"cpu=\s*(?P<cpu_s>[\d.]+)s\s+"
    r"trans_err=(?P<trans_err_m>[\d.]+)m\s+"
    r"yaw_err=\s*(?P<yaw_err_deg>[\-\d.]+)deg\s+"
    r"(?P<status>ACCEPT|reject)"
)

COLORS = {"search": "tab:red", "tracking": "tab:blue"}


def parse_log(path: Path) -> pd.DataFrame:
    """Extract one row per attempt line; silently skips PGO/log-closure noise."""
    rows = []
    for line in path.read_text().splitlines():
        m = LINE_RE.search(line)
        if not m:
            continue
        d = m.groupdict()
        rows.append(
            {
                "ts": float(d["ts"]),
                "n_pts": int(d["n_pts"]),
                "mode": d["mode"],
                "fitness": float(d["fitness"]),
                "wall_s": float(d["wall_s"]),
                "cpu_s": float(d["cpu_s"]),
                "trans_err_m": float(d["trans_err_m"]),
                "yaw_err_deg": float(d["yaw_err_deg"]),
                "accepted": d["status"] == "ACCEPT",
            }
        )
    if not rows:
        raise SystemExit(f"No attempt lines parsed from {path} -- wrong file, or eval.py's log format changed?")
    df = pd.DataFrame(rows)
    df["idx"] = range(len(df))
    return df


def _med_p90(df: pd.DataFrame, col: str, *, accepted_only: bool) -> tuple[float, float]:
    vals = df.loc[df["accepted"], col] if accepted_only else df[col]
    return float(vals.median()), float(np.percentile(vals, 90))


def print_table(global_df: pd.DataFrame, tracking_df: pd.DataFrame) -> None:
    groups = {
        "global (baseline)": global_df,
        "tracking - overall": tracking_df,
        "tracking - search only": tracking_df[tracking_df["mode"] == "search"],
        "tracking - tracking only": tracking_df[tracking_df["mode"] == "tracking"],
    }
    print()
    header = (
        f"{'group':<28} {'n':>4} {'wall med/p90 (s)':>20} {'cpu med/p90 (s)':>20} "
        f"{'trans_err med/p90 (m)':>24} {'yaw_err med/p90 (deg)':>24}"
    )
    print(header)
    print("-" * len(header))
    for name, g in groups.items():
        if g.empty:
            continue
        wall_med, wall_p90 = _med_p90(g, "wall_s", accepted_only=False)
        cpu_med, cpu_p90 = _med_p90(g, "cpu_s", accepted_only=False)
        te_med, te_p90 = _med_p90(g, "trans_err_m", accepted_only=True)
        ye_med, ye_p90 = _med_p90(g, "yaw_err_deg", accepted_only=True)
        print(
            f"{name:<28} {len(g):>4} "
            f"{wall_med:>9.2f}/{wall_p90:<9.2f} "
            f"{cpu_med:>9.2f}/{cpu_p90:<9.2f} "
            f"{te_med:>10.3f}/{te_p90:<12.3f} "
            f"{ye_med:>10.2f}/{ye_p90:<12.2f}"
        )

    print()
    total_global = global_df["wall_s"].sum()
    total_tracking = tracking_df["wall_s"].sum()
    reduction = 100 * (1 - total_tracking / total_global)
    print(f"total wall-clock, {len(global_df)} attempts each:")
    print(f"  global:   {total_global:.1f}s")
    print(f"  tracking: {total_tracking:.1f}s")
    print(f"  -> {reduction:.0f}% reduction in total wall-clock time")


def make_plots(global_df: pd.DataFrame, tracking_df: pd.DataFrame, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. Per-attempt wall-clock latency (log scale -- dynamic range is ~300x).
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(global_df["idx"], global_df["wall_s"], "o-", color="tab:gray", label="global (baseline)")
    ax.plot(tracking_df["idx"], tracking_df["wall_s"], "-", color="tab:blue", alpha=0.3)
    for mode, g in tracking_df.groupby("mode"):
        ax.scatter(g["idx"], g["wall_s"], color=COLORS.get(mode, "black"), label=f"tracking run: {mode}", zorder=3)
    ax.set_yscale("log")
    ax.set_xlabel("attempt #")
    ax.set_ylabel("wall-clock latency (s, log scale)")
    ax.set_title("Per-attempt latency: global search vs. mode-switched tracking")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "latency_per_attempt.png", dpi=150)
    plt.close(fig)

    # 2. Per-attempt translation error vs. ground truth.
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(global_df["idx"], global_df["trans_err_m"], "o-", color="tab:gray", label="global (baseline)")
    ax.plot(tracking_df["idx"], tracking_df["trans_err_m"], "-", color="tab:blue", alpha=0.3)
    for mode, g in tracking_df.groupby("mode"):
        ax.scatter(g["idx"], g["trans_err_m"], color=COLORS.get(mode, "black"), label=f"tracking run: {mode}", zorder=3)
    ax.set_xlabel("attempt #")
    ax.set_ylabel("translation error vs. ground truth (m)")
    ax.set_title("Per-attempt pose error: global search vs. mode-switched tracking")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "trans_err_per_attempt.png", dpi=150)
    plt.close(fig)

    # 3. Total wall-clock time, bar chart.
    fig, ax = plt.subplots(figsize=(5, 4.5))
    totals = [global_df["wall_s"].sum(), tracking_df["wall_s"].sum()]
    bars = ax.bar(["global\n(baseline)", "tracking\n(our method)"], totals, color=["tab:gray", "tab:blue"])
    ax.set_ylabel("total wall-clock time (s)")
    ax.set_title(f"Total time, {len(global_df)} attempts")
    for b, v in zip(bars, totals, strict=True):
        ax.text(b.get_x() + b.get_width() / 2, v, f"{v:.0f}s", ha="center", va="bottom")
    reduction = 100 * (1 - totals[1] / totals[0])
    ax.text(0.5, 0.9, f"-{reduction:.0f}%", transform=ax.transAxes, ha="center", fontsize=16, color="tab:blue")
    fig.tight_layout()
    fig.savefig(out_dir / "total_wall_time.png", dpi=150)
    plt.close(fig)

    # 4. Does self-reported fitness track real pose error?
    fig, ax = plt.subplots(figsize=(6, 4.5))
    ax.scatter(global_df["fitness"], global_df["trans_err_m"], color="tab:gray", label="global (baseline)")
    for mode, g in tracking_df.groupby("mode"):
        ax.scatter(g["fitness"], g["trans_err_m"], color=COLORS.get(mode, "black"), label=f"tracking run: {mode}")
    ax.set_xlabel("ICP fitness (self-reported)")
    ax.set_ylabel("translation error vs. ground truth (m)")
    ax.set_title("Does fitness track real pose error?")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "fitness_vs_trans_err.png", dpi=150)
    plt.close(fig)

    print(f"\nplots written to {out_dir}/")


def main(
    global_log: str = typer.Option(..., "--global-log", help="Log from a --mode global eval.py run"),
    tracking_log: str = typer.Option(..., "--tracking-log", help="Log from a --mode tracking eval.py run"),
    out_dir: str = typer.Option("/tmp/reloc_comparison", "--out-dir"),
) -> None:
    global_df = parse_log(Path(global_log))
    tracking_df = parse_log(Path(tracking_log))
    print_table(global_df, tracking_df)
    make_plots(global_df, tracking_df, Path(out_dir))


if __name__ == "__main__":
    typer.run(main)
