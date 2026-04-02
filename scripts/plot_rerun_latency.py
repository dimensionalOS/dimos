#!/usr/bin/env python3
"""Generate latency graphs from Rerun bridge CSV logs.

Usage:
    python scripts/plot_rerun_latency.py data/rerun_logs/<timestamp>_rerun_latency.csv
"""

import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ALL_METRICS = ["queue_ms", "convert_ms", "rr_log_ms", "cb_total_ms"]
METRIC_LABELS = {
    "queue_ms": "Queue (msg created → cb enter)",
    "convert_ms": "Convert (to_rerun)",
    "rr_log_ms": "rr.log()",
    "cb_total_ms": "Total (msg created → rr.log done)",
}


def load_csv(path: str) -> list[dict]:
    rows = []
    with open(path) as f:
        lines = [l for l in f if not l.startswith("#")]
    reader = csv.DictReader(lines)
    for row in reader:
        row["wall_time"] = float(row["wall_time"])
        for m in ALL_METRICS:
            row[m] = float(row[m])
        try:
            row["n_points"] = int(row["n_points"]) if row.get("n_points", "").strip() else None
        except ValueError:
            row["n_points"] = None
        rows.append(row)
    return rows


def group_by_entity(rows: list[dict]) -> dict[str, list[dict]]:
    groups: dict[str, list[dict]] = defaultdict(list)
    for r in rows:
        groups[r["entity"]].append(r)
    return dict(groups)


def detect_replay(rows: list[dict]) -> bool:
    """Detect replay mode: queue_ms values are absurdly large (stale msg.ts from old recording)."""
    sample = [r["queue_ms"] for r in rows[:100] if r["queue_ms"] > 0]
    if not sample:
        return False
    return np.median(sample) > 1_000_000  # > 1000 seconds = replay


def save(fig: plt.Figure, path: Path) -> None:
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  {path}")


def plot_metrics_vs_time(groups: dict, t0: float, metrics: list[str], out: Path) -> None:
    n = len(metrics)
    fig, axes = plt.subplots(n, 1, figsize=(16, 3.5 * n), sharex=True)
    if n == 1:
        axes = [axes]
    for ax, metric in zip(axes, metrics):
        for entity, rows in sorted(groups.items()):
            t = [r["wall_time"] - t0 for r in rows]
            v = [r[metric] for r in rows]
            ax.plot(t, v, label=entity.split("/")[-1], alpha=0.7, linewidth=0.5)
        ax.set_ylabel(METRIC_LABELS[metric])
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left", fontsize=7)
    axes[0].set_title("Latency Metrics Over Time")
    axes[-1].set_xlabel("Time (s)")
    fig.tight_layout()
    save(fig, out)


def plot_metrics_vs_points(pc2_groups: dict, metrics: list[str], out: Path) -> None:
    n = len(metrics)
    fig, axes = plt.subplots(n, 1, figsize=(14, 3.5 * n), sharex=True)
    if n == 1:
        axes = [axes]
    for ax, metric in zip(axes, metrics):
        for entity, rows in sorted(pc2_groups.items()):
            pts = [(r["n_points"], r[metric]) for r in rows if r["n_points"] is not None]
            if not pts:
                continue
            x, y = zip(*pts)
            ax.scatter(x, y, label=entity.split("/")[-1], alpha=0.4, s=10)
        ax.set_ylabel(METRIC_LABELS[metric])
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left", fontsize=7)
    axes[0].set_title("Latency Metrics vs Point Count")
    axes[-1].set_xlabel("Point count")
    fig.tight_layout()
    save(fig, out)


def plot_points_vs_time(pc2_groups: dict, t0: float, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(14, 5))
    for entity, rows in sorted(pc2_groups.items()):
        pts = [(r["wall_time"] - t0, r["n_points"]) for r in rows if r["n_points"] is not None]
        if not pts:
            continue
        t, n = zip(*pts)
        ax.plot(t, n, label=entity.split("/")[-1], alpha=0.7, linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Point count")
    ax.set_title("Voxel / Point Count Over Time")
    ax.legend()
    ax.grid(True, alpha=0.3)
    save(fig, out)


def plot_boxplot_comparison(csv_paths: list[Path], out: Path) -> None:
    """Boxplot comparing all entities across all CSVs for convert_ms and rr_log_ms."""
    # Load all CSVs, use only convert_ms and rr_log_ms (safe for both replay and hardware)
    metrics = ["convert_ms", "rr_log_ms"]
    all_data: dict[str, dict[str, dict[str, list[float]]]] = {}
    # all_data[metric][entity][csv_label] = [values]

    labels = []
    for csv_path in csv_paths:
        label = csv_path.stem.replace("_rerun_latency", "")
        labels.append(label)
        rows = load_csv(str(csv_path))
        groups = group_by_entity(rows)
        for metric in metrics:
            if metric not in all_data:
                all_data[metric] = {}
            for entity, entity_rows in groups.items():
                short = entity.split("/")[-1]
                if short not in all_data[metric]:
                    all_data[metric][short] = {}
                all_data[metric][short][label] = [r[metric] for r in entity_rows]

    entities = sorted({e for m in all_data.values() for e in m})
    n_csvs = len(labels)
    colors = plt.cm.tab10(np.linspace(0, 1, n_csvs))

    fig, axes = plt.subplots(len(metrics), 1, figsize=(max(16, len(entities) * n_csvs * 0.6), 5 * len(metrics)))
    if len(metrics) == 1:
        axes = [axes]

    for ax, metric in zip(axes, metrics):
        positions = []
        box_data = []
        tick_positions = []
        tick_labels = []
        color_list = []

        for i, entity in enumerate(entities):
            base = i * (n_csvs + 1)
            tick_positions.append(base + (n_csvs - 1) / 2)
            tick_labels.append(entity)
            for j, label in enumerate(labels):
                vals = all_data.get(metric, {}).get(entity, {}).get(label, [])
                if vals:
                    positions.append(base + j)
                    box_data.append(vals)
                    color_list.append(colors[j])

        bp = ax.boxplot(box_data, positions=positions, widths=0.7, showfliers=False, patch_artist=True)
        for patch, color in zip(bp["boxes"], color_list):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)

        ax.set_xticks(tick_positions)
        ax.set_xticklabels(tick_labels, rotation=30, ha="right")
        ax.set_ylabel(f"{METRIC_LABELS[metric]} (ms)")
        ax.grid(True, alpha=0.3, axis="y")

        # Add mean markers
        for pos, vals in zip(positions, box_data):
            mean_val = np.mean(vals)
            ax.plot(pos, mean_val, "D", color="black", markersize=4, zorder=5)

    # Legend
    legend_patches = [plt.Rectangle((0, 0), 1, 1, fc=colors[j], alpha=0.7) for j in range(n_csvs)]
    axes[0].legend(legend_patches, labels, loc="upper right", fontsize=7, title="Run")
    axes[0].set_title("Latency Distribution by Entity Across Runs (diamond = mean)")
    fig.tight_layout()
    save(fig, out)


def main() -> None:
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <csv_path> [csv2 csv3 ...]")
        print(f"  Single CSV: generates per-run graphs")
        print(f"  Multiple CSVs: generates comparison boxplot")
        sys.exit(1)

    csv_paths = [Path(p) for p in sys.argv[1:]]

    if len(csv_paths) > 1:
        out_dir = csv_paths[0].parent.parent / "rerun_graphs" / "comparison"
        out_dir.mkdir(parents=True, exist_ok=True)
        print("Generating comparison boxplot:")
        plot_boxplot_comparison(csv_paths, out_dir / "boxplot_comparison.png")
        print("Done.")
        return

    csv_path = csv_paths[0]
    rows = load_csv(str(csv_path))
    if not rows:
        print("No data found.")
        sys.exit(1)

    out_dir = csv_path.parent.parent / "rerun_graphs" / csv_path.stem
    out_dir.mkdir(parents=True, exist_ok=True)

    groups = group_by_entity(rows)
    t0 = rows[0]["wall_time"]

    is_replay = detect_replay(rows)
    if is_replay:
        print("  Replay detected — skipping queue_ms and cb_total_ms (stale msg.ts)")
        metrics = ["convert_ms", "rr_log_ms"]
    else:
        metrics = ALL_METRICS

    pc2_groups = {
        e: rs for e, rs in groups.items() if any(r["n_points"] is not None for r in rs)
    }

    print("Generating graphs:")

    plot_metrics_vs_time(groups, t0, metrics, out_dir / "1_latency_vs_time.png")

    if pc2_groups:
        plot_metrics_vs_points(pc2_groups, metrics, out_dir / "2_latency_vs_points.png")
        plot_points_vs_time(pc2_groups, t0, out_dir / "3_points_vs_time.png")

    print("Done.")


if __name__ == "__main__":
    main()
