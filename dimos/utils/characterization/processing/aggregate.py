# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Session-level aggregation across runs.

Groups runs by recipe name, computes per-group mean ± std for each
metric, rejects 2σ outlier *runs* (not samples), emits one summary
artifact per session.

Reads:
  - ``run.json`` for recipe name, exit_reason, BMS bookends, validation
  - ``metrics.json`` for the per-run rise/settle/overshoot/steady_state
    (must already exist — run ``python -m dimos.utils.characterization.scripts.analyze_run`` on each run first).

Writes:
  - ``session_summary.json`` — the aggregated structure
  - One CSV per session (``session_summary.csv``) for quick spreadsheet
    eyeballing.
"""

from __future__ import annotations

import csv
import json
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np


_NUMERIC_METRIC_KEYS = (
    "rise_10_90_s",
    "settle_s",
    "overshoot",
    "steady_state",
    "step_t",
    "target",
    "cmd_max",
    "cmd_min",
)


def aggregate_session(session_dir: Path, *, sigma: float = 2.0) -> dict[str, Any]:
    """Aggregate one session. Returns the summary dict."""
    session_dir = Path(session_dir).expanduser().resolve()
    run_dirs = sorted(p for p in session_dir.iterdir() if p.is_dir() and p.name[0].isdigit())

    by_recipe: dict[str, list[dict[str, Any]]] = defaultdict(list)

    for rd in run_dirs:
        run_json_path = rd / "run.json"
        metrics_json_path = rd / "metrics.json"
        if not run_json_path.exists():
            continue
        meta = json.loads(run_json_path.read_text())
        recipe_name = meta["recipe"]["name"]
        exit_reason = meta.get("exit_reason")
        metrics: dict[str, Any] = {}
        if metrics_json_path.exists():
            metrics = json.loads(metrics_json_path.read_text())
        validation_path = rd / "validation.json"
        passed_validation = None
        if validation_path.exists():
            passed_validation = json.loads(validation_path.read_text()).get("passed")

        by_recipe[recipe_name].append({
            "run_id": meta["run_id"],
            "exit_reason": exit_reason,
            "passed_validation": passed_validation,
            "bms_start_soc": (meta.get("bms_start") or {}).get("soc"),
            "bms_end_soc": (meta.get("bms_end") or {}).get("soc"),
            "metrics": metrics,
            "noise_floor": meta.get("noise_floor"),
        })

    groups: list[dict[str, Any]] = []
    for recipe_name, runs in by_recipe.items():
        groups.append(_aggregate_group(recipe_name, runs, sigma=sigma))

    summary: dict[str, Any] = {
        "session_dir": str(session_dir),
        "n_runs_total": sum(len(v) for v in by_recipe.values()),
        "n_recipes": len(by_recipe),
        "outlier_sigma": sigma,
        "groups": groups,
    }

    (session_dir / "session_summary.json").write_text(
        json.dumps(summary, indent=2, default=str) + "\n"
    )
    _write_csv(session_dir / "session_summary.csv", summary)
    return summary


def _aggregate_group(
    recipe_name: str, runs: list[dict[str, Any]], *, sigma: float
) -> dict[str, Any]:
    """Compute per-metric mean ± std, reject 2σ outlier runs, retry."""
    n_total = len(runs)
    runs_with_metrics = [r for r in runs if r["metrics"]]
    n_with_metrics = len(runs_with_metrics)

    metric_arrays: dict[str, np.ndarray] = {}
    for key in _NUMERIC_METRIC_KEYS:
        vals = [r["metrics"].get(key) for r in runs_with_metrics]
        vals = [v for v in vals if isinstance(v, (int, float)) and not isinstance(v, bool)]
        metric_arrays[key] = np.asarray(vals, dtype=float) if vals else np.array([])

    # Outlier rejection: for steady_state (the most stable metric), drop runs
    # whose value is > sigma σ from the mean. Apply to all metrics consistently
    # using the same kept-runs set.
    kept_run_ids: list[str] = [r["run_id"] for r in runs_with_metrics]
    rejected_run_ids: list[str] = []
    if metric_arrays["steady_state"].size >= 4:
        ss = metric_arrays["steady_state"]
        mean, std = float(np.mean(ss)), float(np.std(ss, ddof=1))
        if std > 0:
            keep_mask = np.abs(ss - mean) <= sigma * std
            for r, ok in zip(runs_with_metrics, keep_mask, strict=False):
                if not ok:
                    rejected_run_ids.append(r["run_id"])
            kept_run_ids = [r["run_id"] for r, ok in zip(runs_with_metrics, keep_mask, strict=False) if ok]
            for key in _NUMERIC_METRIC_KEYS:
                if metric_arrays[key].size == ss.size:
                    metric_arrays[key] = metric_arrays[key][keep_mask]

    metric_stats: dict[str, dict[str, float | None]] = {}
    for key, arr in metric_arrays.items():
        if arr.size == 0:
            metric_stats[key] = {"mean": None, "std": None, "n": 0,
                                 "min": None, "max": None}
        else:
            metric_stats[key] = {
                "mean": float(np.mean(arr)),
                "std": float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0,
                "n": int(arr.size),
                "min": float(np.min(arr)),
                "max": float(np.max(arr)),
            }

    # Aggregate noise floor across kept runs (median across runs).
    noise_floor_agg = _aggregate_noise_floor([
        r for r in runs_with_metrics if r["run_id"] in kept_run_ids
    ])

    bms_start = [r["bms_start_soc"] for r in runs if isinstance(r["bms_start_soc"], (int, float))]
    bms_end = [r["bms_end_soc"] for r in runs if isinstance(r["bms_end_soc"], (int, float))]

    return {
        "recipe": recipe_name,
        "n_runs_total": n_total,
        "n_runs_with_metrics": n_with_metrics,
        "n_runs_kept": len(kept_run_ids),
        "n_runs_rejected_2sigma": len(rejected_run_ids),
        "rejected_run_ids": rejected_run_ids,
        "metrics": metric_stats,
        "noise_floor": noise_floor_agg,
        "bms_soc_range": {
            "start_min": min(bms_start) if bms_start else None,
            "start_max": max(bms_start) if bms_start else None,
            "end_min": min(bms_end) if bms_end else None,
            "end_max": max(bms_end) if bms_end else None,
        },
    }


def _aggregate_noise_floor(runs: list[dict[str, Any]]) -> dict[str, dict[str, float | None]]:
    """Median σ per channel across runs."""
    out: dict[str, dict[str, float | None]] = {}
    for ch in ("vx", "vy", "wz", "yaw"):
        stds = [
            (r["noise_floor"] or {}).get(ch, {}).get("std") for r in runs
        ]
        stds = [s for s in stds if isinstance(s, (int, float))]
        out[ch] = {
            "median_std": float(np.median(stds)) if stds else None,
            "n_runs": len(stds),
        }
    return out


def _write_csv(path: Path, summary: dict[str, Any]) -> None:
    """One row per recipe group, columns for each metric mean/std."""
    cols = ["recipe", "n_runs_kept", "n_runs_rejected_2sigma"]
    for k in _NUMERIC_METRIC_KEYS:
        cols.extend([f"{k}_mean", f"{k}_std"])
    cols.extend(["soc_start_min", "soc_end_min"])

    with path.open("w") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for g in summary["groups"]:
            row = [g["recipe"], g["n_runs_kept"], g["n_runs_rejected_2sigma"]]
            for k in _NUMERIC_METRIC_KEYS:
                m = g["metrics"][k]
                row.extend([m["mean"], m["std"]])
            row.extend([
                g["bms_soc_range"]["start_min"], g["bms_soc_range"]["end_min"],
            ])
            w.writerow(row)


__all__ = ["aggregate_session"]
