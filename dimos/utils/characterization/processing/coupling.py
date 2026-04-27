# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""E7 cross-coupling aggregation.

For each E7 run (constant-hold of pure wz or pure vx):
    - active window = phase=="active" rows from cmd_monotonic.jsonl
    - leak = mean abs(measured value on the *other* channel) during active
    - leak as % of commanded primary channel
    - leak as multiples of σ_noise on that channel

Aggregated decision rule:
    leak% < 5%  → SISO is fine
    leak% 5-10% → flag, note in report
    leak% > 10% → MIMO concern, controller needs cross-axis terms
"""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np

from dimos.utils.characterization.processing.validate import _load_measured_for_run


def coupling_stats_session(session_dir: Path) -> dict[str, Any]:
    """Aggregate cross-coupling per E7 run, group by (test, amplitude, direction)."""
    from dimos.utils.characterization.scripts.analyze_run import (
        reconstruct_body_velocities,
    )

    session_dir = Path(session_dir).expanduser().resolve()
    run_dirs = sorted(p for p in session_dir.iterdir() if p.is_dir() and p.name[0].isdigit())

    per_run: list[dict[str, Any]] = []
    by_group: dict[str, list[dict[str, Any]]] = defaultdict(list)

    for rd in run_dirs:
        run_json = rd / "run.json"
        if not run_json.exists():
            continue
        meta = json.loads(run_json.read_text())
        recipe = meta["recipe"]
        if recipe.get("test_type") != "constant":
            continue
        # Heuristic: E7 recipe names start with "e7"
        if not recipe.get("name", "").lower().startswith("e7"):
            continue

        cmd = _load_cmd(rd)
        active = cmd["phase"] == "active"
        if not active.any():
            continue
        cmd_vx = cmd["vx"][active]
        cmd_vy = cmd["vy"][active]
        cmd_wz = cmd["wz"][active]
        t_active_lo = float(cmd["tx_wall"][active][0])
        t_active_hi = float(cmd["tx_wall"][active][-1])

        # Determine primary channel: the one with non-zero commanded amplitude.
        primary = "vx" if abs(cmd_vx).max() > abs(cmd_wz).max() else "wz"
        primary_amp = float(np.mean({"vx": cmd_vx, "wz": cmd_wz}[primary]))

        meas_ts, meas_x, meas_y, meas_yaw = _load_measured_for_run(rd, meta)
        if meas_ts.size < 3:
            continue
        vx_meas, vy_meas, wz_meas = reconstruct_body_velocities(meas_ts, meas_x, meas_y, meas_yaw)
        in_active = (meas_ts >= t_active_lo) & (meas_ts <= t_active_hi)
        if int(in_active.sum()) < 3:
            continue

        meas_primary = {"vx": vx_meas, "wz": wz_meas}[primary][in_active]
        cross_channels = [c for c in ("vx", "vy", "wz") if c != primary]
        sigmas = (meta.get("noise_floor") or {})

        leaks: dict[str, dict[str, Any]] = {}
        for ch in cross_channels:
            arr = {"vx": vx_meas, "vy": vy_meas, "wz": wz_meas}[ch][in_active]
            mean_abs_leak = float(np.mean(np.abs(arr)))
            peak_abs_leak = float(np.max(np.abs(arr)))
            sigma = (sigmas.get(ch) or {}).get("std")
            leaks[ch] = {
                "mean_abs": round(mean_abs_leak, 4),
                "peak_abs": round(peak_abs_leak, 4),
                "leak_pct_of_primary": (
                    round(100.0 * mean_abs_leak / abs(primary_amp), 2)
                    if abs(primary_amp) > 1e-6 else None
                ),
                "sigma_multiple": (
                    round(mean_abs_leak / sigma, 2)
                    if isinstance(sigma, (int, float)) and sigma > 0 else None
                ),
            }

        primary_steady = float(np.mean(meas_primary)) if meas_primary.size else 0.0
        run_summary = {
            "run_id": meta["run_id"],
            "recipe": recipe["name"],
            "primary_channel": primary,
            "primary_cmd": round(primary_amp, 3),
            "primary_meas_steady": round(primary_steady, 4),
            "leaks": leaks,
        }
        per_run.append(run_summary)
        by_group[recipe["name"]].append(run_summary)

    groups: list[dict[str, Any]] = []
    for name, runs in by_group.items():
        groups.append(_aggregate_group(name, runs))

    overall_decision = _overall_decision(groups)

    out = {
        "session_dir": str(session_dir),
        "decision_rule": "siso<5% / flag5-10% / mimo>10%",
        "overall_decision": overall_decision,
        "n_runs": len(per_run),
        "groups": groups,
        "per_run": per_run,
    }
    (session_dir / "coupling_stats.json").write_text(json.dumps(out, indent=2) + "\n")
    return out


# ---------------------------------------------------------------------------- internal

def _load_cmd(run_dir: Path) -> dict[str, np.ndarray]:
    rows = [json.loads(line) for line in (run_dir / "cmd_monotonic.jsonl").read_text().splitlines() if line.strip()]
    return {
        "tx_wall": np.array([r["tx_wall"] for r in rows], dtype=float),
        "phase": np.array([r["phase"] for r in rows]),
        "vx": np.array([r["vx"] for r in rows], dtype=float),
        "vy": np.array([r["vy"] for r in rows], dtype=float),
        "wz": np.array([r["wz"] for r in rows], dtype=float),
    }


def _aggregate_group(name: str, runs: list[dict[str, Any]]) -> dict[str, Any]:
    if not runs:
        return {"recipe": name, "n_runs": 0}

    primary = runs[0]["primary_channel"]
    leak_channels = list(runs[0]["leaks"].keys())
    leak_pcts: dict[str, list[float]] = {ch: [] for ch in leak_channels}
    sigma_mults: dict[str, list[float]] = {ch: [] for ch in leak_channels}
    primary_meas: list[float] = []

    for r in runs:
        for ch in leak_channels:
            v = r["leaks"][ch].get("leak_pct_of_primary")
            if isinstance(v, (int, float)):
                leak_pcts[ch].append(float(v))
            s = r["leaks"][ch].get("sigma_multiple")
            if isinstance(s, (int, float)):
                sigma_mults[ch].append(float(s))
        primary_meas.append(r["primary_meas_steady"])

    return {
        "recipe": name,
        "n_runs": len(runs),
        "primary_channel": primary,
        "primary_meas_steady_mean": round(float(np.mean(primary_meas)), 4) if primary_meas else None,
        "primary_meas_steady_std": (
            round(float(np.std(primary_meas, ddof=1)), 4) if len(primary_meas) > 1 else 0.0
        ),
        "leak_pct_mean": {
            ch: round(float(np.mean(v)), 2) if v else None for ch, v in leak_pcts.items()
        },
        "leak_pct_std": {
            ch: round(float(np.std(v, ddof=1)), 2) if len(v) > 1 else 0.0 for ch, v in leak_pcts.items()
        },
        "sigma_multiple_mean": {
            ch: round(float(np.mean(v)), 2) if v else None for ch, v in sigma_mults.items()
        },
        "decision": _group_decision(leak_pcts),
    }


def _group_decision(leak_pcts: dict[str, list[float]]) -> str:
    """Worst-case across cross channels."""
    worst = 0.0
    for v in leak_pcts.values():
        if v:
            worst = max(worst, float(np.mean(v)))
    if worst < 5.0:
        return "SISO"
    if worst <= 10.0:
        return "FLAG"
    return "MIMO"


def _overall_decision(groups: list[dict[str, Any]]) -> str:
    decisions = [g.get("decision") for g in groups if g.get("decision")]
    if not decisions:
        return "no data"
    if "MIMO" in decisions:
        return "MIMO"
    if "FLAG" in decisions:
        return "FLAG"
    return "SISO"


__all__ = ["coupling_stats_session"]
