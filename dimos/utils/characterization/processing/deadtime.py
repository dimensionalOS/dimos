# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""E8-specific deadtime statistics.

For each run that looks like an E8 short-step recipe (cmd channel
non-zero amplitude, short duration), find:
    - cmd edge time (when commanded leaves the pre-roll zero band)
    - response onset (first time |measured| exceeds 3 × σ_noise of that channel)
    - deadtime = response_onset - cmd_edge

Aggregate across all matching runs in the session: histogram, mean,
median, p95, jitter (std). Emit a JSON with the full deadtime list +
summary stats. **Uses raw measured signal**, not cleaned — onset
timing is exactly the kind of thing the cleaning rule says to keep raw.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from dimos.utils.characterization.processing.validate import _load_measured_for_run


_DEFAULT_THRESHOLD_K = 3.0  # threshold = K × σ_noise


def deadtime_stats_session(
    session_dir: Path,
    *,
    threshold_k: float = _DEFAULT_THRESHOLD_K,
) -> dict[str, Any]:
    """Compute deadtime per E8-style run and aggregate."""
    from dimos.utils.characterization.scripts.analyze_run import (
        reconstruct_body_velocities,
    )

    session_dir = Path(session_dir).expanduser().resolve()
    run_dirs = sorted(p for p in session_dir.iterdir() if p.is_dir() and p.name[0].isdigit())

    per_run: list[dict[str, Any]] = []
    for rd in run_dirs:
        run_json = rd / "run.json"
        if not run_json.exists():
            continue
        meta = json.loads(run_json.read_text())
        recipe = meta["recipe"]
        if recipe.get("test_type") != "step":
            continue
        # Heuristic: short hold (<= 1.0 s) means it's an E8-style run.
        if recipe.get("duration_s", 99) > 1.0:
            continue

        cmd_t, cmd_v, channel, target = _load_cmd_channel(rd, meta)
        if cmd_t.size == 0:
            continue

        meas_ts, meas_x, meas_y, meas_yaw = _load_measured_for_run(rd, meta)
        if meas_ts.size < 3:
            per_run.append({
                "run_id": meta["run_id"], "recipe": recipe["name"],
                "deadtime_s": None, "reason": "no measured samples"
            })
            continue
        vx, vy, wz = reconstruct_body_velocities(meas_ts, meas_x, meas_y, meas_yaw)
        meas_v = {"vx": vx, "vy": vy, "wz": wz}[channel]

        sigma = (meta.get("noise_floor") or {}).get(channel, {}).get("std")
        if not isinstance(sigma, (int, float)) or sigma <= 0:
            per_run.append({
                "run_id": meta["run_id"], "recipe": recipe["name"],
                "deadtime_s": None,
                "reason": "missing noise_floor — run python -m dimos.utils.characterization.scripts.process_session validate first"
            })
            continue

        threshold = threshold_k * sigma
        edge_t = _cmd_edge_time(cmd_t, cmd_v)
        onset_t = _onset_time(meas_ts, meas_v, threshold, after=edge_t, sign=np.sign(target))

        if edge_t is None or onset_t is None:
            per_run.append({
                "run_id": meta["run_id"], "recipe": recipe["name"],
                "deadtime_s": None,
                "edge_t": edge_t, "onset_t": onset_t,
                "threshold": threshold,
                "reason": "edge or onset not detected",
            })
            continue

        per_run.append({
            "run_id": meta["run_id"],
            "recipe": recipe["name"],
            "channel": channel,
            "target": float(target),
            "sigma": float(sigma),
            "threshold": round(threshold, 5),
            "edge_t_wall": round(edge_t, 4),
            "onset_t_wall": round(onset_t, 4),
            "deadtime_s": round(onset_t - edge_t, 4),
        })

    deadtimes = np.array([r["deadtime_s"] for r in per_run if isinstance(r.get("deadtime_s"), (int, float))], dtype=float)
    summary: dict[str, Any]
    if deadtimes.size == 0:
        summary = {"n": 0, "reason": "no usable runs"}
    else:
        summary = {
            "n": int(deadtimes.size),
            "mean_s": round(float(np.mean(deadtimes)), 4),
            "median_s": round(float(np.median(deadtimes)), 4),
            "p95_s": round(float(np.percentile(deadtimes, 95)), 4),
            "p5_s": round(float(np.percentile(deadtimes, 5)), 4),
            "std_s": round(float(np.std(deadtimes, ddof=1)), 4) if deadtimes.size > 1 else 0.0,
            "min_s": round(float(np.min(deadtimes)), 4),
            "max_s": round(float(np.max(deadtimes)), 4),
        }

    out = {
        "session_dir": str(session_dir),
        "threshold_k": threshold_k,
        "summary": summary,
        "per_run": per_run,
    }
    (session_dir / "deadtime_stats.json").write_text(json.dumps(out, indent=2) + "\n")
    return out


# ---------------------------------------------------------------------------- internal

def _load_cmd_channel(run_dir: Path, meta: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, str, float]:
    """Load cmd timestamps + amplitudes for the dominant channel.

    Returns (t_wall, values, channel_name, target_amplitude).
    """
    rows = [json.loads(line) for line in (run_dir / "cmd_monotonic.jsonl").read_text().splitlines() if line.strip()]
    if not rows:
        return np.array([]), np.array([]), "vx", 0.0
    vx = np.array([r["vx"] for r in rows], dtype=float)
    vy = np.array([r["vy"] for r in rows], dtype=float)
    wz = np.array([r["wz"] for r in rows], dtype=float)
    amps = {"vx": np.max(np.abs(vx)), "vy": np.max(np.abs(vy)), "wz": np.max(np.abs(wz))}
    channel = max(amps, key=lambda k: amps[k])
    arr = {"vx": vx, "vy": vy, "wz": wz}[channel]
    target_amp = float(arr[np.argmax(np.abs(arr))]) if arr.size else 0.0
    t = np.array([r["tx_wall"] for r in rows], dtype=float)
    return t, arr, channel, target_amp


def _cmd_edge_time(t: np.ndarray, v: np.ndarray, *, eps: float = 1e-3) -> float | None:
    nonzero = np.flatnonzero(np.abs(v) > eps)
    if nonzero.size == 0:
        return None
    return float(t[nonzero[0]])


def _onset_time(
    t: np.ndarray, v: np.ndarray, threshold: float, *, after: float, sign: float
) -> float | None:
    """First sample after ``after`` whose value exceeds threshold in ``sign`` direction."""
    if v.size == 0:
        return None
    if sign >= 0:
        cond = v > threshold
    else:
        cond = v < -threshold
    cond = cond & (t > after)
    idx = np.argmax(cond)
    if not cond[idx]:
        return None
    return float(t[idx])


__all__ = ["deadtime_stats_session"]
