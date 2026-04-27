# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Per-run noise floor estimation from the pre-roll window.

The pre-roll is recorded with all-zero commanded velocities, so the
measured signal during pre-roll *is* the noise floor (drift, IMU bias,
odometry quantization). Computing σ here once per run gives every
downstream threshold a principled basis instead of a magic constant.

Computed values are appended to ``run.json`` under a new top-level
``noise_floor`` key. Existing fields are not modified. Safe to rerun.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import numpy as np

from dimos.utils.characterization.processing.validate import _load_measured_for_run


def compute_noise_floor(run_dir: Path) -> dict[str, dict[str, float]]:
    """Compute σ, mean, span on each derived measured channel during pre-roll.

    Channels:
      vx, vy: body-frame translational, derived as gradient(x or y) rotated by -yaw
      wz:     yaw rate, gradient(yaw) directly
      x, y:   raw world-frame position
      yaw:    raw yaw

    Returns the dict and writes it into ``run.json`` (additive — never
    overwrites existing fields). Pre-roll is identified from
    ``cmd_monotonic.jsonl``'s ``phase == "pre_roll"`` window.
    """
    from dimos.utils.characterization.scripts.analyze_run import (
        reconstruct_body_velocities,
    )

    run_dir = Path(run_dir).expanduser().resolve()
    run_json_path = run_dir / "run.json"
    meta = json.loads(run_json_path.read_text())

    pre_window = _pre_roll_wall_window(run_dir, meta)
    meas_ts, meas_x, meas_y, meas_yaw = _load_measured_for_run(run_dir, meta)

    out: dict[str, dict[str, float]] = {}
    if meas_ts.size < 3 or pre_window is None:
        out = {"_unavailable": {"reason": "insufficient measured samples"}}  # type: ignore[dict-item]
    else:
        # Slice to pre-roll window only.
        ts_lo, ts_hi = pre_window
        mask = (meas_ts >= ts_lo) & (meas_ts <= ts_hi)
        if int(mask.sum()) < 3:
            out = {"_unavailable": {"reason": "fewer than 3 pre-roll samples", "n": int(mask.sum())}}  # type: ignore[dict-item]
        else:
            t = meas_ts[mask]
            x = meas_x[mask]
            y = meas_y[mask]
            yaw = meas_yaw[mask]
            # Reconstruct body-frame velocities on the same window (small
            # window, no SavGol filtering — too few samples).
            vx, vy, wz = reconstruct_body_velocities(t, x, y, yaw, window=3, order=1)
            for name, arr in [
                ("vx", vx), ("vy", vy), ("wz", wz),
                ("x", x), ("y", y), ("yaw", yaw),
            ]:
                out[name] = {
                    "std": float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0,
                    "mean": float(np.mean(arr)),
                    "span": float(np.ptp(arr)),
                    "n": int(arr.size),
                }

    # Append (don't overwrite existing fields).
    meta["noise_floor"] = out
    _atomic_write_json(run_json_path, meta)
    return out


def compute_session(session_dir: Path) -> dict[str, Any]:
    """Run noise-floor computation across every run in a session."""
    session_dir = Path(session_dir).expanduser().resolve()
    rows = []
    for rd in sorted(p for p in session_dir.iterdir() if p.is_dir() and p.name[0].isdigit()):
        try:
            r = compute_noise_floor(rd)
            vx_std = (r.get("vx") or {}).get("std")
            wz_std = (r.get("wz") or {}).get("std")
            rows.append({"run_id": rd.name, "vx_std": vx_std, "wz_std": wz_std})
        except Exception as e:  # pragma: no cover
            rows.append({"run_id": rd.name, "error": str(e)})
    return {"session_dir": str(session_dir), "runs": rows}


# ---------------------------------------------------------------------------- internal

def _pre_roll_wall_window(run_dir: Path, meta: dict[str, Any]) -> tuple[float, float] | None:
    """Wall-clock [start, end] of the pre-roll window.

    The cmd_monotonic.jsonl rows have both tx_mono and tx_wall, so we
    can directly read pre-roll wall-clock bounds without translating.
    """
    cmd_path = run_dir / "cmd_monotonic.jsonl"
    if not cmd_path.exists():
        return None
    pre_walls: list[float] = []
    with cmd_path.open() as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            if d.get("phase") == "pre_roll":
                pre_walls.append(float(d["tx_wall"]))
    if not pre_walls:
        return None
    return (min(pre_walls), max(pre_walls))


def _atomic_write_json(path: Path, data: dict[str, Any]) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2, default=str) + "\n")
    os.replace(tmp, path)


__all__ = ["compute_noise_floor", "compute_session"]
