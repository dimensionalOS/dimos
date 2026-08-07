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

"""Scoring: gold closeness, then consistency, then speed — by magnitude.

world = gate * (100*gold + 10*consistency + 1*speed), max 111. The order
gaps make priorities lexicographic in effect: no speed buys back deviation,
nothing buys back a collision (gate 0 = non-vetoed interpenetration).
Battery score = mean over worlds; report worst world and DQ count too.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from .sim import Verdict

SPEED_BUDGET_MS = 20.0  # per-plan deadline; full credit at or under it, hyperbolic above
GOLD_SCALE = 0.5  # Frechet/gold_len at which gold closeness hits zero
CONSIST_SCALE = 0.25  # worst(flicker, consist) at which consistency hits zero
STALL_ARC = 0.3  # published path shorter than this = the robot goes nowhere


def frechet(a: np.ndarray, b: np.ndarray) -> float:
    """Discrete Frechet distance between two Nx2 polylines."""
    n, m = len(a), len(b)
    d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    ca = np.full((n, m), np.inf)
    ca[0, 0] = d[0, 0]
    for i in range(n):
        for j in range(m):
            if i == 0 and j == 0:
                continue
            best = min(
                ca[i - 1, j] if i > 0 else np.inf,
                ca[i, j - 1] if j > 0 else np.inf,
                ca[i - 1, j - 1] if i > 0 and j > 0 else np.inf,
            )
            ca[i, j] = max(best, d[i, j])
    return float(ca[-1, -1])


def _resample(xy: np.ndarray, step: float = 0.1) -> np.ndarray:
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    arcs = np.concatenate([[0.0], np.cumsum(seg)])
    if arcs[-1] < 1e-9:
        return xy[:1]
    marks = np.arange(0.0, float(arcs[-1]) + step / 2.0, step)
    return np.column_stack([np.interp(marks, arcs, xy[:, 0]), np.interp(marks, arcs, xy[:, 1])])


def score_world(v: Verdict) -> dict[str, Any]:
    """Score one verdict; see module docstring for the shape."""
    out: dict[str, Any] = {"name": v.scenario.name, "dq": False}

    # Safety gate: a non-vetoed path that interpenetrates truth is a
    # collision the robot would perform. Nothing else matters.
    if v.min_truth < -1e-6 and not v.veto:
        out.update(dq=True, gold=0.0, consistency=0.0, speed=0.0, total=0.0)
        return out
    if v.timed_out:
        out.update(dq="timeout", gold=0.0, consistency=0.0, speed=0.0, total=0.0)
        return out

    path = np.array([[p.position.x, p.position.y] for p in v.final.poses])
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1) if len(path) > 1 else np.array([0.0])
    path_arc = float(np.sum(seg))

    if v.gold is None:
        # Truth says sealed: refusing IS the gold behavior.
        refused = v.veto or path_arc < STALL_ARC
        g = 1.0 if refused else 0.0
    elif v.veto or path_arc < STALL_ARC:
        g = 0.0  # gold proves a route; going nowhere is maximal deviation
    else:
        gold_xy = _resample(v.gold[:, :2])
        gold_len = max(float(np.sum(np.linalg.norm(np.diff(v.gold[:, :2], axis=0), axis=1))), 1e-6)
        d = frechet(_resample(path), gold_xy) / gold_len
        g = max(0.0, 1.0 - d / GOLD_SCALE)

    worst = max(v.flicker, v.consist)
    c = max(0.0, 1.0 - worst / CONSIST_SCALE)
    s = min(1.0, SPEED_BUDGET_MS / max(v.avoid_ms, 1e-6))

    out.update(
        gold=round(g, 4),
        consistency=round(c, 4),
        speed=round(s, 4),
        total=round(100.0 * g + 10.0 * c + s, 2),
    )
    return out


def summarize(worlds: list[dict[str, Any]]) -> dict[str, Any]:
    totals = [w["total"] for w in worlds]
    worst = min(worlds, key=lambda w: w["total"]) if worlds else None
    return {
        "score": round(float(np.mean(totals)), 2) if totals else math.nan,
        "worst": {"name": worst["name"], "total": worst["total"]} if worst else None,
        "dq": sum(1 for w in worlds if w["dq"]),
        "timeouts": sum(1 for w in worlds if w["dq"] == "timeout"),
        "gold": round(float(np.mean([w["gold"] for w in worlds])), 4),
        "consistency": round(float(np.mean([w["consistency"] for w in worlds])), 4),
        "speed": round(float(np.mean([w["speed"] for w in worlds])), 4),
        "worlds": len(worlds),
    }
