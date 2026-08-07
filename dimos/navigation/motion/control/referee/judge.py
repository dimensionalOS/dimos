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

"""The executor judge: score what the robot DID, not what was planned.

Referee-shaped — ``total = gate * (100*arrived + 10*precision + 1*pace)``,
max 111 — with the same lexicographic discipline: pace never buys back a
precision violation, and nothing buys back a crash or a fall (gate 0).

Precision is judged in CLEARANCE space, not deviation space: the embodiment
declares ``precision = 0.05`` — the clearance floor the planner treats as
real — and the pillar is the time the executed body spent below that floor
against exact truth boxes. Deviation with room around it is free (the open
half of a world is not a tightrope); the same deviation beside a wall is the
violation. That makes the score context-specific with zero per-world tuning,
and it is fair to a clearance-blind controller: tracking within the floor
everywhere satisfies it maximally. Cross-track stays as a raw diagnostic.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from dimos.navigation.motion.control.referee.episode import EpisodeResult
from dimos.navigation.motion.control.tracks import track_of
from dimos.navigation.motion.planner.referee.scenarios import Scenario
from dimos.navigation.motion.simulation.walk import COMMAND_SLEW

# The cruise target and the pace weight are per track and travel together --
# see control/tracks.py, which is also what the CLI and the adapter name.
GRACE_S = 2.0  # flat allowance: settle, first-step lag, terminal deceleration
TILT_SCALE = 0.35  # tilt p99 (rad, ~20 deg) at which stability hits 0
CHURN_SCALE = 0.25  # forced-replan deviation (m) at which calm hits 0 (referee CONSIST_SCALE)


def cross_track(pos_xy: np.ndarray, path_xy: np.ndarray) -> np.ndarray:
    """Distance from each executed point to the planned polyline."""
    if len(path_xy) == 0 or len(pos_xy) == 0:
        return np.zeros(0)
    if len(path_xy) == 1:
        return np.asarray(np.linalg.norm(pos_xy - path_xy[0], axis=1))
    a, b = path_xy[:-1], path_xy[1:]
    ab = b - a
    denom = np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-12)
    # (n_pos, n_seg) projection parameter, clamped to the segment
    ap = pos_xy[:, None, :] - a[None, :, :]
    s = np.clip(np.einsum("nij,ij->ni", ap, ab) / denom, 0.0, 1.0)
    closest = a[None, :, :] + s[..., None] * ab[None, :, :]
    return np.asarray(np.min(np.linalg.norm(pos_xy[:, None, :] - closest, axis=2), axis=1))


def executed_clearance(result: EpisodeResult) -> np.ndarray:
    """Exact body clearance to truth per tick: footprint samples vs box SDFs."""
    n = len(result.pos)
    sc = result.scenario
    if not sc.boxes or n == 0:
        return np.full(n, np.inf)
    off = sc.emb.offsets()  # (m, 2) body-frame samples, center_off included
    xy = result.pos[:, :2]
    c, s = np.cos(result.yaw), np.sin(result.yaw)
    wx = xy[:, 0, None] + c[:, None] * off[None, :, 0] - s[:, None] * off[None, :, 1]
    wy = xy[:, 1, None] + s[:, None] * off[None, :, 0] + c[:, None] * off[None, :, 1]
    pts = np.column_stack([wx.ravel(), wy.ravel()])
    d = np.min(np.stack([b.sdf2d(pts) for b in sc.boxes]), axis=0)
    return np.asarray(d.reshape(n, -1).min(axis=1))


def _plan_xy(result: EpisodeResult) -> np.ndarray:
    return np.array([[p.position.x, p.position.y] for p in result.plan.poses]).reshape(-1, 2)


def _ref_xy(ref: Any) -> np.ndarray:
    return np.array([[p.position.x, p.position.y] for p in ref.poses]).reshape(-1, 2)


def active_cross_track(result: EpisodeResult) -> np.ndarray:
    """Cross-track of each tick against the plan that was ACTIVE at that tick.

    Under receding horizon every replan starts at the robot, so error against
    the latest plan resets to ~zero exactly when the follower drifts — the
    replan must not amnesty the executor. Past ticks stay scored against the
    instruction they were actually following.
    """
    if not result.plans or not len(result.t):
        return np.zeros(0)
    starts = np.asarray(result.plan_t)
    which = np.clip(np.searchsorted(starts, result.t, side="right") - 1, 0, len(starts) - 1)
    out = np.zeros(len(result.t))
    for k in range(len(starts)):
        m = which == k
        if m.any():
            out[m] = cross_track(result.pos[m, :2], _ref_xy(result.plans[k]))
    return out


CHURN_HORIZON = 2.0  # compare plans over this much arc ahead (m)


def plan_churn(result: EpisodeResult) -> float:
    """How much the follower forced the planner to re-route (m).

    The world is static and the planner deterministic, so any difference
    between plan n+1 and plan n's remainder is caused by the executor being
    off the line (plus pose staleness). Max near-field deviation of each new
    plan from its predecessor's remainder, over the first CHURN_HORIZON of
    arc — the same consistency idea the referee applies to the planner,
    pointed back at the executor.
    """
    worst = 0.0
    for prev, new in zip(result.plans[:-1], result.plans[1:], strict=False):
        pxy, nxy = _ref_xy(prev), _ref_xy(new)
        if len(pxy) < 2 or len(nxy) < 1:
            continue
        # remainder of the previous plan from the point nearest the new start
        i = int(np.argmin(np.linalg.norm(pxy - nxy[0], axis=1)))
        remainder = pxy[i:]
        if len(remainder) < 2:
            continue
        arcs = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(nxy, axis=0), axis=1))])
        near = nxy[arcs <= CHURN_HORIZON]
        if len(near):
            worst = max(worst, float(np.max(cross_track(near, remainder))))
    return worst


def _saturation(result: EpisodeResult) -> float:
    """Fraction of active ticks where the hardware slew clipped the request."""
    if len(result.twist_cmd) < 2:
        return 0.0
    gap = np.abs(result.twist_cmd - result.used_cmd)
    return float(np.mean(np.any(gap > COMMAND_SLEW[None, :] * 1.001, axis=1)))


def score_episode(result: EpisodeResult) -> dict[str, Any]:
    """One executed episode -> pillars, total, and the raw stats behind them."""
    sc = result.scenario
    refuse_world = sc.expect == "refuse"
    out: dict[str, Any] = {
        "name": sc.name,
        "outcome": result.outcome,
        "dq": result.outcome in ("collision", "fall"),
        "time_to_goal": result.time_to_goal,
    }

    # Gate: physics vetoes everything else -- but the diagnostics still tell
    # the story of HOW it died.
    if out["dq"]:
        clear = executed_clearance(result)
        xt = active_cross_track(result)
        out.update(
            arrived=0.0,
            precision=0.0,
            pace=0.0,
            composure=0.0,
            min_clear=round(float(np.min(clear)), 4) if len(clear) else math.inf,
            below_floor=round(float(np.mean(clear < sc.emb.precision)), 4) if len(clear) else 0.0,
            xtrack_p95=round(float(np.percentile(xt, 95)), 4) if len(xt) else 0.0,
            xtrack_max=round(float(np.max(xt)), 4) if len(xt) else 0.0,
            churn=round(plan_churn(result), 4),
            plan_tight=round(min(result.plan_min_clear), 4) if result.plan_min_clear else math.inf,
            tilt_p99=round(float(np.percentile(result.tilt, 99)), 4) if len(result.tilt) else 0.0,
            total=0.0,
        )
        return out

    refused_ok = refuse_world and result.outcome in ("refused", "timeout")
    arrived = 1.0 if (result.outcome == "goal" or refused_ok) else 0.0

    # Precision: time spent below the clearance floor the plan was built on.
    clear = executed_clearance(result)
    floor = sc.emb.precision
    below = float(np.mean(clear < floor)) if len(clear) else 0.0
    precision = 1.0 if refused_ok else max(0.0, 1.0 - below)

    # Pace: arrival at cruise over the FIRST plan's arc -- the original task
    # length; under replanning the latest plan shrinks to nothing as the
    # robot closes in. Bottom tier, like the referee's speed pillar: it
    # never buys back a precision violation.
    first_xy = _ref_xy(result.plans[0]) if result.plans else _plan_xy(result)
    arc = (
        float(np.sum(np.linalg.norm(np.diff(first_xy, axis=0), axis=1)))
        if len(first_xy) > 1
        else 0.0
    )
    # The hinted track is allowed to outrun the path encoding, so it gets a
    # faster cruise target AND a pace pillar heavy enough to chase (115.5-max
    # totals). Blind cannot legally exceed the encoding: mid-band cruise,
    # bottom-tier weight, totals stay 111-shaped.
    track = track_of(result.cfg.annotate_clearance)
    if refused_ok:
        pace = 1.0
    elif result.outcome == "goal" and result.time_to_goal is not None:
        pace = min(1.0, (arc / track.cruise + GRACE_S) / max(result.time_to_goal, 1e-6))
    else:
        pace = 0.0
    pace_weight = track.pace_weight
    tilt_p99 = float(np.percentile(result.tilt, 99)) if len(result.tilt) else 0.0
    stability = max(0.0, 1.0 - tilt_p99 / TILT_SCALE)
    sat = _saturation(result)
    churn = plan_churn(result)
    calm = max(0.0, 1.0 - churn / CHURN_SCALE)
    composure = (stability + (1.0 - sat) + calm) / 3.0

    xt = active_cross_track(result)
    out.update(
        arrived=arrived,
        precision=round(precision, 4),
        pace=round(pace, 4),
        composure=round(composure, 4),
        min_clear=round(float(np.min(clear)), 4) if len(clear) else math.inf,
        below_floor=round(below, 4),
        xtrack_p95=round(float(np.percentile(xt, 95)), 4) if len(xt) else 0.0,
        xtrack_max=round(float(np.max(xt)), 4) if len(xt) else 0.0,
        churn=round(churn, 4),
        plan_tight=round(min(result.plan_min_clear), 4) if result.plan_min_clear else math.inf,
        tilt_p99=round(tilt_p99, 4),
        saturation=round(sat, 4),
        plan_ms=round(float(np.max(result.plan_ms)), 2) if result.plan_ms else 0.0,
        total=round(100.0 * arrived + 10.0 * precision + pace_weight * pace + 0.5 * composure, 2),
    )
    return out


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    totals = [r["total"] for r in rows]
    worst = min(rows, key=lambda r: r["total"]) if rows else None
    outcomes: dict[str, int] = {}
    for r in rows:
        outcomes[r["outcome"]] = outcomes.get(r["outcome"], 0) + 1
    return {
        "score": round(float(np.mean(totals)), 2) if totals else math.nan,
        "worst": {"name": worst["name"], "total": worst["total"]} if worst else None,
        "dq": sum(1 for r in rows if r["dq"]),
        "outcomes": outcomes,
        "arrived": round(float(np.mean([r["arrived"] for r in rows])), 4) if rows else math.nan,
        "precision": round(float(np.mean([r["precision"] for r in rows])), 4) if rows else math.nan,
        "pace": round(float(np.mean([r["pace"] for r in rows])), 4) if rows else math.nan,
        "worlds": len(rows),
    }


def print_row(row: dict[str, Any], sc: Scenario) -> None:
    ttg = f"{row['time_to_goal']:5.1f}s" if row["time_to_goal"] is not None else "    --"
    mc = row.get("min_clear", math.inf)
    mc_s = f"{mc:5.2f}" if math.isfinite(mc) else "  inf"
    print(
        f"{row['name']:<18s} {row['outcome']:<9s} {ttg}"
        f"  clear {mc_s}  below {row.get('below_floor', 0.0):4.2f}"
        f"  xt95 {row.get('xtrack_p95', 0.0):5.2f}  churn {row.get('churn', 0.0):5.2f}"
        f"  ptight {row.get('plan_tight', 0.0):5.2f}"
        f"  tilt99 {row.get('tilt_p99', 0.0):5.2f}  {row['total']:6.2f}  {sc.note}"
    )
