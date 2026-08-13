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

"""sim: validate avoidance plans in synthetic 2D worlds — no robot motion.

Ground-truth worlds of oriented boxes, sampled into a planner point cloud;
the candidate planner runs at the start pose and the resulting plan is
judged against the exact world (dense truth sampling the planner never
sees), not the planner's own map.

  python -m dimos.navigation.motion.planner            # battery table
  python -m dimos.navigation.motion.planner --view     # write sim2d.rrd
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import itertools
import math
import time
from typing import Any

import numpy as np

from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.motion.embodiment import EMBODIMENTS, Embodiment
from dimos.navigation.motion.geometry import (
    GO2_BODY,
    NEAR_FIELD_M,
    SCORE_STRIDE_M,
    AvoidanceConfig,
    CollisionShape,
    DistanceField,
    _path_arcs,
    angle_diff,
    body_shape,
    near_field_diff,
    scored_clearance,
    station_poses,
    travel_drift,
    turn_mask,
)
from dimos.navigation.motion.obstacles import hard_points, load as load_model
from dimos.navigation.motion.planner.planners.base import load
from dimos.navigation.motion.scenarios import (
    COMMIT_MARGIN,
    GEN_COUNT,
    GEN_SEED,
    SCENARIOS,
    Box,
    Scenario,
    generated,
    path_cost,
    rebody,
    recorded,
    se2_path,
    straight_plan,
    truth_grid,
)

from .score import score_world, summarize

# Planner's view: local_map-like sampling. Truth: dense, planner never sees it.
CLOUD_STEP = 0.05
TRUTH_STEP = 0.02
REPEATS = 2  # seeded re-runs on identical input: the band must be a fixed point
# Consistency chain: advance the start pose down the candidate's own path and
# replan — each answer must agree with its predecessor's remainder. Light
# calls: no dense-truth judging, just the deviation metric.
#
# The chain replays the DEPLOYMENT, so each query is handed its predecessor's
# answer, exactly as `adapter/planner.py` and the control episode hand theirs
# over. That makes an answer that moved a scoreable event rather than a
# reported one: the world here is static and truth is on the table, so the
# referee prices both routes itself and a switch truth does not pay for by
# `COMMIT_MARGIN` is an UNEARNED SWITCH. See planner/revision.md's commitment
# amendment. The first plan of every world still has no incumbent, so
# everything else the verdict carries is untouched.
SPOTS = 2
SPOT_STEP = 0.33  # fraction of the previous solution's arc per advance
# Deviation at which two answers are different ROUTES rather than the same one
# re-smoothed — the control judge's own `rerolls` scale (referee/judge.py).
SWITCH_M = 0.15


@dataclass
class Verdict:
    scenario: Scenario
    final: Path
    plan: Path
    cloud: PointCloud2
    truth_pts: np.ndarray
    repeats: list[np.ndarray]  # xy of each seeded re-run (indecision, visible)
    min_scored: float  # planner's belief along its own plan
    min_truth: float  # exact world clearance along the plan, per-heading (the judge)
    min_union: float  # the same sweep with the all-gait union: the outer bound
    env_viol: float  # deepest envelope violation (union hits, the heading row does not)
    truth_clear: np.ndarray  # per scored waypoint, union (the sweep markers' outer bound)
    # per scored waypoint: (cx, cy, yaw, sx, sy, is_box) of the swept shape
    swept_shapes: np.ndarray
    lat_mean: float
    lat_max: float
    veto: bool
    flicker: float  # max near-field diff across seeded repeats, identical input
    consist: float  # max deviation of chained replans from their predecessor
    chain: list[tuple[np.ndarray, np.ndarray]]  # (replan xy, spot pose xy) per spot
    chain_steps: int  # chained replans actually made (0 = nothing to judge)
    unearned: int  # of those, the ones that switched route without earning it
    fans: int
    avoid_ms: float  # min plan() CPU time across the repeats
    gold: np.ndarray | None  # SE(2) body-true reference maneuver
    gold_ms: float  # measured se2_path() call (cache hits read ~0)
    timed_out: bool = False  # a plan call blew the eval time limit


def _fan_marks(path: Path, cfg: AvoidanceConfig) -> tuple[np.ndarray, int]:
    """Candidate-agnostic fan detection: waypoints where the path commands
    yaw with (near-)zero displacement. Returns (per-waypoint mask, run count)."""
    xy = np.array([[p.position.x, p.position.y] for p in path.poses])
    yaws = np.array([p.orientation.euler[2] for p in path.poses])
    mask = np.zeros(len(xy), dtype=bool)
    for i in range(1, len(xy)):
        ds = float(np.linalg.norm(xy[i] - xy[i - 1]))
        dyaw = abs(angle_diff(float(yaws[i]), float(yaws[i - 1])))
        # Commanded rotation = steep yaw-per-meter (fans run ~5 rad/m at
        # turn_spacing; ordinary curves stay well under 2) or a pure step.
        if dyaw > cfg.turn_yaw_eps and (ds < 1e-6 or dyaw / max(ds, 1e-6) > 3.0):
            mask[i] = True
            mask[i - 1] = True
    runs = int(np.sum(np.diff(mask.astype(int)) == 1) + (1 if mask[:1].any() else 0))
    return mask, runs


def _priced(spot: tuple[float, float, float], path: Path) -> np.ndarray:
    """A published path as SE(2), priced from where the robot is standing —
    both routes in a comparison have to be walked to from the same place."""
    return np.vstack(
        [[list(spot)], [[p.position.x, p.position.y, p.orientation.euler[2]] for p in path.poses]]
    )


def _emb_cfg(cfg: AvoidanceConfig, sc: Scenario) -> AvoidanceConfig:
    """Condition the config on the scenario's embodiment (GO2 = unchanged)."""
    e = sc.emb
    shape = body_shape(e)
    if repr(shape) == repr(GO2_BODY) and cfg.shape is GO2_BODY:
        return cfg
    return cfg.model_copy(update={"shape": shape, "veto_clearance": -e.precision})


def envelope_sweep(
    emb: Embodiment, union: CollisionShape, poses: Any, truth_pts: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Per-pose clearance to exact truth, measured twice: heading row, union.

    The row is the swept box the embodiment measured for the pose's own
    direction of travel -- the shape the search planned that edge with -- taken
    over BOTH segments adjacent to the pose, since the body arrives on one and
    leaves on the other. A pose with no direction of travel (a fan, a stop, the
    standing start) reads the union, the same honest fallback the search's
    turn-in-place edges take. The union is never narrower than a row, so the
    gap between the two readings is exactly the slack the planner's
    motion-conditioned assumption is spending.
    """
    xy = np.array([[p.position.x, p.position.y] for p in poses])
    yaws = np.array([p.orientation.euler[2] for p in poses])
    drifts = travel_drift(xy, yaws)
    shapes: dict[tuple[float, ...], CollisionShape] = {}
    row = np.empty(len(xy))
    uni = np.empty(len(xy))
    for i, p in enumerate(poses):
        uni[i] = float(np.min(union.at(p).distance(truth_pts)))
        vals = []
        for d in drifts[i]:
            key = tuple(round(v, 9) for v in emb.envelope_at(d))
            if key not in shapes:
                shapes[key] = body_shape(emb, d)
            vals.append(float(np.min(shapes[key].at(p).distance(truth_pts))))
        row[i] = min(vals) if vals else uni[i]
    return row, uni


def judge(
    sc: Scenario,
    cfg: AvoidanceConfig,
    planner: str = "target",
    time_limit_ms: float | None = None,
) -> Verdict:
    cfg = _emb_cfg(cfg, sc)
    cloud_pts = (
        np.concatenate([b.surface(CLOUD_STEP) for b in sc.boxes]) if sc.boxes else np.empty((0, 3))
    )
    truth_pts = (
        np.concatenate([b.surface(TRUTH_STEP) for b in sc.boxes]) if sc.boxes else np.empty((0, 3))
    )
    cloud = PointCloud2.from_numpy(cloud_pts.astype(np.float32), frame_id="world")
    # What the candidate is allowed to see, as obstacle xy: the search itself is
    # planar (planners/base.py), so the harness names the z rule here rather
    # than leaving one buried in a planner. These worlds stand on z = 0, which
    # is the frame `raw_band` is written for.
    obstacles = hard_points(load_model("raw_band", sc.emb), cloud.points_f32(), 0.0)[:, :2]
    plan = straight_plan(
        sc.start, sc.goal
    )  # display only: the hint concept lives inside candidates

    episode = load(planner)(sc, cfg)
    episode.reset()

    outs = []
    times = []
    timed_out = False
    for _ in range(REPEATS):
        # CPU time, not wall time: candidates are single-threaded by rule, so
        # this is the work the plan actually cost, and it does not absorb
        # whatever else the machine was doing during the call.
        t0 = time.process_time()
        outs.append(episode.plan(obstacles, sc.start, sc.goal))
        times.append((time.process_time() - t0) * 1e3)
        if time_limit_ms is not None and times[-1] > time_limit_ms:
            # Over the eval budget: stop feeding this candidate this world.
            timed_out = True
            break
    n = round(NEAR_FIELD_M / cfg.resolution)
    flicker = max((near_field_diff(a, b, n) for a, b in itertools.pairwise(outs)), default=0.0)

    # Consistency chain: walk the start pose down the latest solution and
    # replan, handing each query the answer before it; the new answer must
    # match the old answer's remainder, and where it does not, truth has to
    # justify the difference.
    consist = 0.0
    chain: list[tuple[np.ndarray, np.ndarray]] = []
    chain_steps = 0
    unearned = 0
    grid: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
    prev_out = outs[-1]
    for _ in range(0 if timed_out else SPOTS):
        pxy = np.array([[p.position.x, p.position.y] for p in prev_out.poses])
        if len(pxy) < 3:
            break
        parcs = _path_arcs(pxy)
        s_idx = int(np.searchsorted(parcs, SPOT_STEP * float(parcs[-1])))
        s_idx = min(max(s_idx, 1), len(pxy) - 2)
        tang = pxy[s_idx + 1] - pxy[s_idx - 1]
        spot = (float(pxy[s_idx][0]), float(pxy[s_idx][1]), math.atan2(tang[1], tang[0]))
        remainder = Path(ts=0.0, frame_id=prev_out.frame_id, poses=prev_out.poses[s_idx:])
        new_out = episode.plan(obstacles, spot, sc.goal, prev_out)
        dev = near_field_diff(remainder, new_out, n)
        consist = max(consist, dev)
        chain_steps += 1
        if dev > SWITCH_M:
            # A different route. The referee prices both from the spot on the
            # BOX-EXACT truth field — the candidate had to judge this on a
            # cloud, the judge does not — and a switch truth will not pay
            # `COMMIT_MARGIN` for is one the candidate made for no reason.
            if grid is None:
                fgx, fgy, sdf, _ = truth_grid(sc.boxes, sc.start, sc.goal)
                grid = (fgx, fgy, sdf)
            held = path_cost(*grid, _priced(spot, remainder), sc.emb)
            fresh = path_cost(*grid, _priced(spot, new_out), sc.emb)
            unearned += int(not fresh < held - COMMIT_MARGIN)
        chain.append((np.array([[p.position.x, p.position.y] for p in new_out.poses]), pxy[s_idx]))
        prev_out = new_out

    field = DistanceField(cloud)
    final = outs[-1]
    fanwp, fan_count = _fan_marks(final, cfg)

    # Score exactly like the module: stride + fan waypoints over the horizon.
    stride = max(1, round(SCORE_STRIDE_M / cfg.resolution))
    xy = np.array([[p.position.x, p.position.y] for p in final])
    arcs = _path_arcs(xy)
    k = min(len(final), int(np.searchsorted(arcs, cfg.horizon, side="right")) + 1)
    idx = sorted(set(range(0, k, stride)) | set(np.flatnonzero(fanwp[:k]).tolist()))
    poses, _, owner = station_poses(final, arcs, idx, end=k - 1)
    s_fan = np.where(owner >= 0, fanwp[owner], False)
    yaws = np.array([p.orientation.euler[2] for p in poses])
    turn = turn_mask(yaws, cfg.turn_yaw_eps) & ~s_fan
    swept, clear = scored_clearance(field, cfg.shape, poses, cfg, turn=turn, prev_yaw=sc.start[2])

    # Truth at the scored waypoints, on the union: these are the sweep markers,
    # and a marker's job is to show the outer bound the body may occupy...
    if len(truth_pts):
        truth = np.array([float(np.min(cfg.shape.at(p).distance(truth_pts))) for p in poses])
        # ...but the gate judges EVERY final pose (a stride sampler steps clean
        # over a 0.15 m wall -- boxed_in scored +0.03 through one), and it
        # judges the body the plan actually promised: the envelope row for that
        # pose's own direction of travel, since planner/revision.md that is what
        # the search plans each edge with. The union reading rides along as the
        # outer bound, and where the two disagree the difference is an ENVELOPE
        # VIOLATION -- the world reaches inside the slack between what the
        # planner assumed and what any gait could do -- named and reported
        # instead of silently DQ-ing a path nothing touches.
        row_d, uni_d = envelope_sweep(sc.emb, cfg.shape, final.poses[:k], truth_pts)
        dense = float(np.min(row_d))
        dense_union = float(np.min(uni_d))
        clean = row_d > -1e-6
        env_viol = float(np.max(np.maximum(-uni_d[clean], 0.0))) if clean.any() else 0.0
    else:
        truth = np.full(len(poses), np.inf)
        dense = dense_union = math.inf
        env_viol = 0.0

    t0 = time.perf_counter()
    gold = se2_path(sc.boxes, sc.start, sc.goal, sc.emb)
    gold_ms = (time.perf_counter() - t0) * 1e3

    # Lateral offset vs the straight start->goal line (side choice).
    u = np.array([sc.goal[0] - sc.start[0], sc.goal[1] - sc.start[1]])
    u = u / max(np.linalg.norm(u), 1e-9)
    nrm = np.array([-u[1], u[0]])
    off = (xy - np.array(sc.start[:2])) @ nrm
    return Verdict(
        scenario=sc,
        final=final,
        plan=plan,
        cloud=cloud,
        truth_pts=truth_pts,
        repeats=[np.array([[p.position.x, p.position.y] for p in o.poses]) for o in outs],
        min_scored=float(np.min(clear)) if len(clear) else math.inf,
        min_truth=dense,
        min_union=dense_union,
        env_viol=env_viol,
        truth_clear=truth,
        swept_shapes=np.array(
            [
                [
                    s.pose.position.x,
                    s.pose.position.y,
                    s.pose.orientation.euler[2],
                    s.primitive.dimensions[0]
                    if s.primitive.type == s.primitive.BOX
                    else 2 * s.primitive.dimensions[1],
                    s.primitive.dimensions[1]
                    if s.primitive.type == s.primitive.BOX
                    else 2 * s.primitive.dimensions[1],
                    float(s.primitive.type == s.primitive.BOX),
                ]
                for s in swept
            ]
        ),
        lat_mean=float(np.mean(off)),
        lat_max=float(np.max(np.abs(off))),
        veto=bool(np.any(clear < cfg.veto_clearance)),
        flicker=float(flicker),
        consist=float(consist),
        chain=chain,
        chain_steps=chain_steps,
        unearned=unearned,
        fans=fan_count,
        avoid_ms=float(np.min(times)),  # min-of-repeats: least load noise
        gold=gold,
        gold_ms=gold_ms,
        timed_out=timed_out,
    )


def densify_se2(se2: np.ndarray) -> np.ndarray:
    """Shortcut-smoothed vertices to a dense pose sequence (yaw = shortest arc).

    The gold's own validity check interpolates exactly this way, so a body
    ridden along this is a body the oracle proved fits.
    """
    dense = [se2[0]]
    for a, b in itertools.pairwise(se2):
        dyaw = math.remainder(b[2] - a[2], 2 * math.pi)
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.15), int(abs(dyaw) / 0.2))
        for t in np.linspace(1.0 / n, 1.0, n):
            dense.append(
                np.array([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * dyaw])
            )
    return np.array(dense)


def body_marks(se2: np.ndarray, e: Any, z: float, step: float = 0.35) -> list[np.ndarray]:
    """Body-box outlines along a dense maneuver: every `step` m, and every turn."""
    seg = np.linalg.norm(np.diff(se2[:, :2], axis=0), axis=1)
    arcs = np.concatenate([[0.0], np.cumsum(seg)])
    marks = np.unique(
        np.concatenate(
            [
                np.searchsorted(arcs, np.arange(0.0, arcs[-1], step)),
                np.flatnonzero(np.diff(se2[:, 2]) != 0.0),
                [len(se2) - 1],
            ]
        )
    )
    outs = []
    for k in marks:
        x, y, yaw = se2[k]
        cx = x + math.cos(yaw) * e.center_off
        cy = y + math.sin(yaw) * e.center_off
        outs.append(Box(cx, cy, e.length, e.width, yaw).outline(z))
    return outs


def geo_extent(sc: Scenario) -> tuple[float, float, float, float]:
    """(x0, x1, y0, y1) from world geometry alone — known BEFORE judging, so
    the grid streams case by case instead of waiting for the whole battery.
    Paths can overshoot the pad slightly; placement stays fixed regardless."""
    xs = [sc.start[0], sc.goal[0]]
    ys = [sc.start[1], sc.goal[1]]
    for b in sc.boxes:
        o = b.outline(0.0)
        xs.extend(o[:, 0])
        ys.extend(o[:, 1])
    return float(min(xs)) - 1.0, float(max(xs)) + 1.0, float(min(ys)) - 1.0, float(max(ys)) + 1.0


def render(
    v: Verdict,
    dx: float = 0.0,
    dy: float = 0.0,
    ext: tuple[float, float, float, float] | None = None,
) -> None:
    """Log one scenario under its own entity root, shifted to (dx, dy)."""
    import rerun as rr

    off3 = np.array([dx, dy, 0.0])
    root = v.scenario.name
    e = v.scenario.emb
    if ext is not None:
        x0, x1, y0, y1 = ext
        border = np.array(
            [(x0, y0, 0.0), (x1, y0, 0.0), (x1, y1, 0.0), (x0, y1, 0.0), (x0, y0, 0.0)]
        )
        rr.log(
            f"{root}/border",
            rr.LineStrips3D([border + off3], colors=[[255, 255, 255, 70]], radii=0.006),
            static=True,
        )

    def strip(path: str, xyz: np.ndarray, color: list[int]) -> None:
        rr.log(f"{root}/{path}", rr.LineStrips3D([xyz + off3], colors=[color]), static=True)

    # Gold standard: the SE(2) brute-force maneuver with the real body —
    # body boxes at the search's own yaws (turns only where it truly fits).
    se2 = v.gold
    if se2 is not None:
        # Densify the shortcut-smoothed vertices so body boxes ride the whole
        # maneuver (yaw = shortest arc, matching the validity check).
        se2 = densify_se2(se2)
        gold = [255, 190, 40]
        line = np.column_stack([se2[:, :2], np.full(len(se2), 0.02)])
        rr.log(
            f"{root}/ideal_se2",
            rr.LineStrips3D([line + off3], colors=[gold], radii=0.012),
            static=True,
        )
        rr.log(
            f"{root}/ideal_se2_body",
            rr.LineStrips3D(
                [o + off3 for o in body_marks(se2, e, 0.02)], colors=[gold], radii=0.004
            ),
            static=True,
        )
        # Animated body on the "step" timeline: scrub to watch the maneuver.
        for t, (x, y, yaw) in enumerate(se2):
            rr.set_time("step", sequence=t)
            cx = x + math.cos(yaw) * e.center_off
            cy = y + math.sin(yaw) * e.center_off
            rr.log(
                f"{root}/anim/gold_body",
                rr.LineStrips3D(
                    [Box(cx, cy, e.length, e.width, yaw).outline(0.03) + off3],
                    colors=[gold],
                    radii=0.015,
                ),
            )

    def body_outlines(xy: np.ndarray, z: float, step: float = 0.35) -> list[np.ndarray]:
        """GO2 body box outlines sampled every `step` m along a polyline,
        yaw from tangents (positions-only paths carry no orientation)."""
        if len(xy) < 2:
            return []
        seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        arcs = np.concatenate([[0.0], np.cumsum(seg)])
        marks = np.arange(0.0, arcs[-1], step)
        idx = np.unique(np.searchsorted(arcs, marks))
        outs = []
        for i in idx:
            j = min(i + 1, len(xy) - 1)
            t = xy[j] - xy[max(i - 1, 0)]
            yaw = math.atan2(t[1], t[0]) if np.linalg.norm(t) > 1e-9 else 0.0
            cx = xy[i][0] + math.cos(yaw) * e.center_off
            cy = xy[i][1] + math.sin(yaw) * e.center_off
            outs.append(Box(cx, cy, e.length, e.width, yaw).outline(z) + off3)
        return outs

    scored = "inf" if math.isinf(v.min_scored) else f"{v.min_scored:.2f}"
    truth = "inf" if math.isinf(v.min_truth) else f"{v.min_truth:.2f}"
    tagline = "" if e.tag == "go2" else f" [{e.tag} {e.length:.2f}x{e.width:.2f}]"
    caption = (
        f"{v.scenario.name}{tagline}\n"
        f"avoid {v.avoid_ms:.0f} ms · gold {v.gold_ms:.0f} ms\n"
        f"scored {scored} · truth {truth}" + ("\nVETO" if v.veto else "")
    )
    lx, ly = (ext[0] + 0.3, ext[3] - 0.2) if ext is not None else (-1.0, 0.0)
    rr.log(
        f"{root}/label",
        rr.Points3D([[lx + dx, ly + dy, 0.3]], radii=0.001, labels=[caption]),
        static=True,
    )
    for i, b in enumerate(v.scenario.boxes):
        for z in (0.0, b.height):
            strip(f"world/box{i}_z{z:.1f}", b.outline(z), [180, 180, 180])
    pts, _ = v.cloud.as_numpy()
    if len(pts):
        rr.log(
            f"{root}/world/cloud",
            rr.Points3D(pts + off3, radii=0.008, colors=[[120, 120, 140]]),
            static=True,
        )
    plan_xy = np.array([[p.position.x, p.position.y, 0.02] for p in v.plan.poses])
    strip("plan", plan_xy, [100, 100, 255])
    # All seeded repeats: identical input, so any spread you can SEE is the
    # band failing to be a fixed point (the indecision core). Body boxes ride
    # each line in its color — divergence in actual body geometry.
    reds = [[255, 80, 80], [90, 160, 255], [255, 220, 60]]
    for i, xy in enumerate(v.repeats):
        z = 0.04 + 0.01 * i
        xyz = np.column_stack([xy, np.full(len(xy), z)])
        strip(f"avoid/repeat{i}", xyz, reds[i % len(reds)])
        bodies = body_outlines(xy, z)
        if bodies:
            rr.log(
                f"{root}/avoid/repeat{i}_body",
                rr.LineStrips3D(bodies, colors=[reds[i % len(reds)]], radii=0.0015),
                static=True,
            )
    # Consistency chain: replans from spots down our own path (magenta, thin);
    # a dot marks each spot. Deviation from the white line = inconsistency.
    for ci, (cxy, spot_xy) in enumerate(v.chain):
        cz = np.column_stack([cxy, np.full(len(cxy), 0.06 + 0.01 * ci)])
        rr.log(
            f"{root}/chain/replan{ci}",
            rr.LineStrips3D([cz + off3], colors=[[255, 100, 255]], radii=0.003),
            static=True,
        )
        rr.log(
            f"{root}/chain/spot{ci}",
            rr.Points3D(
                [[spot_xy[0] + dx, spot_xy[1] + dy, 0.1]], radii=0.03, colors=[[255, 100, 255]]
            ),
            static=True,
        )
    fin = np.array([[p.position.x, p.position.y, 0.08] for p in v.final.poses])
    strip("avoid/final", fin, [255, 255, 255])
    # Required precision, as the wire dialect encodes it (path timestamps):
    # circle radius = the per-waypoint clearance hint -- nearest obstacle point
    # minus the half-width, capped at 0.35 m where the governor grants full
    # speed anyway. Red = at/under the precision floor (creep + track tight),
    # yellow = governed, green = full speed. Keep the math in step with
    # control/world.path_clearance and control/profile.py -- which means
    # reading the obstacles through a model, as the harness does.
    if len(pts) and len(fin):
        from scipy.spatial import cKDTree

        band = hard_points(load_model("raw_band", e), pts, 0.0)[:, :2]
        if len(band):
            d, _ = cKDTree(band).query(fin[:, :2])
            clear = d - e.width / 2.0
            full = 0.35  # AvoidanceConfig.speed_clearance
            seg = np.linalg.norm(np.diff(fin[:, :2], axis=0), axis=1)
            arcs = np.concatenate([[0.0], np.cumsum(seg)])
            idx = np.unique(np.searchsorted(arcs, np.arange(0.0, arcs[-1] + 1e-9, 0.35)))
            circles, pcols = [], []
            a = np.linspace(0.0, 2 * math.pi, 33)
            for i in idx:
                r = float(min(max(clear[i], 0.02), full))
                cx, cy = fin[i][0], fin[i][1]
                circles.append(
                    np.column_stack([cx + r * np.cos(a), cy + r * np.sin(a), np.full(33, 0.05)])
                    + off3
                )
                if clear[i] <= e.precision:
                    pcols.append([255, 60, 60])
                elif clear[i] < full:
                    pcols.append([255, 220, 60])
                else:
                    pcols.append([80, 220, 80])
            rr.log(
                f"{root}/avoid/precision",
                rr.LineStrips3D(circles, colors=pcols, radii=0.002),
                static=True,
            )
    # The published path's body, same "step" timeline (commanded yaws).
    for t, p in enumerate(v.final.poses):
        rr.set_time("step", sequence=t)
        yaw = float(p.orientation.euler[2])
        cx = p.position.x + math.cos(yaw) * e.center_off
        cy = p.position.y + math.sin(yaw) * e.center_off
        rr.log(
            f"{root}/anim/plan_body",
            rr.LineStrips3D(
                [Box(cx, cy, e.length, e.width, yaw).outline(0.09) + off3],
                colors=[[255, 255, 255]],
                radii=0.012,
            ),
        )
    # Swept body poses (live sweep-marker equivalent): oriented box outline
    # per scored waypoint, colored by TRUTH clearance (red hit, yellow tight,
    # green clear). Turn waypoints render as their placement circle.
    strips, cols = [], []
    for (cx, cy, yaw, sx, sy, is_box), t in zip(v.swept_shapes, v.truth_clear, strict=False):
        if is_box:
            o = Box(cx, cy, sx, sy, yaw).outline(0.06)
        else:
            a = np.linspace(0.0, 2 * math.pi, 25)
            o = np.column_stack(
                [cx + sx / 2 * np.cos(a), cy + sy / 2 * np.sin(a), np.full(25, 0.06)]
            )
        strips.append(o + off3)
        cols.append([255, 60, 60] if t < 0.0 else ([255, 220, 60] if t < 0.10 else [80, 220, 80]))
    rr.log(f"{root}/avoid/sweep", rr.LineStrips3D(strips, colors=cols, radii=0.0015), static=True)
    syaw = v.scenario.start[2]
    body = Box(
        v.scenario.start[0] + math.cos(syaw) * e.center_off,
        v.scenario.start[1] + math.sin(syaw) * e.center_off,
        e.length,
        e.width,
        syaw,
        height=0.4,
    )
    strip("robot/body_floor", body.outline(0.0), [255, 200, 0])
    strip("robot/body_top", body.outline(0.4), [255, 200, 0])


def _record(v: Verdict, sc: Scenario, score: bool) -> dict[str, Any]:
    """One world's result as plain data: table row fields + score dict."""
    side = "L" if v.lat_mean > 0.02 else ("R" if v.lat_mean < -0.02 else "-")
    return {
        "name": sc.name,
        "note": sc.note,
        "expect": sc.expect,
        "emb": sc.emb.tag,
        "side": side,
        "lat_max": v.lat_max,
        "min_scored": v.min_scored,
        "min_truth": v.min_truth,
        "min_union": v.min_union,
        "env_viol": v.env_viol,
        "veto": v.veto,
        "fans": v.fans,
        "flicker": v.flicker,
        "consist": v.consist,
        "unearned": v.unearned,
        "avoid_ms": v.avoid_ms,
        "gold_ms": v.gold_ms,
        "timed_out": v.timed_out,
        "world": score_world(v) if score else None,
    }


def _print_row(r: dict[str, Any], score: bool) -> None:
    score_col = ""
    if score:
        w = r["world"]
        score_col = f"{'DQ' if w['dq'] else w['total']:>8}"
    print(
        f"{r['name']:<14}{r['side']:>6}{r['lat_max']:>9.2f}{r['min_scored']:>9.2f}"
        f"{r['min_truth']:>9.2f}{r['env_viol']:>7.3f}{r['veto']!s:>6}{r['fans']:>6}"
        f"{r['flicker']:>9.3f}"
        f"{r['consist']:>9.3f}{r['unearned']:>5d}{r['avoid_ms']:>7.1f}"
        f"{r['gold_ms']:>7.1f}{score_col}  {r['note']}"
    )


def _worker_main(
    core: int | None,
    planner: str,
    time_limit_ms: float | None,
    inq: Any,
    outq: Any,
) -> None:
    """Battery worker: judge scenarios pulled from inq, push (idx, record).

    Each worker is its own process pinned to its own core, so
    time.process_time() inside judge() measures exactly this worker's CPU —
    parallel workers do not contaminate each other's avoid_ms. (They do share
    L3/memory bandwidth and the turbo budget, so absolute timings carry a
    small conservative bias vs a serial run; gold and consistency are exact.)
    """
    import os

    if core is not None:
        try:
            os.sched_setaffinity(0, {core})
        except (AttributeError, OSError):
            pass  # non-Linux: unpinned, timings advisory
    while True:
        item = inq.get()
        if item is None:
            return
        idx, sc = item
        v = judge(sc, AvoidanceConfig(), planner=planner, time_limit_ms=time_limit_ms)
        outq.put((idx, _record(v, sc, score=True)))


def _run_parallel(
    todo: list[Scenario], planner: str, time_limit_ms: float | None, jobs: int
) -> list[dict[str, Any]]:
    """Fan the battery over worker processes, one pinned core each.

    The parent warms both pickle caches serially first (they are whole-file
    read-modify-write, unsafe under concurrent writers), then spawns workers
    with AUTORESEARCH_CACHE_RO=1 in the inherited environment — spawn
    re-imports scenarios.py in each child, which reads the flag at import.
    """
    import multiprocessing as mp
    import os

    for sc in todo:  # warm the gold cache with the judge's exact queries
        se2_path(sc.boxes, sc.start, sc.goal, sc.emb)

    try:
        cores = sorted(os.sched_getaffinity(0))
    except AttributeError:
        cores = []
    jobs = max(1, min(jobs, len(todo), len(cores) or jobs))
    ctx = mp.get_context("spawn")
    inq: Any = ctx.Queue()
    outq: Any = ctx.Queue()
    for item in enumerate(todo):
        inq.put(item)
    for _ in range(jobs):
        inq.put(None)

    os.environ["AUTORESEARCH_CACHE_RO"] = "1"
    try:
        workers = [
            ctx.Process(
                target=_worker_main,
                args=(cores[i] if cores else None, planner, time_limit_ms, inq, outq),
                daemon=True,
            )
            for i in range(jobs)
        ]
        for w in workers:
            w.start()
        results: dict[int, dict[str, Any]] = {}
        for _ in range(len(todo)):
            idx, rec = outq.get()
            results[idx] = rec
        for w in workers:
            w.join()
    finally:
        del os.environ["AUTORESEARCH_CACHE_RO"]
    return [results[i] for i in range(len(todo))]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-s", "--scenario", help="run one scenario by name")
    ap.add_argument("--view", action="store_true", help="write sim2d.rrd")
    ap.add_argument("--spawn", action="store_true", help="spawn a live viewer instead")
    ap.add_argument(
        "--gen",
        nargs="?",
        const=GEN_COUNT,
        default=0,
        type=int,
        help=f"append N generated worlds (default {GEN_COUNT}), cached per generator hash",
    )
    ap.add_argument("--seed", default=GEN_SEED, type=int, help="base seed for --gen")
    ap.add_argument(
        "--recorded",
        action="append",
        default=[],
        metavar="NPZ",
        help="append a world recorded on the robot (simulation.recorded_world npz)",
    )
    ap.add_argument(
        "--planner",
        default="target",
        help='candidate: registry name (target, target-py, gold) or "module:factory"',
    )
    ap.add_argument(
        "--emb",
        default="mix",
        choices=[*sorted(EMBODIMENTS), "mix"],
        help="body every world runs: one of the roster, or mix (rotate the "
        "roster per generated seed; curated and recorded stay go2). Naming a "
        "body re-labels the curated worlds against it -- a gap is only clear "
        "for whoever fits through it -- which costs a truth solve per world "
        "the first time, then caches",
    )
    ap.add_argument("--score", action="store_true", help="score each world + summary")
    ap.add_argument(
        "--quiet", action="store_true", help="suppress the table; print only the summary JSON"
    )
    ap.add_argument(
        "--time-limit-ms",
        type=float,
        default=None,
        help="per-plan-call budget for scored runs (default 6000 with --score)",
    )
    ap.add_argument(
        "--jobs",
        type=int,
        default=1,
        help="worker processes for the battery, one pinned core each (default 1: "
        "serial, identical to the classic run). Speed timings are per-process "
        "CPU time, so they stay valid; gold/consistency are exact either way.",
    )
    ap.add_argument(
        "--json",
        action="store_true",
        help="emit one JSON document with per-world records + summary (implies --score)",
    )
    ap.add_argument(
        "--build",
        action="store_true",
        help="build + install the rust candidate (maturin develop --release) first",
    )
    args = ap.parse_args()

    if args.build:
        import pathlib
        import subprocess

        manifest = pathlib.Path(__file__).parent / "rust" / "Cargo.toml"
        subprocess.run(
            ["uv", "run", "maturin", "develop", "--uv", "--release", "-m", str(manifest)],
            check=True,
        )
    if args.json:
        args.score = True
    if args.jobs > 1 and (args.view or args.spawn):
        ap.error("--jobs > 1 cannot render (--view/--spawn); run those serially")
    if args.jobs > 1 and not args.score:
        args.score = True  # parallel runs exist for scoring; records carry scores
    if args.score and args.time_limit_ms is None:
        args.time_limit_ms = 6000.0
    if args.score and args.jobs == 1:
        # Scored runs are timing runs: pin to one core so candidate threads
        # cannot help (single-thread rule) and ms is stable under load.
        # (With --jobs > 1 each WORKER pins itself; the parent stays free.)
        import os

        try:
            os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
        except (AttributeError, OSError):
            pass  # non-Linux: unpinned, timings advisory

    cfg = AvoidanceConfig()
    # None only for "mix", which is the roster rotation and names no single
    # body -- so it is also the one value that leaves the curated worlds on the
    # go2 they were drawn for. Naming a body means every world runs it.
    forced = EMBODIMENTS.get(args.emb)
    pool = (
        (SCENARIOS if forced is None else [rebody(sc, forced) for sc in SCENARIOS])
        + (generated(args.gen, args.seed, emb=forced) if args.gen else [])
        + [recorded(p) if forced is None else recorded(p, emb=forced) for p in args.recorded]
    )
    todo = [s for s in pool if args.scenario is None or s.name == args.scenario]
    if not todo:
        raise SystemExit(f"no scenario named {args.scenario!r}")

    if args.view or args.spawn:
        import rerun as rr

        rr.init("sim2d", spawn=args.spawn)
        if not args.spawn:
            rr.save("sim2d.rrd")

    hdr = (
        f"{'scenario':<14}{'side':>6}{'lat_max':>9}{'scored':>9}{'truth':>9}{'envio':>7}"
        f"{'veto':>6}{'fans':>6}{'flicker':>9}{'consist':>9}{'unrn':>5}{'ms':>7}{'gold':>7}  note"
    )
    if not args.quiet and not args.json:
        print(hdr)
        print("-" * len(hdr))
    # Grid placement from geometry alone, so rendering streams per case.
    ncols = max(1, math.ceil(math.sqrt(len(todo))))
    gap = 1.2
    place: dict[str, tuple[float, float, tuple[float, float, float, float]]] = {}
    top = 0.0
    for r in range(0, len(todo), ncols):
        row = todo[r : r + ncols]
        exts = [geo_extent(sc) for sc in row]
        cur_x = 0.0
        for sc, e in zip(row, exts, strict=False):
            place[sc.name] = (cur_x - e[0], top - e[3], e)
            cur_x += (e[1] - e[0]) + gap
        top -= max(e[3] - e[2] for e in exts) + gap

    records: list[dict[str, Any]] = []
    if args.jobs > 1:
        records = _run_parallel(todo, args.planner, args.time_limit_ms, args.jobs)
        if not args.quiet and not args.json:
            for rec in records:
                _print_row(rec, score=True)
    else:
        for sc in todo:
            v = judge(sc, cfg, planner=args.planner, time_limit_ms=args.time_limit_ms)
            rec = _record(v, sc, score=args.score)
            records.append(rec)
            if not args.quiet and not args.json:
                _print_row(rec, score=args.score)
            if args.view or args.spawn:
                dx, dy, e = place[sc.name]
                render(v, dx=dx, dy=dy, ext=e)

    if args.score:
        import json

        scores = [r["world"] for r in records]
        if args.json:

            def finite(o: Any) -> Any:
                if isinstance(o, float):
                    return o if math.isfinite(o) else None
                if isinstance(o, dict):
                    return {k: finite(x) for k, x in o.items()}
                if isinstance(o, list):
                    return [finite(x) for x in o]
                return o

            print(json.dumps(finite({"summary": summarize(scores), "worlds": records})))
        else:
            print(json.dumps(summarize(scores)))

    if args.spawn:
        # The last burst of logs must reach the viewer before we exit.
        import rerun as rr

        time.sleep(1.5)
        rr.disconnect()


if __name__ == "__main__":
    main()
