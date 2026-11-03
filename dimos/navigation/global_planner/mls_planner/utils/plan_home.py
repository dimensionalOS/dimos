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

"""The MLS planner replanning home (to the robot's pose at --goal-at, default its first) over a whole GSC run, as the report viewer's plan JSON.
Replays the recording through the reforming ray tracer + planner (reform_metrics.lockstep) with GSC's closures applied
when GSC adopted them. The goal is held and warped with the map (as the module does); the planner replans as the module
does, after every live region update and after every warp, with replan. Every --fresh-every s, and every plan
within --fresh-after s of a closure, the same plan is also made on a planner rebuilt from scratch from the warped
planner's voxels (update_global_map), for comparison.
Per plan: ts, why (scan / warp), route, length, the fresh route's length, backtrack_m (the longest stretch of the route
that comes back within 0.3 m of where it was), full (replan found a route; with none it returns no path),
bad_legs (interior legs the chord walk calls untraversable), end_leg_m, plan wall and process CPU ms (replan only).
Run from the mls_reform worktree with its venv:
usage: plan_home.py <gsc run dir> <world db> <out json> [--fresh-every 2] [--fresh-after 4] [--threads 4] [--until s] [--at-dump] [--goal-at s]"""

import argparse
import json
from pathlib import Path
import resource
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).parent))
import reform_metrics as rm

parser = argparse.ArgumentParser()
parser.add_argument("run", type=Path)
parser.add_argument("db", type=Path)
parser.add_argument("out", type=Path)
parser.add_argument("--fresh-every", type=float, default=2.0)
parser.add_argument("--fresh-after", type=float, default=4.0)
parser.add_argument(
    "--big-shift",
    type=float,
    default=0.3,
    help="a closure moving a keyframe this far (m) gets fresh checks",
)
parser.add_argument("--threads", type=int, default=0)
parser.add_argument(
    "--warp-min-shift",
    type=float,
    default=0.0,
    help="a warp moving the map less than this (m) replans like a scan (subject to --plan-period)",
)
parser.add_argument(
    "--plan-period",
    type=float,
    default=0.0,
    help="replan on scans at most this often (s); warps always replan",
)
parser.add_argument(
    "--goal-at", type=float, default=0.0, help="the goal is the robot's pose this many s in"
)
parser.add_argument(
    "--at-dump",
    action="store_true",
    help="apply each closure at its event-dump time, not its adoption",
)
parser.add_argument(
    "--until", type=float, default=None, help="stop after this many s of recording (a quick look)"
)
parser.add_argument(
    "--lidar-stream",
    default="lidar",
    help="a cloud not in frame world is taken as in the odometry's frame",
)
parser.add_argument("--odom-stream", default="odom")
args = parser.parse_args()

import itertools

from dimos.navigation.global_planner.mls_planner.mls_reform_planner_native import (
    MLSReformPlannerNativeConfig,
)
from dimos.robot.unitree.go2.nav_3d_config import mls_planner_config

live = MLSReformPlannerNativeConfig()
threads = args.threads or live.worker_threads
base_drop = mls_planner_config.start_z_offset_m
goal_tolerance = mls_planner_config.goal_tolerance
keyframes = np.fromfile(args.run / "a.kf", rm.KEYFRAME)
closures = rm.read_closures(args.run / "a.kf.closures")
t0 = float(keyframes["ts"][0])
dumped_ts = [ts for ts, _ in closures]
# pgo-eval dumps pose changes only at keyframes, while the live module publishes a solve the moment it adopts it:
# an event that follows an adoption (solve.jsonl) is moved back to the adopted scan's time
if not args.at_dump and (args.run / "solve.jsonl").exists():
    from dimos.memory.store.sqlite import SqliteStore

    with SqliteStore(path=str(args.db)) as store:
        scan_ts = np.array([o.ts for o in getattr(store.streams, args.lidar_stream).order_by("ts")])
    adoptions = []
    for line in (args.run / "solve.jsonl").read_text().splitlines():
        solve = json.loads(line)
        scan = solve.get("adopted_scan")
        adoptions.append(
            float(scan_ts[scan])
            if scan is not None and scan < len(scan_ts)
            else solve["adopt_time"]
        )
    previous = -np.inf
    for i, (ts, records) in enumerate(closures):
        earlier = [a for a in adoptions if previous < a <= ts]
        closures[i] = (min(earlier) if earlier else ts, records)
        previous = ts

real_planner_class = rm.MLSPlanner
planner_kwargs: dict = {}


def planner_capped(*a, **k):
    k["worker_threads"] = threads
    planner_kwargs.update(k)
    return real_planner_class(*a, **k)


rm.MLSPlanner = planner_capped


def cpu_s() -> float:
    usage = resource.getrusage(resource.RUSAGE_SELF)
    return usage.ru_utime + usage.ru_stime


def length(path: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) if len(path) > 1 else 0.0


def backtrack_m(path: np.ndarray, near_m: float = 0.3, step_m: float = 0.1) -> float:
    """The longest stretch of route between two of its points within near_m of each other (0 for a route that never comes back)."""
    if len(path) < 2:
        return 0.0
    points, along = [path[0]], [0.0]
    for u, v in itertools.pairwise(path):
        d = float(np.linalg.norm(v - u))
        n = max(1, int(d / step_m))
        for k in range(1, n + 1):
            points.append(u + (v - u) * k / n)
            along.append(along[-1] + d / n)
    points, along = np.asarray(points), np.asarray(along)
    from scipy.spatial import cKDTree

    pairs = cKDTree(points).query_pairs(near_m, output_type="ndarray")
    if len(pairs) == 0:
        return 0.0
    gaps = np.abs(along[pairs[:, 1]] - along[pairs[:, 0]])
    # two points on one straight stretch are near_m apart along it; anything longer came back
    return float(max(gaps.max() - near_m * 2, 0.0)) if gaps.max() > near_m * 3 else 0.0


planners = []
held: list[tuple[float, float, float]] = []
state = {"origin": None, "last_fresh": -1e18, "closure_ts": [], "applied": 0}
plans: list[dict] = []
closure_rows: list[dict] = []


class HomePlanner:
    """The lockstep planner, replanning home after every live region update, the goal passed through every warp."""

    def __init__(self, planner):
        self._planner = planner

    def __getattr__(self, name):
        return getattr(self._planner, name)

    def update_region(self, *a, **k):
        self._planner.update_region(*a, **k)
        ts = a[-1] if len(a) >= 7 else k.get("stamp")
        replan(float(ts), "scan")

    def apply_loop_closure(self, records, **k):
        global held
        started = time.perf_counter()
        result = self._planner.apply_loop_closure(records, held, **k)
        if result is not None and held:
            held = [tuple(map(float, p)) for p in result["held"]]
        state["warp_ms"] = (time.perf_counter() - started) * 1e3
        state["warped"] = result is not None
        return result


def modules():
    tracer, planner = real_modules()
    planners.append(planner)
    return tracer, HomePlanner(planner)


real_modules = rm.lockstep_modules
rm.lockstep_modules = modules


def fresh_plan(start, goal):
    """The same plan on a planner rebuilt from scratch from the warped planner's voxels."""
    kwargs = dict(planner_kwargs, voxel_times=False)
    rebuilt = real_planner_class(**kwargs)
    begun, cpu = time.perf_counter(), cpu_s()
    rebuilt.update_global_map(planners[0].voxel_map())
    path = rebuilt.plan(start, goal)
    return path, (time.perf_counter() - begun) * 1e3, (cpu_s() - cpu) * 1e3


def replan(ts: float, why: str) -> None:
    global held
    origin = state["origin"]
    if origin is None:
        return
    start = (float(origin[0]), float(origin[1]), float(origin[2]) - base_drop)
    if why == "scan" and ts - state.get("last_plan", -1e18) < args.plan_period:
        return
    state["last_plan"] = ts
    if not held:
        if ts - t0 < args.goal_at:
            return
        held = [start]
    goal = held[0]
    planner = planners[0]
    begun, cpu = time.perf_counter(), cpu_s()
    path = np.asarray(planner.replan(start, goal), np.float64).reshape(-1, 3)
    plan_ms, cpu_ms = (time.perf_counter() - begun) * 1e3, (cpu_s() - cpu) * 1e3
    # whether a full route exists now (replan falls back to the cached route when none does)
    full = planner.plan(start, goal) is not None
    home_m = float(np.linalg.norm(np.subtract(start[:2], goal[:2])))
    row = {
        "ts": round(ts - t0, 3),
        "why": why,
        "goal": [round(goal[0], 3), round(goal[1], 3)],
        "robot": [round(start[0], 3), round(start[1], 3)],
        "path": np.round(path[:, :2], 2).tolist(),
        "len": round(length(path), 3),
        "full": full,
        # legs the planner's own chord walk calls untraversable (must be none)
        # interior legs (cell to cell) the planner's chord walk calls untraversable; the two end legs join the
        # raw robot and goal poses to their snapped cells (within the planner's snap radius) and may cross
        # floor the sensor never saw, so they are measured instead (end_leg_m)
        "bad_legs": int(
            sum(not ok for ok in planner.legs_traversable(path[1:-1].astype(np.float32)))
        )
        if len(path) > 3
        else 0,
        "end_leg_m": round(
            max(
                float(np.linalg.norm(path[1, :2] - path[0, :2])),
                float(np.linalg.norm(path[-1, :2] - path[-2, :2])),
            ),
            2,
        )
        if len(path) > 1
        else 0.0,
        "backtrack_m": round(backtrack_m(path), 2),
        "home_m": round(home_m, 2),
        "plan_ms": round(plan_ms, 2),
        "cpu_ms": round(cpu_ms, 2),
    }
    since_closure = ts - max(state["closure_ts"], default=-1e18)
    big_warp = why == "warp" and since_closure == 0
    if home_m > 1.0 and (
        big_warp
        or since_closure <= args.fresh_after
        or ts - state["last_fresh"] >= args.fresh_every
    ):
        state["last_fresh"] = ts
        fresh, fresh_ms, fresh_cpu_ms = fresh_plan(start, goal)
        row["fresh_len"] = round(length(np.asarray(fresh)), 3) if fresh is not None else None
        row["fresh_ms"], row["fresh_cpu_ms"] = round(fresh_ms, 1), round(fresh_cpu_ms, 1)
        if since_closure <= args.fresh_after:
            row["fresh_path"] = (
                np.round(np.asarray(fresh)[:, :2], 2).tolist() if fresh is not None else []
            )
    plans.append(row)
    if why == "warp" or len(plans) % 200 == 0:
        print(
            f"t={ts - t0:7.1f} {why}: len {row['len']:.1f} fresh {row.get('fresh_len')} back {row['backtrack_m']} "
            f"full {row['full']} {plan_ms:.0f} ms",
            flush=True,
        )


def on_scan(ts: float, points: np.ndarray, origin: np.ndarray, tracer) -> None:
    state["origin"] = origin


offset = {"rotation": R.identity(), "translation": np.zeros(3)}


def on_closure(ts: float, records: np.ndarray, tracer) -> None:
    # the correction in force after this closure, as lockstep computes it: newest keyframe's post * raw^-1
    n = records[np.argmax(records["id"])]
    delta = R.from_quat(n["dq"].astype(float))
    post_rotation = delta * R.from_quat(n["q"].astype(float))
    post_translation = delta.apply(n["xyz"].astype(float)) + n["dt"]
    k = int(n["id"])
    rotation = post_rotation * R.from_quat(keyframes["raw"][k, 3:]).inv()
    translation = post_translation - rotation.apply(keyframes["raw"][k, :3])
    post = (
        R.from_quat(records["dq"].astype(float)).apply(records["xyz"].astype(float)) + records["dt"]
    )
    shift = float(np.linalg.norm(post - records["xyz"], axis=1).max())
    first = int(np.argmin(records["id"]))
    if state["origin"] is not None:
        raw = offset["rotation"].inv().apply(state["origin"] - offset["translation"])
        state["origin"] = rotation.apply(raw) + translation
    offset["rotation"], offset["translation"] = rotation, translation
    if (
        shift >= args.big_shift
    ):  # every keyframe makes an event; the fresh checks follow the ones that move the map
        state["closure_ts"].append(ts)
    closure_rows.append(
        {
            "ts": round(ts - t0, 3),
            "dumped_ts": round(dumped_ts[len(closure_rows)] - t0, 3),
            "shift_m": round(shift, 3),
            "warped": state.get("warped", False),
            "warp_ms": round(state.get("warp_ms", 0.0), 1),
            # where GSC puts the first keyframe after this closure, against the planner's warped goal
            "gsc_start": [round(float(v), 3) for v in post[first, :2]],
            "goal_after": [round(held[0][0], 3), round(held[0][1], 3)] if held else None,
        }
    )
    replan(ts, "warp" if shift >= args.warp_min_shift else "scan")


started = time.monotonic()
until_ts = t0 + args.until if args.until else None
rm.lockstep(
    args.db,
    keyframes,
    closures,
    with_planner=True,
    until_ts=until_ts,
    on_scan=on_scan,
    on_closure=on_closure,
    lidar_stream=args.lidar_stream,
    odom_stream=args.odom_stream,
)
wall_s = time.monotonic() - started
result = {
    "method": "gsc",
    "frame": "GSC corrected",
    "t0": t0,
    "goal": f"the robot's pose at {args.goal_at:.0f} s, held and warped with the map",
    "planner_threads": threads,
    "wall_s": round(wall_s, 1),
    "closures": closure_rows,
    "states": plans,
}
args.out.parent.mkdir(parents=True, exist_ok=True)
args.out.write_text(json.dumps(result, separators=(",", ":")))
print(
    f"wrote {args.out}: {len(plans)} plans, {len(closure_rows)} closures in {wall_s:.0f} s",
    flush=True,
)
