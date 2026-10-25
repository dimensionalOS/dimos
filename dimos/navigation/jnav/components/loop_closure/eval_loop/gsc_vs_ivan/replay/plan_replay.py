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

"""MLS planner paths through a lap closure, saved to a replay db (then render_plan.py draws it).
Replays the recording through the reforming ray tracer + planner (reform_metrics.lockstep). At `--before` s
before the biggest closure, a goal is set at `--goal-frac` of the way around the lap (from the old keyframe the
lap returns to), so the only mapped route back runs through the lap's second half. From then until `--after` s
past the closure the planner replans as the module does: after every scan's map update and after every warp. The goal and the first path are held
through every warp (planner.apply_loop_closure warps them with the map).
Streams: planned_path (each replan), held_path (the first path, warped), goal, robot, surface (planner surface
snapshots), lidar_live (window scans).
usage: plan_replay.py <a.kf> <out.db> --db <world db> [--before 60] [--after 60] [--goal-frac 0.4] [--lean]
       (--lean skips the surface and lidar_live clouds: plans, goal and robot only, for gsc_plan.py)"""

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, "dimos/navigation/global_planner/mls_planner/utils")
import reform_metrics as rm

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

parser = argparse.ArgumentParser()
parser.add_argument("keyframes", type=Path)
parser.add_argument("out", type=Path)
parser.add_argument("--db", type=Path, required=True)
parser.add_argument("--before", type=float, default=60.0)
parser.add_argument("--after", type=float, default=60.0)
parser.add_argument("--goal-frac", type=float, default=0.4)
parser.add_argument("--stride", type=int, default=1)
parser.add_argument("--lean", action="store_true")
args = parser.parse_args()

keyframes = np.fromfile(args.keyframes, rm.KEYFRAME)
closures = rm.read_closures(Path(f"{args.keyframes}.closures"))


def post_xyz(records: np.ndarray) -> np.ndarray:
    return (
        R.from_quat(records["dq"].astype(float)).apply(records["xyz"].astype(float)) + records["dt"]
    )


shifts = [float(np.linalg.norm(post_xyz(r) - r["xyz"], axis=1).max()) for _, r in closures]
lap = int(np.argmax(shifts))
lap_ts, lap_records = closures[lap]
post = post_xyz(lap_records)
newest = int(np.argmax(lap_records["id"]))
old = lap_records["ts"] < lap_ts - 2 * args.before
returned_to = np.flatnonzero(old)[np.argmin(np.linalg.norm(post[old] - post[newest], axis=1))]
t_old = float(lap_records["ts"][returned_to])
goal_ts = t_old + args.goal_frac * (lap_ts - t_old)
print(
    f"lap closure at {lap_ts:.1f} (shift {shifts[lap]:.1f} m), returns to t={t_old:.1f}, goal kf t={goal_ts:.1f}",
    flush=True,
)
window = (lap_ts - args.before, lap_ts + args.after)

args.out.unlink(missing_ok=True)
out = SqliteStore(path=str(args.out))
out.__enter__()
streams = {
    name: out.stream(name, PointCloud2)
    for name in ("planned_path", "held_path", "goal", "surface", "lidar_live")
}
robot = out.stream("robot", Odometry)
info = {
    "lap_ts": lap_ts,
    "lap_shift_m": shifts[lap],
    "t_old": t_old,
    "goal_ts": goal_ts,
    "plans": [],
}

real_modules = rm.lockstep_modules
planners = []
held: list[tuple[float, float, float]] = []  # goal, then the first path's waypoints


class HeldPlanner:
    """The planner with the goal and first path passed through every warp as held points."""

    def __init__(self, planner):
        self._planner = planner

    def __getattr__(self, name):
        return getattr(self._planner, name)

    def apply_loop_closure(self, records, **k):
        global held
        result = self._planner.apply_loop_closure(records, held, **k)
        if result is not None and held:
            held = [tuple(map(float, p)) for p in result["held"]]
        return result


def modules():
    tracer, planner = real_modules()
    planners.append(planner)
    return tracer, HeldPlanner(planner)


rm.lockstep_modules = modules


def write(name: str, points: np.ndarray, ts: float) -> None:
    cloud = PointCloud2.from_numpy(np.asarray(points, np.float32).reshape(-1, 3), frame_id="map")
    cloud.ts = ts
    streams[name].append(cloud, ts=ts)


from dimos.robot.unitree.go2.nav_3d_config import mls_planner_config

base_drop = mls_planner_config.start_z_offset_m
last_surface = -1e18


def replan(ts: float, origin: np.ndarray, why: str) -> None:
    """As the module's maybe_replan: from the robot's base to the held goal."""
    global held, last_surface
    planner = planners[0]
    start = (float(origin[0]), float(origin[1]), float(origin[2]) - base_drop)
    if not held:
        k = int(np.searchsorted(keyframes["ts"], goal_ts))
        rotation, translation = offset_state["rotation"], offset_state["translation"]
        goal = rotation.apply(keyframes["raw"][k, :3]) + translation
        held = [(float(goal[0]), float(goal[1]), float(goal[2]) - base_drop)]
    path = planner.plan(start, held[0])
    if path is not None and len(held) == 1:
        held = held + [tuple(map(float, p)) for p in path]
    write("goal", np.array([held[0]]), ts)
    if len(held) > 1:
        write("held_path", np.array(held[1:]), ts)
    write("planned_path", path if path is not None else np.zeros((0, 3)), ts)
    if not args.lean and (why == "warp" or ts - last_surface >= 2.0):
        write("surface", np.asarray(planner.surface_map(), np.float32).reshape(-1, 3), ts)
        last_surface = ts
    length = (
        float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        if path is not None and len(path) > 1
        else None
    )
    info["plans"].append({"ts": ts, "why": why, "planned": path is not None, "length_m": length})
    if why == "warp" or len(info["plans"]) % 20 == 0:
        print(
            f"t={ts - lap_ts:+6.1f} s {why}: path {'none' if path is None else f'{length} m'}",
            flush=True,
        )


last_origin = np.zeros(3)


def on_scan(ts: float, points: np.ndarray, origin: np.ndarray, tracer) -> None:
    global last_origin
    if not window[0] <= ts <= window[1]:
        return
    last_origin = origin
    if not args.lean:
        write("lidar_live", points[::4], ts)
    robot.append(
        Odometry(
            ts=ts,
            frame_id="map",
            child_frame_id="robot",
            pose=Pose(*map(float, origin), 0, 0, 0, 1),
        ),
        ts=ts,
    )
    replan(ts, origin, "scan")


offset_state = {"rotation": R.identity(), "translation": np.zeros(3)}


def rm_offset():
    return offset_state["rotation"], offset_state["translation"]


def on_closure(ts: float, records: np.ndarray, tracer) -> None:
    # the correction in force after this closure, as lockstep computes it: newest keyframe's post * raw^-1
    n = records[np.argmax(records["id"])]
    delta = R.from_quat(n["dq"].astype(float))
    post_rotation = delta * R.from_quat(n["q"].astype(float))
    post_translation = delta.apply(n["xyz"].astype(float)) + n["dt"]
    k = int(n["id"])
    rotation = post_rotation * R.from_quat(keyframes["raw"][k, 3:]).inv()
    global last_origin
    # the robot's last position, taken from the old correction into the new one, so a warp replan starts where it is
    raw_origin = offset_state["rotation"].inv().apply(last_origin - offset_state["translation"])
    offset_state["rotation"], offset_state["translation"] = (
        rotation,
        post_translation - rotation.apply(keyframes["raw"][k, :3]),
    )
    last_origin = rotation.apply(raw_origin) + offset_state["translation"]
    if window[0] <= ts <= window[1] and held:
        robot.append(
            Odometry(
                ts=ts,
                frame_id="map",
                child_frame_id="robot",
                pose=Pose(*map(float, last_origin), 0, 0, 0, 1),
            ),
            ts=ts,
        )
        replan(ts, last_origin, "warp")


started = time.monotonic()
rm.lockstep(
    args.db,
    keyframes,
    closures,
    scan_stride=args.stride,
    with_planner=True,
    until_ts=window[1],
    on_scan=on_scan,
    on_closure=on_closure,
)
out.__exit__(None, None, None)
args.out.with_suffix(".plan.json").write_text(json.dumps(info, indent=1))
print(f"wrote {args.out} in {time.monotonic() - started:.0f} s", flush=True)
