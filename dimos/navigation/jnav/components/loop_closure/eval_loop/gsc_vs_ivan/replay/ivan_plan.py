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

"""MLS planner paths on Ivan's map through the same window and goal as gsc_plan.py, as the report viewer's plan JSON.
Ivan's map is rebuilt as PGOMap builds it: every scan into a stamped, column-carving VoxelGrid, and when a closure
state (a.kf.closures) is newer than the last rebuild and the rebuild cooldown has passed, every voxel re-placed by
that state's keyframe corrections, anchored at its latest keyframe (so current data stays at raw odom). Ivan has no
goal warp: the goal stays at fixed map coordinates. The window, the lap and the goal keyframe come from the GSC run,
picked as plan_replay.py picks them (the goal is the goal keyframe's raw pose under the correction in force when it
is set, which in Ivan's robot-anchored map is the raw pose itself). In the window the planner takes the whole map at
its start and after each rebuild, the scan's region of the map after every other scan, and replans after each.
Output x/y are in Ivan's a.kf corrected frame (the map frame moved by the latest rebuild's anchor), ts in s since
Ivan's first keyframe: {"states": [{"ts", "goal", "path", "robot", "why"}, ...]}.
Every scan is fed (Ivan's live run dropped the ones that arrived while it was busy).
Run from a dimos worktree with dimos_mls_planner built:
usage: ivan_plan.py <ivan run dir> <gsc run dir> <world db> <out json> [--threads N] [--before 60] [--after 60]
       [--goal-frac 0.4]
e.g. (ivan_final runs, from the mls_reform worktree):
  G=~/.dimos/recordings/gsc_runs; for r in huge small_mid gir1 gir2 park1 athens office555; do nice -n 10 env -u PYTHONPATH \
  .venv/bin/python <this dir>/ivan_plan.py $G/ivan_final/${r}_live $G/2026-10-25_${r}_final $G/world_dbs/$r.db \
  ~/.dimos/recordings/report_build/plans/$r/ivan_plan.json; done"""

import argparse
import json
from pathlib import Path
import sys
import time

from dimos_mls_planner import MLSPlanner
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

from dimos.mapping.voxels.grid import VoxelGrid
from dimos.memory.store.sqlite import SqliteStore
from dimos.navigation.go2.loop_closure.module import PGOVoxelMapperConfig
from dimos.robot.unitree.go2.nav_3d_config import mls_planner_config

sys.path.insert(0, "dimos/navigation/global_planner/mls_planner/utils")
import reform_metrics as rm

parser = argparse.ArgumentParser()
parser.add_argument("ivan", type=Path)
parser.add_argument("gsc", type=Path)
parser.add_argument("db", type=Path)
parser.add_argument("out", type=Path)
parser.add_argument("--threads", type=int, default=8)
parser.add_argument("--before", type=float, default=60.0)
parser.add_argument("--after", type=float, default=60.0)
parser.add_argument("--goal-frac", type=float, default=0.4)
args = parser.parse_args()
args.out.parent.mkdir(parents=True, exist_ok=True)


def post_xyz(records: np.ndarray) -> np.ndarray:
    return (
        R.from_quat(records["dq"].astype(float)).apply(records["xyz"].astype(float)) + records["dt"]
    )


# the lap, window and goal keyframe exactly as plan_replay.py picks them from the GSC run
gsc_keyframes = np.fromfile(args.gsc / "a.kf", rm.KEYFRAME)
gsc_closures = rm.read_closures(args.gsc / "a.kf.closures")
shifts = [float(np.linalg.norm(post_xyz(r) - r["xyz"], axis=1).max()) for _, r in gsc_closures]
lap = int(np.argmax(shifts))
lap_ts, lap_records = gsc_closures[lap]
post = post_xyz(lap_records)
newest = int(np.argmax(lap_records["id"]))
old = lap_records["ts"] < lap_ts - 2 * args.before
returned_to = np.flatnonzero(old)[np.argmin(np.linalg.norm(post[old] - post[newest], axis=1))]
t_old = float(lap_records["ts"][returned_to])
goal_ts = t_old + args.goal_frac * (lap_ts - t_old)
window = (lap_ts - args.before, lap_ts + args.after)
del gsc_closures
base_drop = mls_planner_config.start_z_offset_m
raw_goal = gsc_keyframes["raw"][int(np.searchsorted(gsc_keyframes["ts"], goal_ts)), :3]
goal = (float(raw_goal[0]), float(raw_goal[1]), float(raw_goal[2]) - base_drop)

ivan_keyframes = np.fromfile(args.ivan / "a.kf", rm.KEYFRAME)
t0 = float(ivan_keyframes["ts"][0])
states = rm.read_closures(args.ivan / "a.kf.closures")
state_ts = np.array([ts for ts, _ in states])
# a state's largest keyframe drift (optimized vs raw); what a rebuild moves is "moved_m" below
state_shift = [float(np.linalg.norm(post_xyz(r) - r["xyz"], axis=1).max()) for _, r in states]
print(
    f"GSC lap closure at {lap_ts - t0:.1f} s (shift {shifts[lap]:.1f} m), goal kf at {goal_ts - t0:.1f} s; "
    f"Ivan states in the window: "
    + ", ".join(
        f"{ts - t0:.1f} s ({s:.2f} m)"
        for ts, s in zip(state_ts, state_shift, strict=True)
        if window[0] <= ts <= window[1]
    ),
    flush=True,
)


class Drifts:
    """A closure state's per-keyframe drift (raw -> corrected), interpolated by time as PoseGraph._corrections."""

    def __init__(self, records: np.ndarray) -> None:
        ts, first = np.unique(records["ts"], return_index=True)
        if len(ts) == 1:
            ts, first = np.array([ts[0], ts[0] + 1e-6]), np.array([first[0], first[0]])
        self.ts = ts
        self.slerp = Slerp(ts, R.from_quat(records["dq"][first].astype(float)))
        self.t = records["dt"][first].astype(float)
        self.now_rotation, self.now_translation = self.at(np.array([np.inf]))

    def at(self, stamps: np.ndarray) -> tuple[R, np.ndarray]:
        clipped = np.clip(stamps, self.ts[0], self.ts[-1])
        return self.slerp(clipped), np.stack(
            [np.interp(clipped, self.ts, self.t[:, i]) for i in range(3)], axis=1
        )

    def place(self, cloud):
        """PoseGraph.place: each point by its stamp's correction, anchored at the latest keyframe."""
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        points = cloud.points_f32()
        stamps = cloud.stamps_f64()
        unique, inverse = np.unique(stamps, return_inverse=True)
        rotation, translation = self.at(unique)
        anchor = self.now_rotation[0].inv()
        rotation, translation = (
            anchor * rotation,
            anchor.apply(translation - self.now_translation[0]),
        )
        return PointCloud2.from_numpy(
            rotation[inverse].apply(points.astype(np.float64)) + translation[inverse],
            frame_id=cloud.frame_id,
            timestamp=cloud.ts,
        )

    def anchored(self, xyz: np.ndarray, stamps: np.ndarray) -> np.ndarray:
        """Raw points taken at `stamps` where this state's rebuild puts them in the map."""
        rotation, translation = self.at(stamps)
        corrected = rotation.apply(np.asarray(xyz, float)) + translation
        return self.now_rotation[0].inv().apply(corrected - self.now_translation[0])

    def to_corrected(self, xyz: np.ndarray) -> np.ndarray:
        """Robot-anchored map coordinates into the a.kf corrected frame."""
        return (
            self.now_rotation[0].apply(np.asarray(xyz, float).reshape(-1, 3))
            + self.now_translation[0]
        )


cfg = PGOVoxelMapperConfig()
grid = VoxelGrid(
    voxel_size=cfg.voxel_size, frame_id=cfg.frame_id, stamped=True, show_startup_log=False
)
pc = mls_planner_config
planner = MLSPlanner(
    voxel_size=pc.voxel_size,
    robot_height=pc.robot_height,
    max_overhead_m=pc.max_overhead_m,
    surface_closing_radius=pc.surface_closing_radius,
    node_spacing_m=pc.node_spacing_m,
    wall_clearance_m=pc.wall_clearance_m,
    wall_buffer_m=pc.wall_buffer_m,
    wall_buffer_weight=pc.wall_buffer_weight,
    step_threshold_m=pc.step_threshold_m,
    step_penalty_weight=pc.step_penalty_weight,
    worker_threads=args.threads,
)

with SqliteStore(path=str(args.db)) as store:
    odom = np.array(
        [
            (o.ts, o.data.position.x, o.data.position.y, o.data.position.z)
            for o in store.streams.odom.order_by("ts")
        ]
    )
odom = odom[np.unique(odom[:, 0], return_index=True)[1]]


def robot_at(ts: float) -> np.ndarray:
    ts = float(np.clip(ts, odom[0, 0], odom[-1, 0]))
    return np.array([np.interp(ts, odom[:, 0], odom[:, k]) for k in (1, 2, 3)])


def r3(values) -> list[float]:
    return [round(float(v), 2) for v in values]


placed, placed_drifts, last_rebuild = -1, None, -np.inf
fed_whole = False
out_states, rebuilds = [], []
started = time.monotonic()
with SqliteStore(path=str(args.db)) as store:
    for obs in store.streams.lidar.order_by("ts"):
        if obs.ts > window[1]:
            break
        cloud = obs.data
        grid.add_frame(cloud)
        latest = int(np.searchsorted(state_ts, obs.ts, side="right")) - 1
        rebuilt = latest > placed and obs.ts - last_rebuild >= cfg.rebuild_cooldown_s
        if rebuilt:
            new_drifts = Drifts(states[latest][1])
            kf_raw, kf_ts = states[latest][1]["xyz"].astype(float), states[latest][1]["ts"]
            # how far the rebuild moved the map, at the keyframes: the planner's view of this closure
            moved = (
                float(
                    np.linalg.norm(
                        new_drifts.anchored(kf_raw, kf_ts) - placed_drifts.anchored(kf_raw, kf_ts),
                        axis=1,
                    ).max()
                )
                if placed_drifts is not None
                else 0.0
            )
            placed_drifts = new_drifts
            grid.reproject(placed_drifts.place)
            placed, last_rebuild = latest, obs.ts
        if obs.ts < window[0]:
            continue
        origin = robot_at(obs.ts)
        voxels = np.asarray(grid._impl.points(), np.float32)
        if rebuilt or not fed_whole:
            planner.clear()
            planner.update_global_map(voxels)
            fed_whole = True
            why = "rebuild" if rebuilt else "start"
            if rebuilt:
                rebuilds.append(
                    {
                        "ts": round(obs.ts - t0, 3),
                        "state": latest,
                        "drift_m": state_shift[latest],
                        "moved_m": moved,
                    }
                )
        elif not len(cloud.points_f32()):
            why = "scan"
        else:
            # the scan's region of the map: everything the new frame could have added or carved
            points = cloud.points_f32()
            radius = (
                float(np.linalg.norm(points[:, :2] - origin[:2], axis=1).max()) + cfg.voxel_size
            )
            # carving clears a touched column at every height, so the region spans the map's whole height there
            inside = np.linalg.norm(voxels[:, :2] - origin[:2], axis=1) <= radius
            if not inside.any():
                continue
            z_min = float(voxels[inside, 2].min()) - cfg.voxel_size
            z_max = float(voxels[inside, 2].max()) + cfg.voxel_size
            planner.update_region(
                voxels[inside],
                (float(origin[0]), float(origin[1])),
                radius,
                z_min,
                z_max,
                float(origin[2]),
            )
            why = "scan"
        start = (float(origin[0]), float(origin[1]), float(origin[2]) - base_drop)
        path = planner.plan(start, goal)
        path = np.zeros((0, 3)) if path is None else np.asarray(path, float)
        to_corrected = (
            placed_drifts.to_corrected
            if placed_drifts is not None
            else lambda x: np.asarray(x).reshape(-1, 3)
        )
        out_states.append(
            {
                "ts": round(obs.ts - t0, 3),
                "goal": r3(to_corrected(goal)[0, :2]),
                "path": [r3(p) for p in to_corrected(path)[:, :2]] if len(path) else [],
                "robot": r3(to_corrected(origin)[0, :2]),
                "why": why,
            }
        )
        if why != "scan" or len(out_states) % 100 == 0:
            length = (
                float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
                if len(path) > 1
                else None
            )
            print(
                f"t={obs.ts - lap_ts:+6.1f} s {why}: {len(voxels)} voxels, path "
                f"{'none' if length is None else f'{length:.1f} m'} ({time.monotonic() - started:.0f} s)",
                flush=True,
            )

result = {
    "method": "ivan",
    "frame": "Ivan a.kf corrected (PGO optimized); planned in the robot-anchored map, moved by each rebuild's anchor",
    "t0": t0,
    "lap_ts": round(lap_ts - t0, 3),
    "lap_shift_m": shifts[lap],
    "goal_kf_ts": round(goal_ts - t0, 3),
    "rebuilds": rebuilds,
    "states": out_states,
}
args.out.write_text(json.dumps(result))
planned = sum(1 for s in out_states if s["path"])
print(
    f"wrote {args.out}: {len(out_states)} states, {planned} with a path, {len(rebuilds)} rebuilds in the window, "
    f"{time.monotonic() - started:.0f} s",
    flush=True,
)
grid.dispose()
