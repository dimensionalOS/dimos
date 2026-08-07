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

"""Replay a real recording through the planner + precision annotation.

Validation against reality, not a benchmark: mem2 recordings carry real
Point-LIO clouds (sensor frame) and odometry; this transforms each scan
through its nearest odometry pose, estimates the LOCAL floor (the planner's
z-band assumes flat ground — on stairs only the neighbourhood is planar),
plans toward a carrot taken from the robot's own future track, and logs
everything to rerun with the same required-precision circles the referee
draws: radius = clearance hint, red at the floor, yellow governed, green
full speed.

  python -m dimos.navigation.motion.adapter.replay mid360_athens_stairs.db --spawn
  python -m dimos.navigation.motion.adapter.replay mid360_athens_stairs.db --save athens.rrd
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np

from dimos.navigation.motion.adapter.floor import estimate_floor
from dimos.navigation.motion.adapter.follower import path_clearance
from dimos.navigation.motion.planner.referee.geometry import AvoidanceConfig
from dimos.navigation.motion.planner.referee.planners.base import load as load_planner
from dimos.navigation.motion.planner.referee.scenarios import EMBODIMENTS, Scenario
from dimos.navigation.motion.planner.referee.types import PointCloud2 as RefereeCloud

FLOOR_BAND = (0.05, 0.45)  # the planner's z-band over the estimated local floor
FULL_SPEED_CLEAR = 0.35  # AvoidanceConfig.speed_clearance, circle cap


def _quat_mat(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("dataset", help="mem2 db name resolvable by get_data")
    ap.add_argument("--spawn", action="store_true", help="live rerun viewer")
    ap.add_argument("--save", default="replay.rrd", help="output rrd (default replay.rrd)")
    ap.add_argument("--every", type=float, default=3.0, help="seconds between planned samples")
    ap.add_argument("--scans", type=int, default=5, help="lidar scans accumulated per sample")
    ap.add_argument("--carrot", type=float, default=3.0, help="goal: metres of future track arc")
    ap.add_argument("--embodiment", default="go2")
    args = ap.parse_args()

    import rerun as rr

    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.utils.data import get_data

    store = SqliteStore(path=get_data(args.dataset))
    odom = [(o.ts, o.data) for o in store.streams["pointlio_odometry"]]
    scans = [(o.ts, o.data) for o in store.streams["pointlio_lidar"]]
    t0 = odom[0][0]
    ots = np.array([t for t, _ in odom])
    opos = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for _, m in odom])
    oquat = np.array(
        [
            [m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w]
            for _, m in odom
        ]
    )
    sts = np.array([t for t, _ in scans])

    rr.init("motion-replay", spawn=args.spawn)
    if not args.spawn:
        rr.save(args.save)
    rr.log(
        "odom/track",
        rr.LineStrips3D([opos], colors=[[255, 255, 255]], radii=0.01),
        static=True,
    )

    emb = EMBODIMENTS[args.embodiment]
    sc = Scenario("replay", [], goal=(0.0, 0.0), emb=emb)
    episode = load_planner("target")(sc, AvoidanceConfig())
    episode.reset()

    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(opos[:, :2], axis=0), axis=1))])
    sample_ts = np.arange(t0, ots[-1], args.every)
    for si, t in enumerate(sample_ts):
        k = int(np.searchsorted(ots, t))
        if k >= len(odom):
            break
        # sensor pose and heading (x-axis of mid360_link projected to the plane)
        R = _quat_mat(oquat[k])
        pos = opos[k]
        yaw = math.atan2(R[1, 0], R[0, 0])
        # accumulate scans around t, each through its own nearest odom pose
        j = int(np.searchsorted(sts, t))
        world_pts = []
        for jj in range(max(0, j - args.scans // 2), min(len(scans), j + (args.scans + 1) // 2)):
            m = int(np.clip(np.searchsorted(ots, sts[jj]), 0, len(odom) - 1))
            pts = scans[jj][1].points_f32().astype(np.float64)
            world_pts.append(pts @ _quat_mat(oquat[m]).T + opos[m])
        cloud = np.concatenate(world_pts) if world_pts else np.empty((0, 3))
        # local floor, by the same estimator the planner adapter anchors with
        # (adapter/floor.py) — stairs are only locally planar, which is exactly
        # the planner's operating assumption
        estimated = estimate_floor(cloud, (float(pos[0]), float(pos[1])))
        if estimated is None:
            continue
        floor = estimated
        shifted = cloud - np.array([0.0, 0.0, floor])
        pose2d = (float(pos[0]), float(pos[1]), yaw)
        # carrot: the robot's own future position, args.carrot of arc ahead
        g = int(np.searchsorted(arc, arc[k] + args.carrot))
        goal = (float(opos[min(g, len(opos) - 1)][0]), float(opos[min(g, len(opos) - 1)][1]))

        ref_cloud = RefereeCloud.from_numpy(shifted.astype(np.float32), frame_id="odom")
        started = time.process_time()
        path = episode.plan(ref_cloud, pose2d, goal)
        plan_ms = (time.process_time() - started) * 1e3

        rr.set_time("sample", sequence=si)
        band = shifted[(shifted[:, 2] > FLOOR_BAND[0]) & (shifted[:, 2] < FLOOR_BAND[1])]
        show = band + np.array([0.0, 0.0, floor])
        rr.log("cloud/band", rr.Points3D(show[::3], radii=0.01, colors=[[230, 120, 60]]))
        rest = shifted[(shifted[:, 2] <= FLOOR_BAND[0]) | (shifted[:, 2] >= FLOOR_BAND[1])]
        showr = rest + np.array([0.0, 0.0, floor])
        rr.log("cloud/rest", rr.Points3D(showr[::6], radii=0.004, colors=[[110, 110, 130]]))
        rr.log("robot", rr.Points3D([pos], radii=0.09, colors=[[255, 200, 0]]))
        rr.log(
            "goal", rr.Points3D([[goal[0], goal[1], pos[2]]], radii=0.07, colors=[[80, 220, 80]])
        )

        xy = np.array([[p.position.x, p.position.y] for p in path.poses]).reshape(-1, 2)
        line = np.column_stack([xy, np.full(len(xy), pos[2] + 0.03)])
        rr.log("plan/path", rr.LineStrips3D([line], colors=[[100, 160, 255]], radii=0.012))
        clear = path_clearance(xy, shifted.astype(np.float32), emb.width / 2.0)
        seg = np.linalg.norm(np.diff(xy, axis=0), axis=1) if len(xy) > 1 else np.zeros(1)
        arcs = np.concatenate([[0.0], np.cumsum(seg)])
        idx = np.unique(np.searchsorted(arcs, np.arange(0.0, arcs[-1] + 1e-9, 0.35)))
        a = np.linspace(0.0, 2 * math.pi, 33)
        circles, cols = [], []
        for i in idx:
            i = min(int(i), len(xy) - 1)
            r = float(min(max(clear[i], 0.02), FULL_SPEED_CLEAR))
            circles.append(
                np.column_stack(
                    [xy[i][0] + r * np.cos(a), xy[i][1] + r * np.sin(a), np.full(33, pos[2] + 0.02)]
                )
            )
            if clear[i] <= emb.precision:
                cols.append([255, 60, 60])
            elif clear[i] < FULL_SPEED_CLEAR:
                cols.append([255, 220, 60])
            else:
                cols.append([80, 220, 80])
        rr.log("plan/precision", rr.LineStrips3D(circles, colors=cols, radii=0.004))
        veto = " VETO/stub" if len(path.poses) < 2 else ""
        print(
            f"t={t - t0:6.1f}s floor={floor:+.2f} pts={len(cloud)} "
            f"plan={plan_ms:6.1f}ms wps={len(path.poses)}{veto}"
        )


if __name__ == "__main__":
    main()
