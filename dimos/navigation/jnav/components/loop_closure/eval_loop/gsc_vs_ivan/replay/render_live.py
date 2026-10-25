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

"""Top-down video of a live plan run (tap_live.py db): the planner's surface map, its published path, the goal and
the robot (GSC's corrected odometry), one frame per `step` seconds of data from `start` s before the goal.
The planner publishes its surface as 4 m tiles (newest tile per cell wins) and stamps them, its path and the goal on
the wall clock, while GSC uses the recording clock; `--offset` (wall - recording, from live_plan.json) aligns them.
usage: render_live.py <live.db> <out.mp4> <live_plan.json> [step_s] [lead_s] [title] [closure_wall_ts]"""

import json
from pathlib import Path
import subprocess
import sys

import matplotlib
import numpy as np
from scipy.spatial.transform import Rotation as R

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from dimos.memory.store.sqlite import SqliteStore

sys.path.insert(0, "dimos/navigation/global_planner/mls_planner/utils")
import reform_metrics as rm

db, out = Path(sys.argv[1]), Path(sys.argv[2])
first = json.loads(Path(sys.argv[3]).read_text())["first"]
offset = first["wall"] - first["ts"]
step = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
lead = float(sys.argv[5]) if len(sys.argv) > 5 else 15.0
title = sys.argv[6] if len(sys.argv) > 6 else db.stem
TILE = 4.0
closure_wall = (
    float(sys.argv[7]) if len(sys.argv) > 7 else None
)  # wall time of the lap closure, from the run log

with SqliteStore(path=str(db)) as store:
    surfaces = (
        [
            (o.ts - offset, np.asarray(o.data.points_f32()).reshape(-1, 3))
            for o in store.streams.surface_map.order_by("ts")
        ]
        if "surface_map" in store.list_streams()
        else []
    )
    paths = [
        (
            o.ts - offset,
            np.array([[p.position.x, p.position.y] for p in o.data.poses]).reshape(-1, 2),
        )
        for o in store.streams.path.order_by("ts")
    ]
    goals = [(o.ts - offset, (o.data.x, o.data.y)) for o in store.streams.goal.order_by("ts")]
    # the ray tracer's whole map (recording clock), downsampled for drawing
    lidar_maps = []
    for o in store.streams.global_map.order_by("ts"):
        m = np.asarray(o.data.points_f32()).reshape(-1, 3)
        keys = np.floor(m / 0.3).astype(np.int64)
        _, first_of = np.unique(keys, axis=0, return_index=True)
        lidar_maps.append((o.ts, m[first_of]))
    robot = np.array(
        [
            (o.ts, o.data.pose.position.x, o.data.pose.position.y)
            for o in store.streams.corrected_odometry.order_by("ts")
        ]
    )
    shifts = []
    for o in store.streams.loop_closure_records.order_by("ts"):
        records = np.frombuffer(o.data.data, rm.CLOSURE_RECORD) if hasattr(o.data, "data") else None
        if records is not None and len(records):
            post = (
                R.from_quat(records["dq"].astype(float)).apply(records["xyz"].astype(float))
                + records["dt"]
            )
            shifts.append((o.ts, float(np.linalg.norm(post - records["xyz"], axis=1).max())))
print(
    f"{len(surfaces)} surfaces, {len(paths)} paths, {len(goals)} goals, closures {[(round(t - goals[0][0]), round(s, 1)) for t, s in shifts if s > 0.5] if goals else shifts[:5]}",
    flush=True,
)


def latest(items, t):
    i = np.searchsorted([ts for ts, _ in items], t, side="right") - 1
    return items[i][1] if i >= 0 else None


t_goal = goals[0][0]
t_end = max(surfaces[-1][0] if surfaces else 0, paths[-1][0] if paths else 0, robot[-1, 0])
window_robot = robot[(robot[:, 0] >= t_goal - lead) & (robot[:, 0] <= t_end), 1:]
everything = np.concatenate(
    [window_robot, np.array([g for _, g in goals])] + [p for _, p in paths if len(p)]
)
everything = everything[np.isfinite(everything).all(axis=1)]
lo, hi = everything.min(0) - 25, everything.max(0) + 25
frames = out.with_suffix("")
frames.mkdir(parents=True, exist_ok=True)
big = [t for t, s in shifts if s > 1.0] or ([closure_wall - offset] if closure_wall else [])
tiles: dict[tuple, np.ndarray] = {}
next_surface = 0
before_closure = None
for n, t in enumerate(
    list(np.arange(t_goal - lead, t_end + step, step)) + [t_end + step] * (4 * 8)
):
    while next_surface < len(surfaces) and surfaces[next_surface][0] <= t:
        tile = surfaces[next_surface][1]
        if len(tile):
            tiles[
                tuple(np.floor((tile[:, :2].min(0) + tile[:, :2].max(0)) / 2 / TILE).astype(int))
            ] = tile
        next_surface += 1
    fig, ax = plt.subplots(figsize=(11, 8), dpi=100, facecolor="black")
    ax.set_facecolor("black")
    lidar = latest(lidar_maps, t)
    if lidar is not None and len(lidar):
        ax.scatter(
            lidar[:, 0],
            lidar[:, 1],
            s=0.15,
            c=lidar[:, 2],
            cmap="viridis",
            vmin=np.percentile(lidar[:, 2], 2),
            vmax=np.percentile(lidar[:, 2], 98),
            linewidths=0,
        )
    s = None if lidar_maps else (np.concatenate(list(tiles.values())) if tiles else None)
    if s is not None and len(s):
        ax.scatter(
            s[:, 0],
            s[:, 1],
            s=0.3,
            c=s[:, 2],
            cmap="viridis",
            vmin=np.percentile(s[:, 2], 2),
            vmax=np.percentile(s[:, 2], 98),
            linewidths=0,
        )
    p = latest(paths, t)
    if big and t < big[0] and p is not None and len(p) > 10:
        before_closure = p
    if big and t >= big[0] and before_closure is not None:
        ax.plot(
            before_closure[:, 0],
            before_closure[:, 1],
            color="orange",
            lw=1.5,
            ls="--",
            label="last full path before the closure",
        )
    if p is not None and len(p) and t >= t_goal:
        ax.plot(p[:, 0], p[:, 1], color="cyan", lw=2.2, label="MLS planner path (as published)")
    g = latest(goals, t)
    if g is not None:
        ax.plot(*g, "*", color="yellow", ms=16, label="goal (sent once)")
    seen = robot[robot[:, 0] <= t]
    if len(seen):
        ax.plot(seen[-300:, 1], seen[-300:, 2], color="white", lw=0.6, alpha=0.6)
        ax.plot(seen[-1, 1], seen[-1, 2], "o", color="lime", ms=8, label="robot (GSC corrected)")
    done = [b for b in big if b <= t]
    note = f"   LOOP CLOSED {t - done[0]:.0f} s ago" if done else ""
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(
        f"{title}   t = {t - t_goal:+6.1f} s from goal{note}",
        color="orange" if done else "white",
        fontsize=11,
    )
    ax.legend(loc="upper left", facecolor="black", edgecolor="gray", labelcolor="white", fontsize=8)
    fig.tight_layout()
    fig.savefig(frames / f"{n:05d}.png", facecolor="black")
    plt.close(fig)
subprocess.run(
    [
        "ffmpeg",
        "-y",
        "-loglevel",
        "error",
        "-framerate",
        "8",
        "-i",
        str(frames / "%05d.png"),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-vf",
        "pad=ceil(iw/2)*2:ceil(ih/2)*2",
        str(out),
    ],
    check=True,
)
subprocess.run(["rm", "-rf", str(frames)], check=True)
print("wrote", out, flush=True)
