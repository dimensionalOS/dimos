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

"""Top-down video of a plan replay db (plan_replay.py): planner surface, the current plan, the first plan as the
warps carried it, the held goal and the robot, one frame per `step` seconds of data.
usage: render_plan.py <plan_replay.db> <out.mp4> [step_s] [title]"""

import json
from pathlib import Path
import subprocess
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from dimos.memory.store.sqlite import SqliteStore

db, out = Path(sys.argv[1]), Path(sys.argv[2])
step = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
title = sys.argv[4] if len(sys.argv) > 4 else db.stem
info = json.loads(db.with_suffix(".plan.json").read_text())
lap_ts = info["lap_ts"]


def cloud_stream(store, name):
    return [
        (o.ts, np.asarray(o.data.points_f32()).reshape(-1, 3))
        for o in getattr(store.streams, name).order_by("ts")
    ]


with SqliteStore(path=str(db)) as store:
    surfaces = cloud_stream(store, "surface")
    plans = cloud_stream(store, "planned_path")
    held = cloud_stream(store, "held_path")
    goals = cloud_stream(store, "goal")
    scans = [(t, s[:, :2]) for t, s in cloud_stream(store, "lidar_live")]
    robot = np.array(
        [
            (o.ts, o.data.pose.position.x, o.data.pose.position.y)
            for o in store.streams.robot.order_by("ts")
        ]
    )


def latest(items, t):
    i = np.searchsorted([ts for ts, _ in items], t, side="right") - 1
    return items[i][1] if i >= 0 else None


everything = np.concatenate(
    [surfaces[-1][1][:, :2], surfaces[0][1][:, :2]] + [p[:, :2] for _, p in plans if len(p)]
)
lo, hi = np.percentile(everything, 0.5, axis=0) - 5, np.percentile(everything, 99.5, axis=0) + 5
frames = out.with_suffix("")
frames.mkdir(parents=True, exist_ok=True)
t0, t1 = robot[0, 0], robot[-1, 0]
for n, t in enumerate(np.arange(t0, t1 + step, step)):
    fig, ax = plt.subplots(figsize=(11, 8), dpi=100, facecolor="black")
    ax.set_facecolor("black")
    s = latest(surfaces, t)
    if s is not None:
        ax.scatter(
            s[:, 0],
            s[:, 1],
            s=0.15,
            c=s[:, 2],
            cmap="Greys_r",
            vmin=np.percentile(s[:, 2], 2) - 1,
            vmax=np.percentile(s[:, 2], 98),
            linewidths=0,
        )
    recent = [x for ts, x in scans if t - 1.0 < ts <= t]
    if recent:
        r = np.concatenate(recent)
        ax.scatter(r[:, 0], r[:, 1], s=0.2, c="red", alpha=0.5, linewidths=0)
    h = latest(held, t)
    if h is not None and len(h):
        ax.plot(
            h[:, 0],
            h[:, 1],
            color="orange",
            lw=1.5,
            ls="--",
            label="first plan (carried by the warps)",
        )
    p = latest(plans, t)
    if p is not None and len(p):
        ax.plot(p[:, 0], p[:, 1], color="cyan", lw=2.0, label="current plan")
    g = latest(goals, t)
    if g is not None:
        ax.plot(g[0, 0], g[0, 1], "*", color="yellow", ms=16, label="goal (held)")
    i = np.searchsorted(robot[:, 0], t, side="right") - 1
    if i >= 0:
        ax.plot(robot[i, 1], robot[i, 2], "o", color="lime", ms=8, label="robot")
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect("equal")
    ax.axis("off")
    phase = "BEFORE lap closure" if t < lap_ts else "AFTER lap closure"
    ax.set_title(f"{title}   t = {t - lap_ts:+6.1f} s ({phase})", color="white", fontsize=11)
    leg = ax.legend(
        loc="lower right", facecolor="black", edgecolor="gray", labelcolor="white", fontsize=8
    )
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
        "12",
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
