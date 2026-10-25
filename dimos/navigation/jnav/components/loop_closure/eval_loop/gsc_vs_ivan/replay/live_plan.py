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

"""Replay a go2 recording through the real unitree-go2-mls-reform stack at 1x (`dtk run --replay --record sqlite`)
and send one goal: at `--goal-at` s into the recording, a point `--goal-frac` of the way around the lap (from the
place the lap returns to), so the only mapped route runs back through the lap's second half. The planner module
replans on its own as the map changes and warps. Stops `--settle` s after `--stop-at`.
The goal is the goal keyframe where the live GSC holds it at --goal-at (its newest pose_graph_records); the
offline run only picks which keyframe (the nearest in time to --goal-kf-at) and is the fallback.
usage (from the dimos worktree, venv active):
  live_plan.py --db <recording.db> --kf <offline a.kf> --lap-return <s> --lap-end <s> --goal-at <s> --stop-at <s>"""

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import psutil
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, "dimos/navigation/global_planner/mls_planner/utils")
import reform_metrics as rm

from dimos.core.transport_factory import make_transport
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.nav_3d_config import mls_planner_config

parser = argparse.ArgumentParser()
parser.add_argument("--db", type=Path, required=True)
parser.add_argument("--kf", type=Path, required=True)
parser.add_argument(
    "--lap-return", type=float, required=True, help="recording s of the place the lap comes back to"
)
parser.add_argument(
    "--lap-end", type=float, required=True, help="recording s when the robot is back there"
)
parser.add_argument("--goal-at", type=float, required=True)
parser.add_argument("--goal-frac", type=float, default=0.4)
parser.add_argument(
    "--goal-kf-at",
    type=float,
    default=None,
    help="recording s of the keyframe to use as the goal (overrides the lap fraction)",
)
parser.add_argument("--stop-at", type=float, required=True)
parser.add_argument("--settle", type=float, default=20.0)
parser.add_argument("--max-rss-gb", type=float, default=24.0)
parser.add_argument("--out", type=Path, required=True)
parser.add_argument(
    "--extra",
    default="",
    help='more config tokens for dtk run, e.g. --extra="--gscpgo.loop-gnc-deterministic false"',
)
args = parser.parse_args()

with SqliteStore(path=str(args.db)) as store:
    t0 = next(iter(store.streams.lidar.order_by("ts"))).ts
keyframes = np.fromfile(args.kf, rm.KEYFRAME)
closures = rm.read_closures(Path(f"{args.kf}.closures"))
goal_ts = t0 + (
    args.goal_kf_at
    if args.goal_kf_at is not None
    else args.lap_return + args.goal_frac * (args.lap_end - args.lap_return)
)
# the correction in force at --goal-at: the newest closure's newest keyframe, post * raw^-1
rotation, translation = R.identity(), np.zeros(3)
for ts, records in closures:
    if ts > t0 + args.goal_at:
        break
    n = records[np.argmax(records["id"])]
    delta = R.from_quat(n["dq"].astype(float))
    post_rotation = delta * R.from_quat(n["q"].astype(float))
    post_translation = delta.apply(n["xyz"].astype(float)) + n["dt"]
    k = int(n["id"])
    rotation = post_rotation * R.from_quat(keyframes["raw"][k, 3:]).inv()
    translation = post_translation - rotation.apply(keyframes["raw"][k, :3])
k = int(np.searchsorted(keyframes["ts"], goal_ts))
goal_xyz = rotation.apply(keyframes["raw"][k, :3]) + translation
goal_xyz[2] -= mls_planner_config.start_z_offset_m
print(
    f"goal (map) {goal_xyz.round(2)} from keyframe at {keyframes['ts'][k] - t0:.1f} s", flush=True
)

args.out.mkdir(parents=True, exist_ok=True)
log = (args.out / "run.log").open("w")
launched = time.time()
proc = subprocess.Popen(
    [
        "dtk",
        "run",
        "unitree-go2-mls-reform",
        "--replay",
        "--replay-db",
        str(args.db),
        *args.extra.split(),
    ],
    stdout=log,
    stderr=subprocess.STDOUT,
    start_new_session=True,
)

graph = {}


def on_graph(msg) -> None:
    graph["points"] = np.asarray(msg.points_f32()).reshape(-1, 3)


first = {}


def on_odom(msg) -> None:
    if "wall" not in first:
        first["wall"], first["ts"] = time.time(), msg.ts
        print(
            f"first odom ts {msg.ts:.2f} ({'recording clock' if abs(msg.ts - t0) < 3600 else 'wall clock'})",
            flush=True,
        )


odom = make_transport("odom", PoseStamped)
odom.subscribe(on_odom)
goal_pub = make_transport("goal", PointStamped)
graph_sub = make_transport("pose_graph_records", PointCloud2)
graph_sub.subscribe(on_graph)


def recording_s() -> float:
    return time.time() - first["wall"] if "wall" in first else -1.0


rss_peak: dict[str, float] = {}
sent = 0
try:
    while proc.poll() is None:
        for p in psutil.Process(proc.pid).children(recursive=True):
            try:
                name = p.name()
                rss_peak[name] = max(rss_peak.get(name, 0.0), p.memory_info().rss / 2**30)
                if p.memory_info().rss / 2**30 > args.max_rss_gb:
                    print(f"{name} over {args.max_rss_gb} GB, stopping", flush=True)
                    raise KeyboardInterrupt
            except psutil.Error:
                pass
        t = recording_s()
        if t >= args.goal_at and sent < 3 and t >= args.goal_at + 2.0 * sent:
            if sent == 0 and "points" in graph and k < len(graph["points"]):
                goal_xyz = graph["points"][k].astype(float)
                goal_xyz[2] -= mls_planner_config.start_z_offset_m
                print(f"goal from the live pose graph: {goal_xyz.round(2)}", flush=True)
            goal_pub.broadcast(None, PointStamped(*map(float, goal_xyz), frame_id="map"))
            sent += 1
            print(f"goal sent at recording {t:.1f} s", flush=True)
        if t >= args.stop_at + args.settle:
            break
        time.sleep(1.0)
except KeyboardInterrupt:
    pass
finally:
    if proc.poll() is None:
        import os
        import signal

        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=60)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
recordings = sorted(Path("recordings").glob("*/memory.db"), key=lambda p: p.stat().st_mtime)
newest = [str(p) for p in recordings if p.stat().st_mtime >= launched]
info = {
    "t0": t0,
    "goal_xyz": goal_xyz.tolist(),
    "goal_at": args.goal_at,
    "first": first,
    "rss_peak_gb": rss_peak,
    "recording": newest,
}
(args.out / "live_plan.json").write_text(json.dumps(info, indent=1))
print(json.dumps(info, indent=1), flush=True)
