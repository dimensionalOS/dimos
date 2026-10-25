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

"""GSC's planned paths through its biggest lap closure, as the report viewer's plan JSON.
Runs plan_replay.py --lean (reforming ray tracer + MLS planner, goal held and warped through the warps), then writes {"states": [{"ts", "goal", "path", "robot", "why"}, ...]} (ts in s since the run's first
keyframe, x/y in GSC's corrected frame), one state per replan.
Run from the mls_reform worktree (it needs dimos_mls_planner and dimos_voxel_ray_tracing):
usage: gsc_plan.py <gsc run dir> <world db> <out json> [--threads N] [--before 60] [--after 60] [--goal-frac 0.4]
       [--keep-db]   (the replay db, beside the json as gsc_plan.db, is deleted after unless --keep-db; --threads caps
       the ray tracer and planner workers)"""

import argparse
import json
from pathlib import Path
import runpy
import sys

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("run", type=Path)
parser.add_argument("db", type=Path)
parser.add_argument("out", type=Path)
parser.add_argument("--threads", type=int, default=0)
parser.add_argument("--before", type=float, default=60.0)
parser.add_argument("--after", type=float, default=60.0)
parser.add_argument("--goal-frac", type=float, default=0.4)
parser.add_argument("--keep-db", action="store_true")
args = parser.parse_args()
args.out.parent.mkdir(parents=True, exist_ok=True)
replay_db = args.out.with_name("gsc_plan.db")

sys.path.insert(0, "dimos/navigation/global_planner/mls_planner/utils")
import reform_metrics as rm

if args.threads:
    from dimos_mls_planner import MLSPlanner

    from dimos.robot.unitree.go2.nav_3d_config import ray_tracing_config

    ray_tracing_config.worker_threads = args.threads

    def threads_capped(*a, **k):
        k["worker_threads"] = args.threads
        return MLSPlanner(*a, **k)

    rm.MLSPlanner = threads_capped
plan_replay = Path(__file__).with_name("plan_replay.py")
sys.argv = [
    str(plan_replay),
    str(args.run / "a.kf"),
    str(replay_db),
    "--db",
    str(args.db),
    "--before",
    str(args.before),
    "--after",
    str(args.after),
    "--goal-frac",
    str(args.goal_frac),
    "--lean",
]
runpy.run_path(str(plan_replay), run_name="__main__")

from dimos.memory.store.sqlite import SqliteStore

keyframes = np.fromfile(args.run / "a.kf", rm.KEYFRAME)
t0 = float(keyframes["ts"][0])
info = json.loads(replay_db.with_suffix(".plan.json").read_text())
with SqliteStore(path=str(replay_db)) as store:

    def cloud_stream(name):
        return [
            (o.ts, np.asarray(o.data.points_f32()).reshape(-1, 3))
            for o in getattr(store.streams, name).order_by("ts")
        ]

    plans, goals = cloud_stream("planned_path"), cloud_stream("goal")
    robot = np.array(
        [
            (o.ts, o.data.pose.position.x, o.data.pose.position.y)
            for o in store.streams.robot.order_by("ts")
        ]
    )
# plan_replay writes goal and planned_path once per replan, in order; info["plans"] says why each one ran
assert len(plans) == len(goals) == len(info["plans"]), (len(plans), len(goals), len(info["plans"]))
states = []
for (ts, path), (_, goal), plan in zip(plans, goals, info["plans"], strict=True):
    i = max(int(np.searchsorted(robot[:, 0], ts, side="right")) - 1, 0)
    states.append(
        {
            "ts": round(ts - t0, 3),
            "goal": [round(float(v), 3) for v in goal[0, :2]],
            "path": [[round(float(x), 2), round(float(y), 2)] for x, y in path[:, :2]],
            "robot": [round(float(v), 3) for v in robot[i, 1:]],
            "why": plan["why"],
        }
    )
result = {
    "method": "gsc",
    "frame": "GSC corrected (a.kf corrected / closure post poses)",
    "t0": t0,
    "lap_ts": round(info["lap_ts"] - t0, 3),
    "lap_shift_m": info["lap_shift_m"],
    "goal_kf_ts": round(info["goal_ts"] - t0, 3),
    "states": states,
}
args.out.write_text(json.dumps(result))
if not args.keep_db:
    for suffix in ("", "-wal", "-shm"):
        Path(f"{replay_db}{suffix}").unlink(missing_ok=True)
planned = sum(1 for s in states if s["path"])
print(f"wrote {args.out}: {len(states)} states, {planned} with a path", flush=True)
