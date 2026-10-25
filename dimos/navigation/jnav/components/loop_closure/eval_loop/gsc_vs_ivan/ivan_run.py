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

"""Ivan's live module (PGOMap at PGOVoxelMapperConfig defaults, global map every emit_every frames) on a gsc_runs
world db, written in gsc_run's layout (a.kf + a.kf.closures) so score_run.py and tag_agreement.py score it unchanged.
Live: scans arrive at their recorded time and the module keeps only the newest one that arrived while busy (its
backpressure subscription); --offline feeds every scan. A closure state is written each time the loop count changes.
usage: ivan_run.py <world db> <out dir> [--offline]   (run from the dimos worktree)"""

import argparse
import json
from pathlib import Path
import queue
import struct
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.navigation.go2.loop_closure import module as ivan_module, pgo_map

KEYFRAME = np.dtype([("ts", "<f8"), ("raw", "<f8", 7), ("corrected", "<f8", 7)])
CLOSURE_RECORD = np.dtype(
    [
        ("xyz", "<f4", 3),
        ("id", "<u4"),
        ("ts", "<f8"),
        ("q", "<f4", 4),
        ("dt", "<f4", 3),
        ("dq", "<f4", 4),
    ]
)

parser = argparse.ArgumentParser()
parser.add_argument("db")
parser.add_argument("out", type=Path)
parser.add_argument("--offline", action="store_true")
args = parser.parse_args()
args.out.mkdir(parents=True, exist_ok=True)


def row(tf):
    t, r = tf.translation, tf.rotation
    return [t.x, t.y, t.z, r.x, r.y, r.z, r.w]


with SqliteStore(path=args.db) as store:
    odom = np.array(
        [
            (
                o.ts,
                o.data.position.x,
                o.data.position.y,
                o.data.position.z,
                o.data.orientation.x,
                o.data.orientation.y,
                o.data.orientation.z,
                o.data.orientation.w,
            )
            for o in store.streams.odom.order_by("ts")
        ]
    )
odom = odom[np.unique(odom[:, 0], return_index=True)[1]]
odom_slerp = Slerp(odom[:, 0], R.from_quat(odom[:, 4:]))


def level_pose(ts):
    """The world db's odom (odometry * body tf, gravity level) at a scan's time."""
    ts = float(np.clip(ts, odom[0, 0], odom[-1, 0]))
    xyz = [float(np.interp(ts, odom[:, 0], odom[:, k])) for k in (1, 2, 3)]
    return Pose(position=xyz, orientation=list(odom_slerp([ts])[0].as_quat()))


cfg = ivan_module.PGOVoxelMapperConfig()
gmap = pgo_map.PGOMap(
    voxel_size=cfg.voxel_size,
    frame_id=cfg.frame_id,
    rebuild_cooldown_s=cfg.rebuild_cooldown_s,
    pgo=cfg.pgo,
)
closures = (args.out / "a.kf.closures").open("wb")
frames = dropped = states = 0
busy_s = 0.0


def work(obs):
    global frames, states, busy_s
    t0 = time.perf_counter()
    loops = gmap.n_loops
    gmap.add(obs.data, level_pose(obs.ts))
    frames += 1
    if frames % cfg.emit_every == 0:
        gmap.global_map()
    if gmap.n_loops != loops:
        keyframes = gmap.graph().keyframes
        records = np.zeros(len(keyframes), CLOSURE_RECORD)
        for i, kf in enumerate(keyframes):
            local, optimized = np.array(row(kf.local)), np.array(row(kf.optimized))
            delta = R.from_quat(optimized[3:]) * R.from_quat(local[3:]).inv()
            records[i] = (
                local[:3],
                i,
                kf.ts,
                local[3:],
                optimized[:3] - delta.apply(local[:3]),
                delta.as_quat(),
            )
        closures.write(struct.pack("<dI", obs.ts, len(records)) + records.tobytes())
        states += 1
    busy_s += time.perf_counter() - t0


scans: queue.Queue = queue.Queue(maxsize=256)


def read():
    with SqliteStore(path=args.db) as store:
        for obs in store.streams.lidar.order_by("ts"):
            obs.data.points_f32()  # decode off the module's thread, as its transport does
            scans.put(obs)
    scans.put(None)


threading.Thread(target=read, daemon=True).start()
started = time.perf_counter()
wall0 = ts0 = None
newest = None
for obs in iter(scans.get, None):
    if args.offline:
        work(obs)
        continue
    if wall0 is None:
        wall0, ts0 = time.perf_counter(), obs.ts
    arrival = wall0 + (obs.ts - ts0)
    if arrival > time.perf_counter() and newest is not None:
        work(newest)
        newest = None
    if arrival <= time.perf_counter():
        if newest is not None:
            dropped += 1
        newest = obs
        continue
    while (remaining := arrival - time.perf_counter()) > 0:
        time.sleep(max(remaining / 8, 0.0005))  # macOS stretches long background sleeps
    work(obs)
if newest is not None:
    work(newest)
closures.close()

graph = gmap.graph()
kf = np.zeros(len(graph.keyframes), KEYFRAME)
for i, k in enumerate(graph.keyframes):
    kf[i] = (k.ts, row(k.local), row(k.optimized))
kf.tofile(args.out / "a.kf")
wall = time.perf_counter() - started
summary = dict(
    mode="offline" if args.offline else "live",
    frames=frames,
    dropped=dropped,
    keyframes=len(kf),
    loops=len(graph.loops),
    closure_states=states,
    wall_s=round(wall, 1),
    busy_s=round(busy_s, 1),
)
(args.out / "ivan.json").write_text(json.dumps(summary, indent=1))
# the accepted loop edges (keyframe ids, ICP fitness), for loop inspection; PGOMap keeps them on its PGO
loops = [
    dict(source_id=l.source, target_id=l.target, score=l.score) for l in gmap._pgo._accepted_loops
]
(args.out / "loops.json").write_text(json.dumps(loops))
gmap.dispose()
print(args.out.name, json.dumps(summary), flush=True)
