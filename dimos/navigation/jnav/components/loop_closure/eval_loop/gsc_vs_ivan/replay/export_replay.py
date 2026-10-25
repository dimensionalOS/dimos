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

"""Replay a GSC run through the reforming ray tracer and write what happens over time into a new db:
lidar_live (each scan where it was placed then), gsc_odometry (GSC's newest keyframe after each update),
gsc_keyframes (every keyframe position after each update that moved the path), reformed_map (the tracer's
healthy voxels, downsampled, after each warp and every `--map-every` seconds). Every scan goes through the tracer;
every `--write-every`th is stored, downsampled to `--scan-voxel`.
With --edges (pgo-eval's <a.kf>.edges), GSC's loop-edge lifecycle goes in as loop_edges: one cloud per event
time, each event two points (source keyframe, then target keyframe, where they were then), intensity = confidence,
tag = state (0 proposed, 1 gate-rejected, 2 committed, 3 GNC vote, 4 accepted, 5 rejected, 6 applied),
offset_time = keyframe id.
usage: export_replay.py <a.kf> <out.db> [--map-every 20] [--map-voxel 0.25] [--scan-voxel 0.15] [--write-every 3] [--edges a.kf.edges]"""

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import psutil
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
parser.add_argument(
    "--db",
    type=Path,
    default=Path.home() / ".cache/dimos/eval_loop/dbf5283feeb446d49f065a100985a78e.db",
)
parser.add_argument("--map-every", type=float, default=20.0)
parser.add_argument("--map-voxel", type=float, default=0.25)
parser.add_argument("--stride", type=int, default=1)
parser.add_argument("--scan-voxel", type=float, default=0.15)
parser.add_argument("--write-every", type=int, default=3)
parser.add_argument("--edges", type=Path, default=None)
args = parser.parse_args()

keyframes = np.fromfile(args.keyframes, rm.KEYFRAME)
closures = rm.read_closures(Path(f"{args.keyframes}.closures"))
args.out.unlink(missing_ok=True)
out = SqliteStore(path=str(args.out))
out.__enter__()
lidar_live = out.stream("lidar_live", PointCloud2)
gsc_odometry = out.stream("gsc_odometry", Odometry)
gsc_keyframes = out.stream("gsc_keyframes", PointCloud2)
reformed_map = out.stream("reformed_map", PointCloud2)

# GSC's loop-edge records (gsc_pgo LoopEdgeEvent::put_record)
EDGE = np.dtype(
    [
        ("xyz", "<f4", 3),
        ("target_xyz", "<f4", 3),
        ("source_id", "<u4"),
        ("target_id", "<u4"),
        ("state", "<u4"),
        ("confidence", "<f4"),
        ("yank_z", "<f4"),
        ("ts", "<f8"),
    ]
)
edge_records = np.fromfile(args.edges, EDGE) if args.edges else np.zeros(0, EDGE)
if args.edges:
    loop_edges = out.stream("loop_edges", PointCloud2)
    for ts in np.unique(edge_records["ts"]):
        group = edge_records[edge_records["ts"] == ts]
        points = np.stack([group["xyz"], group["target_xyz"]], axis=1).reshape(-1, 3)
        cloud = PointCloud2.from_numpy(
            points.astype(np.float32),
            frame_id="map",
            intensities=np.repeat(group["confidence"], 2).astype(np.float32),
            tags=np.repeat(group["state"], 2).astype(np.uint8),
            offset_times=np.stack([group["source_id"], group["target_id"]], axis=1)
            .reshape(-1)
            .astype(np.uint32),
        )
        cloud.ts = float(ts)
        loop_edges.append(cloud, ts=float(ts))
    print(f"{len(edge_records)} loop-edge events", flush=True)

positions: dict[int, np.ndarray] = {}
last_map_ts = -1e18
started = time.monotonic()


def downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return points[first]


def write_map(ts: float, tracer) -> None:
    global last_map_ts
    centers = np.asarray(tracer.healthy_centers(), np.float32).reshape(-1, 3)
    cloud = PointCloud2.from_numpy(downsample(centers, args.map_voxel), frame_id="map")
    cloud.ts = ts
    reformed_map.append(cloud, ts=ts)
    last_map_ts = ts


def on_closure(ts: float, records: np.ndarray, tracer) -> None:
    rotations = R.from_quat(records["dq"].astype(float)) * R.from_quat(records["q"].astype(float))
    xyz = (
        R.from_quat(records["dq"].astype(float)).apply(records["xyz"].astype(float)) + records["dt"]
    )
    moved = 0.0
    for i, k in enumerate(records["id"].tolist()):
        if k in positions:
            moved = max(moved, float(np.linalg.norm(xyz[i] - positions[k])))
        positions[k] = xyz[i]
    newest = int(np.argmax(records["id"]))
    x, y, z = xyz[newest]
    qx, qy, qz, qw = rotations[newest].as_quat()
    gsc_odometry.append(
        Odometry(
            ts=ts, frame_id="map", child_frame_id="gsc_keyframe", pose=Pose(x, y, z, qx, qy, qz, qw)
        ),
        ts=ts,
        pose=(x, y, z, qx, qy, qz, qw),
    )
    if moved > 0.05 or len(positions) % 50 == 0:
        path = np.array([positions[k] for k in sorted(positions)], np.float32)
        cloud = PointCloud2.from_numpy(path, frame_id="map")
        cloud.ts = ts
        gsc_keyframes.append(cloud, ts=ts)
    if moved > 1.0:
        write_map(ts, tracer)


scans_seen = 0


def on_scan(ts: float, points: np.ndarray, origin: np.ndarray, tracer) -> None:
    global scans_seen
    scans_seen += 1
    if ts - last_map_ts >= args.map_every:
        write_map(ts, tracer)
    if scans_seen % args.write_every:
        return
    cloud = PointCloud2.from_numpy(
        downsample(points.astype(np.float32), args.scan_voxel), frame_id="map"
    )
    cloud.ts = ts
    lidar_live.append(cloud, ts=ts, pose=(*map(float, origin), 0.0, 0.0, 0.0, 1.0))


process = psutil.Process()
perf = {
    "scan_ms": [],
    "scan_ts": [],
    "warp_ms": [],
    "warp_ts": [],
    "local_map_ms": [],
    "rss_mb": [],
    "rss_ts": [],
}
real_modules = rm.lockstep_modules


class TimedTracer:
    """The tracer with its scan path (add_frame_world, local_map) and warps timed, RSS sampled per scan."""

    def __init__(self, tracer):
        self._tracer = tracer
        self._ts = 0.0

    def __getattr__(self, name):
        return getattr(self._tracer, name)

    def set_clock(self, ts):
        self._ts = ts
        return self._tracer.set_clock(ts)

    def add_frame_world(self, *a):
        t = time.perf_counter()
        out = self._tracer.add_frame_world(*a)
        perf["scan_ms"].append((time.perf_counter() - t) * 1e3)
        perf["scan_ts"].append(self._ts)
        if len(perf["scan_ts"]) % 10 == 0:
            perf["rss_mb"].append(process.memory_info().rss / 2**20)
            perf["rss_ts"].append(self._ts)
        return out

    def local_map(self, *a):
        t = time.perf_counter()
        out = self._tracer.local_map(*a)
        perf["local_map_ms"].append((time.perf_counter() - t) * 1e3)
        return out

    def apply_loop_closure(self, *a, **k):
        t = time.perf_counter()
        out = self._tracer.apply_loop_closure(*a, **k)
        perf["warp_ms"].append((time.perf_counter() - t) * 1e3)
        perf["warp_ts"].append(self._ts)
        return out


def timed_modules():
    tracer, planner = real_modules()
    return TimedTracer(tracer), planner


rm.lockstep_modules = timed_modules
rm.lockstep(
    args.db,
    keyframes,
    closures,
    scan_stride=args.stride,
    with_planner=False,
    on_scan=on_scan,
    on_closure=on_closure,
)
out.__exit__(None, None, None)
perf["peak_rss_mb"] = max(perf["rss_mb"], default=0.0)
args.out.with_suffix(".perf.json").write_text(json.dumps(perf))
print(f"wrote {args.out} in {time.monotonic() - started:.0f} s", flush=True)
