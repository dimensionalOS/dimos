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

"""Live check of an m20 run: rates, frames, sources, DimSLAM odometry, tf chain, relocalization fitness, self-hits.

usage: verify_run.py <run_id> [seconds] [router]
"""

import collections
import sys
import threading
import time

import numpy as np
import zenoh

from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

run_id = sys.argv[1]
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
router = sys.argv[3] if len(sys.argv) > 3 else "tcp/127.0.0.1:7447"

lock = threading.Lock()
keys: collections.Counter = collections.Counter()
sources: dict[str, set] = collections.defaultdict(set)
frames: dict[str, collections.Counter] = collections.defaultdict(collections.Counter)
scan_stats: dict[str, list] = collections.defaultdict(list)
odom: list[tuple[float, float, float, float, float]] = []
edges: dict[tuple[str, str], list] = collections.defaultdict(list)
fitness: list[float] = []
decoders = {
    "lidar": PointCloud2,
    "lidar_filtered": PointCloud2,
    "lidar_imu": Imu,
    "slam_odom": Odometry,
    "tf": TFMessage,
    "relocalization_strength": Float32,
}


def stream_of(key: str) -> str:
    # dimos-domains/<run>/<topic...>#<type>; the topic is the last path part before any type suffix
    tail = key.split(f"dimos-domains/{run_id}/", 1)[-1]
    return tail.rsplit("/", 1)[0]


def on_sample(sample) -> None:
    key = str(sample.key_expr)
    stream = stream_of(key)
    now = time.time()
    with lock:
        keys[key] += 1
        source = getattr(sample, "source_info", None)
        zid = None
        if source is not None and getattr(source, "source_id", None) is not None:
            zid = str(source.source_id.zid)
        sources[stream].add(zid)
    decoder = decoders.get(stream)
    if decoder is None:
        return
    try:
        msg = decoder.lcm_decode(sample.payload.to_bytes())
    except Exception as error:
        with lock:
            frames[stream][f"decode error {type(error).__name__}"] += 1
        return
    with lock:
        if isinstance(msg, PointCloud2):
            frames[stream][msg.frame_id] += 1
            points = msg.points_f32()
            ranges = np.linalg.norm(points, axis=1) if len(points) else np.zeros(1)
            scan_stats[(stream, msg.frame_id)].append(
                (now, msg.ts, len(points), float((ranges < 0.6).sum()), float(ranges.min()))
            )
        elif isinstance(msg, Imu):
            frames[stream][msg.frame_id] += 1
        elif isinstance(msg, Odometry):
            p = msg.pose.position
            odom.append((now, msg.ts, p.x, p.y, p.z))
            frames[stream][f"{msg.frame_id}->{msg.child_frame_id}"] += 1
        elif isinstance(msg, TFMessage):
            for t in msg.transforms:
                edges[(t.frame_id, t.child_frame_id)].append(
                    (now, t.ts, t.translation.x, t.translation.y, t.translation.z)
                )
        elif isinstance(msg, Float32):
            fitness.append(msg.data)


config = zenoh.Config()
config.insert_json5("mode", '"client"')
config.insert_json5("connect/endpoints", f'["{router}"]')
config.insert_json5("scouting/multicast/enabled", "false")
session = zenoh.open(config)
print("verifier zid", session.info.zid())
print("routers", [str(z) for z in session.info.routers_zid()])
subscriber = session.declare_subscriber(f"dimos-domains/{run_id}/**", on_sample)
time.sleep(seconds)
subscriber.undeclare()

with lock:
    print(f"\n== {seconds:.0f} s on {router}, run {run_id}")
    for key, count in sorted(keys.items()):
        print(f"{count / seconds:8.1f} Hz  {key}")
    print("\n== publishers (zenoh source zids) per stream")
    for stream, zids in sorted(sources.items()):
        print(f"  {stream}: {sorted(str(z) for z in zids)}")
    print("\n== frames per stream")
    for stream, counter in frames.items():
        print(f"  {stream}: {dict(counter)}")
    print("\n== scans (points, points within 0.6 m of the lidar, nearest m)")
    for (stream, frame), rows in sorted(scan_stats.items()):
        arr = np.array(rows)
        print(
            f"  {stream} {frame}: {len(rows) / seconds:.1f} Hz, {arr[:, 2].mean():.0f} pts, "
            f"{arr[:, 3].mean():.1f} near, nearest {arr[:, 4].min():.2f} m, "
            f"wall-stamp lag {np.median(arr[:, 0] - arr[:, 1]) * 1e3:.0f} ms"
        )
    if odom:
        arr = np.array(odom)
        span = arr[-1, 1] - arr[0, 1]
        print(
            f"\n== slam_odom: {len(arr) / seconds:.1f} Hz, stamps span {span:.1f} s in {seconds:.0f} s wall, "
            f"latency median {np.median(arr[:, 0] - arr[:, 1]) * 1e3:.0f} ms, "
            f"position first {arr[0, 2:]} last {arr[-1, 2:]}, excursion {np.ptp(arr[:, 2:], axis=0)}"
        )
    print("\n== tf edges")
    for (parent, child), rows in sorted(edges.items()):
        arr = np.array(rows)
        print(
            f"  {parent} -> {child}: {len(rows) / seconds:.1f} Hz, last xyz {arr[-1, 2:].round(3)}"
        )
    if fitness:
        print(
            f"\n== relocalization_strength: {len(fitness)} attempts, values {[round(f, 3) for f in fitness[-12:]]}"
        )
session.close()
