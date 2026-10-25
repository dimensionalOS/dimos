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

"""Derive a go2-eval-style db (world-frame `lidar`, PoseStamped `odom`, plus `color_image` and `raw_april_tags`
when present) from a recording whose lidar is in the sensor/body frame, so every replay tool can read it.
World points = odometry pose * sensor cloud (the odometry's child frame is the cloud's frame). The written odom is
odometry * body_tf, e.g. a gravity-level frame for a tilted Mid-360, so loop search sees level clouds.
usage: make_world_db.py <src.db> <odom stream> <lidar stream> <out.db> [--body-tf x,y,z,qx,qy,qz,qw] [--copy STREAM ...]"""

import argparse
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

parser = argparse.ArgumentParser()
parser.add_argument("src", type=Path)
parser.add_argument("odom")
parser.add_argument("lidar")
parser.add_argument("out", type=Path)
parser.add_argument("--body-tf", default="0,0,0,0,0,0,1")
parser.add_argument("--copy", nargs="*", default=[])
args = parser.parse_args()
body = [float(v) for v in args.body_tf.split(",")]
body_rotation, body_translation = R.from_quat(body[3:]), np.array(body[:3])

args.out.unlink(missing_ok=True)
with SqliteStore(path=str(args.src)) as src, SqliteStore(path=str(args.out)) as out:
    odom_rows = []
    for obs in getattr(src.streams, args.odom).order_by("ts"):
        p = (
            obs.data.pose
            if hasattr(obs.data, "pose") and hasattr(obs.data.pose, "position")
            else obs.data
        )
        odom_rows.append(
            (
                obs.ts,
                p.position.x,
                p.position.y,
                p.position.z,
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w,
            )
        )
    odom = np.array(odom_rows)
    odom_out = out.stream("odom", PoseStamped)
    for ts, x, y, z, qx, qy, qz, qw in odom:
        rotation = R.from_quat([qx, qy, qz, qw]) * body_rotation
        position = np.array([x, y, z]) + R.from_quat([qx, qy, qz, qw]).apply(body_translation)
        q = rotation.as_quat()
        msg = PoseStamped(
            ts=float(ts), frame_id="world", position=list(position), orientation=list(q)
        )
        odom_out.append(msg, ts=float(ts), pose=(*position, *q))
    lidar_out = out.stream("lidar", PointCloud2)
    written = 0
    for obs in getattr(src.streams, args.lidar).order_by("ts"):
        i = int(np.searchsorted(odom[:, 0], obs.ts, side="right")) - 1
        if i < 0:
            continue
        i = min((i, i + 1), key=lambda j: abs(odom[min(j, len(odom) - 1), 0] - obs.ts))
        i = min(i, len(odom) - 1)
        ts, x, y, z, qx, qy, qz, qw = odom[i]
        points = np.asarray(obs.data.points_f32(), np.float64)
        if len(points) == 0:
            continue
        world = [*R.from_quat([qx, qy, qz, qw]).apply(points), x, y, z]
        cloud = PointCloud2.from_numpy(world.astype(np.float32), frame_id="world")
        cloud.ts = obs.ts
        lidar_out.append(cloud, ts=obs.ts, pose=(x, y, z, qx, qy, qz, qw))
        written += 1
    for name in args.copy:
        if name not in src.list_streams():
            continue
        source = getattr(src.streams, name)
        sink = None
        for obs in source.order_by("ts"):
            if sink is None:
                sink = out.stream(name, type(obs.data))
            sink.append(obs.data, ts=obs.ts, pose=obs.pose)
print(f"wrote {args.out}: {len(odom)} odom, {written} lidar", flush=True)
