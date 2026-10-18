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

"""Turn a (sensor-frame lidar, odometry, tf) recording into the go2 shape both PGOs take:
a `lidar` stream of world-frame scans, each stamped with the lidar's world pose.

usage: prep.py <name> <db> <lidar_stream> <odom_stream> [<pose_of>] [--voxel=<m>]   -> $EVAL_LOOP_OUT/<name>.src.db
       --voxel: downsample each scan to one point (the centroid) per <m> voxel, as Point-LIO does to
       what it publishes (filter_size_surf), for a recording that stored the scans undownsampled
"""

import sys

import numpy as np
from paths import OUT as HERE
from scipy.spatial.transform import Rotation, Slerp

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def static_chain(store, parent: str, child: str):
    """parent<-child as (Rotation, t) from the first sample of each recorded tf edge, through their
    common ancestor (so it holds whichever of the two sits higher in the tree)."""
    if parent == child:
        return Rotation.identity(), np.zeros(3)
    edges = {}
    for i, obs in enumerate(store.stream("tf").order_by("ts")):
        for tf in obs.data.transforms:
            edges.setdefault(tf.child_frame_id, (tf.frame_id, tf.translation, tf.rotation))
        if i > 5000:
            break

    def to_ancestors(frame):
        """{ancestor: ancestor<-frame} for the frame and every frame above it."""
        rotation, translation, chain = Rotation.identity(), np.zeros(3), {}
        while True:
            chain[frame] = (rotation, translation)
            if frame not in edges:
                return chain
            up, t, q = edges[frame]
            r = Rotation.from_quat([q.x, q.y, q.z, q.w])
            rotation, translation = r * rotation, r.apply(translation) + np.array([t.x, t.y, t.z])
            frame = up

    from_parent, from_child = to_ancestors(parent), to_ancestors(child)
    common = next((frame for frame in from_child if frame in from_parent), None)
    if common is None:
        raise LookupError(f"no tf path between {parent} and {child}")
    (pr, pt), (cr, ct) = from_parent[common], from_child[common]
    return pr.inv() * cr, pr.inv().apply(ct - pt)


def voxel_downsample(points: np.ndarray, leaf: float) -> np.ndarray:
    """One point per `leaf` voxel: the centroid of the points in it (PCL VoxelGrid's rule)."""
    keys = np.floor(points / leaf).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    sums = np.zeros((len(counts), 3))
    np.add.at(sums, inverse.ravel(), points)
    return sums / counts[:, None]


def main():
    voxel = next((float(a.split("=", 1)[1]) for a in sys.argv if a.startswith("--voxel=")), 0.0)
    args = [a for a in sys.argv[1:] if not a.startswith("--voxel=")]
    name, db, lidar_name, odom_name = args[:4]
    # the frame whose pose the odometry really reports, when its child_frame_id is wrong about it
    pose_of = args[4] if len(args) > 4 else ""
    out = HERE / f"{name}.src.db"
    tmp = out.with_suffix(".tmp.db")
    tmp.unlink(missing_ok=True)
    with SqliteStore(path=db) as store:
        odom = [(o.ts, o.data) for o in store.stream(odom_name).order_by("ts")]
        times = np.array([ts for ts, _ in odom])
        poses = [getattr(m, "pose", m) for _, m in odom]  # Odometry.pose or the PoseStamped itself
        poses = [getattr(p, "pose", p) for p in poses]  # PoseWithCovariance.pose
        positions = np.array([[p.position.x, p.position.y, p.position.z] for p in poses])
        quats = np.array(
            [[p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w] for p in poses]
        )
        body = pose_of or getattr(odom[0][1], "child_frame_id", "") or "base_link"
        times, unique = np.unique(times, return_index=True)
        positions, quats = positions[unique], quats[unique]
        slerp = Slerp(times, Rotation.from_quat(quats))
        lidar_stream = store.stream(lidar_name)
        lidar_frame = lidar_stream.first().data.frame_id
        # go2 legacy: scans are already world-frame, only the body pose is missing
        already_world = lidar_frame in ("world", "odom", "map")
        mount_r, mount_t = (
            (Rotation.identity(), np.zeros(3))
            if already_world
            else static_chain(store, body, lidar_frame)
        )
        print(
            f"{name}: odom {odom_name} ({len(odom)}) body={body}, lidar {lidar_name} frame={lidar_frame}, "
            f"mount t={np.round(mount_t, 3)}",
            flush=True,
        )
        written = 0
        with SqliteStore(path=str(tmp)) as dst:
            stream = dst.stream("lidar", PointCloud2)
            for obs in lidar_stream.order_by("ts"):
                if not (times[0] <= obs.ts <= times[-1]):
                    continue
                pts = np.asarray(obs.data.points_f32(), np.float64)[:, :3]
                if len(pts) == 0:
                    continue
                if voxel > 0:
                    pts = voxel_downsample(pts, voxel)
                body_r = slerp([obs.ts])[0]
                body_t = np.array([np.interp(obs.ts, times, positions[:, i]) for i in range(3)])
                lidar_r, lidar_t = body_r * mount_r, body_r.apply(mount_t) + body_t
                world = pts if already_world else lidar_r.apply(pts) + lidar_t
                x, y, z = lidar_t
                qx, qy, qz, qw = lidar_r.as_quat()
                stream.append(
                    PointCloud2.from_numpy(
                        world.astype(np.float32), frame_id="world", timestamp=obs.ts
                    ),
                    ts=obs.ts,
                    pose=(x, y, z, qx, qy, qz, qw),
                )
                written += 1
    tmp.rename(out)
    print(f"{name}: wrote {written} scans -> {out}", flush=True)


if __name__ == "__main__":
    main()
