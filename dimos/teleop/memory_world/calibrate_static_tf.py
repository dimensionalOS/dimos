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

"""Measure the camera's real mount from the data, and write it back as a corrected tf.

The cart recordings' ``tf_static`` puts the camera 90 degrees out in yaw: pictures
hang askew against a level world even though the pictures themselves are level.
Nothing about the recording says so, and the only ground truth is the lidar -- the
same walls, seen twice. So this converts each depth image into a point cloud, pools
correspondences to the simultaneous lidar scan over every sampled frame, and solves
the ONE rigid transform that explains them all. Per-frame registration was tried
first and is far too unstable to trust: a rigid mount only shows up when the frames
are solved together.

The answer is written as a ``tf_static_corrected`` stream (one edge, the camera's
mount) which :func:`recording.build_tf_tree` applies over the recording's own
static tf. The recording itself is never rewritten.

    python -m dimos.teleop.memory_world.calibrate_static_tf <recording.db|.mcap> [--samples 20]
        [--dry-run]
"""

from __future__ import annotations

import argparse
import logging
from typing import Any

import numpy as np

from dimos.teleop.memory_world.recording import CORRECTED_STATIC_STREAM
from dimos.teleop.memory_world.tf_tree import quaternion_from_matrix

logger = logging.getLogger(__name__)

MIN_DEPTH_M, MAX_DEPTH_M = 1.0, 4.0  # outside this a d455 returns noise, not geometry
MAX_LIDAR_M = 12.0
VOXEL_M = 0.06
GRID_DEG = np.arange(-40, 41, 8.0)  # the seed rotations, around the nominal axis swap
SEEDS_REFINED = 24
# camera optical (x right, y down, z forward) -> a livox body frame (x fwd, y left, z up)
NOMINAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def _rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _thinned(points: np.ndarray, size: float, cap: int) -> np.ndarray:
    """One point per *size* cube, then at most *cap* of them."""
    _, keep = np.unique(np.floor(points / size).astype(np.int64), axis=0, return_index=True)
    points = points[keep]
    if len(points) > cap:
        points = points[np.random.default_rng(0).choice(len(points), cap, replace=False)]
    return points


def _pairs(store: Any, streams: dict[str, Any], samples: int) -> list[tuple[np.ndarray, Any]]:
    """(depth cloud in the camera's optical frame, kd-tree of the lidar scan) per sample."""
    from scipy.spatial import cKDTree

    from dimos.teleop.memory_world.recording import depth_info_stream_for

    info_name = depth_info_stream_for(
        set(store.list_streams()), streams["depth"], streams["camera_info"]
    )
    K = np.asarray(store.streams[info_name].first().data.K, dtype=np.float64)
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    depth, lidar = store.streams[streams["depth"]], store.streams[streams["lidar"]]
    first, last = depth.first(), depth.last()

    out = []
    for k in range(samples):
        ts = float(first.ts) + (k + 0.5) / samples * (float(last.ts) - float(first.ts))
        try:
            near = depth.at(ts, tolerance=0.2).first()
            scan = lidar.at(float(near.ts), tolerance=0.12).first()
        except LookupError:
            continue
        millimetres = np.asarray(near.data.data)
        if millimetres.ndim != 2:
            continue
        step = 4  # a 1280x720 depth image is far more than the fit needs
        rows, cols = np.mgrid[0 : millimetres.shape[0] : step, 0 : millimetres.shape[1] : step]
        z = millimetres[::step, ::step].astype(np.float64) / 1000.0
        usable = (z > MIN_DEPTH_M) & (z < MAX_DEPTH_M) & np.isfinite(z)
        if usable.sum() < 2000:
            continue
        zz = z[usable]
        cloud = np.stack([(cols[usable] - cx) * zz / fx, (rows[usable] - cy) * zz / fy, zz], axis=1)
        points = np.asarray(scan.data.points_f32(), dtype=np.float64)
        points = points[np.isfinite(points).all(axis=1)]
        points = points[np.linalg.norm(points, axis=1) < MAX_LIDAR_M]
        if len(points) < 3000:
            continue
        out.append((_thinned(cloud, VOXEL_M, 2500), cKDTree(_thinned(points, VOXEL_M, 60000))))
    return out


def _residual(matrix: np.ndarray, pairs: list[Any], trim: float) -> float:
    """Mean distance from the camera cloud to the lidar cloud, capped at *trim*."""
    total, count = 0.0, 0
    for cloud, kd in pairs:
        distance, _ = kd.query(cloud @ matrix[:3, :3].T + matrix[:3, 3], workers=-1)
        total += float(np.minimum(distance, trim).sum())
        count += len(distance)
    return total / max(count, 1)


def _joint_icp(matrix: np.ndarray, pairs: list[Any], rounds: int = 40) -> np.ndarray:
    """Pool every frame's correspondences and solve one rigid transform from the pool."""
    matrix = matrix.copy()
    for step in range(rounds):
        trim = max(0.12, 0.6 * (0.9**step))  # a wide gate first, tightening as it settles
        source, target = [], []
        for cloud, kd in pairs:
            distance, nearest = kd.query(cloud @ matrix[:3, :3].T + matrix[:3, 3], workers=-1)
            keep = distance < trim
            if keep.sum() < 50:
                continue
            source.append(cloud[keep])
            target.append(kd.data[nearest[keep]])
        if not source:
            return matrix
        source, target = np.vstack(source), np.vstack(target)
        source_mid, target_mid = source.mean(axis=0), target.mean(axis=0)
        u, _, vt = np.linalg.svd((source - source_mid).T @ (target - target_mid))
        rotation = (u @ np.diag([1, 1, np.sign(np.linalg.det(u @ vt))]) @ vt).T
        matrix = np.eye(4)
        matrix[:3, :3], matrix[:3, 3] = rotation, target_mid - rotation @ source_mid
    return matrix


def measure_camera_from_lidar(store: Any, streams: dict[str, Any], samples: int = 20) -> Any:
    """Solve ``lidar_T_depth_optical`` from *samples* depth/lidar pairs. None when too few."""
    pairs = _pairs(store, streams, samples)
    logger.info("%d usable depth/lidar pairs", len(pairs))
    if len(pairs) < 4:
        return None
    scout = pairs[:: max(1, len(pairs) // 5)][:5]  # a cheap score picks the seeds to refine
    seeds = []
    for roll in np.radians(GRID_DEG):
        for pitch in np.radians(GRID_DEG):
            for yaw in np.radians(GRID_DEG):
                seed = np.eye(4)
                seed[:3, :3] = _rotation(roll, pitch, yaw) @ NOMINAL
                seeds.append((_residual(seed, scout, 0.6), seed))
    seeds.sort(key=lambda pair: pair[0])

    best, best_cost = None, np.inf
    for _, seed in seeds[:SEEDS_REFINED]:
        fitted = _joint_icp(seed, pairs)
        cost = _residual(fitted, pairs, 0.3)
        if cost < best_cost:
            best, best_cost = fitted, cost
    if best is None:
        return None
    inliers = [
        float((kd.query(cloud @ best[:3, :3].T + best[:3, 3], workers=-1)[0] < 0.1).mean())
        for cloud, kd in pairs
    ]
    logger.info(
        "residual %.4f m, median inlier fraction %.2f", best_cost, float(np.median(inliers))
    )
    return best, best_cost, inliers


def sensor_lidar_stream(store: Any, tree: Any, streams: dict[str, Any]) -> str | None:
    """The lidar stream that is still in the sensor's own frame.

    A stitched recording also carries its clouds in the map frame (``*_corrected``,
    stamped ``corrected_odom``). Those are the same walls seen from a moving robot,
    so no single transform relates them to the camera; only the raw scans calibrate.
    """
    for name in [streams.get("lidar"), *streams.get("lidar_candidates", [])]:
        if not name:
            continue
        frame = str(getattr(store.streams[name].first().data, "frame_id", "") or "").lstrip("/")
        if any(child == frame for _, child in tree._edges):  # a frame that hangs off the robot
            return name
    return None


def camera_mount_edge(tree: Any, camera_frame: str) -> tuple[str | None, str]:
    """The edge that carries the whole camera: (whatever it is bolted to, the camera root).

    Correcting it moves colour, depth and infra together, which is what a wrong
    mount actually means -- the frames inside the camera are the vendor's and right.
    """
    parent_of = {child: parent for parent, child in tree._edges}
    node = camera_frame
    while parent_of.get(node, "").startswith("camera"):
        node = parent_of[node]
    return parent_of.get(node), node


def corrected_mount(
    tree: Any, lidar_T_camera: np.ndarray, camera_frame: str, lidar_frame: str, ts: float
) -> Any:
    """What the camera's mount edge should have been, given the measured extrinsic."""
    mount, camera_root = camera_mount_edge(tree, camera_frame)
    if mount is None:
        return None
    mount_T_lidar = tree.lookup(mount, lidar_frame, ts, 0.5)
    root_T_camera = tree.lookup(camera_root, camera_frame, ts, 0.5)
    if mount_T_lidar is None or root_T_camera is None:
        return None
    matrix = np.asarray(mount_T_lidar) @ lidar_T_camera @ np.linalg.inv(np.asarray(root_T_camera))
    return mount, camera_root, matrix


def write_corrected_static(
    store: Any, mount: str, child: str, matrix: np.ndarray, ts: float
) -> None:
    """Record the corrected edge where :func:`recording.build_tf_tree` will find it."""
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage

    if CORRECTED_STATIC_STREAM in store.list_streams():
        store.delete_stream(CORRECTED_STATIC_STREAM)  # one corrected answer per recording
    x, y, z, w = quaternion_from_matrix(matrix[:3, :3])
    store.stream(CORRECTED_STATIC_STREAM, TFMessage).append(
        TFMessage(
            Transform(
                translation=Vector3(*(float(v) for v in matrix[:3, 3])),
                rotation=Quaternion(float(x), float(y), float(z), float(w)),
                frame_id=mount,
                child_frame_id=child,
                ts=ts,
            )
        ),
        ts=ts,
    )


def _report(name: str, matrix: np.ndarray) -> str:
    r = matrix[:3, :3]
    pitch = float(np.arcsin(np.clip(-r[2, 0], -1, 1)))
    if abs(r[2, 0]) > 0.999999:
        roll, yaw = float(np.arctan2(-r[1, 2], r[1, 1])), 0.0
    else:
        roll, yaw = float(np.arctan2(r[2, 1], r[2, 2])), float(np.arctan2(r[1, 0], r[0, 0]))
    xyz = np.round(matrix[:3, 3], 4)
    return f"{name}: xyz {xyz}  rpy {np.round(np.degrees([roll, pitch, yaw]), 2)} deg"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--dry-run", action="store_true", help="measure and print, write nothing")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    from dimos.teleop.memory_world.recording import build_tf_tree, detect_streams, open_recording

    store = open_recording(args.recording)
    store.start()
    try:
        streams = detect_streams(store)
        for role in ("depth", "lidar", "camera_info", "tf"):
            if not streams.get(role):
                raise SystemExit(f"{args.recording} has no {role} stream; cannot calibrate")
        tree = build_tf_tree(store, streams["tf"])
        sensor_lidar = sensor_lidar_stream(store, tree, streams)
        if sensor_lidar is None:
            raise SystemExit(
                f"every lidar stream of {args.recording} is in a map frame; the raw scans"
                " are what calibrates against the camera"
            )
        streams["lidar"] = sensor_lidar
        camera_frame = str(
            getattr(store.streams[streams["depth"]].first().data, "frame_id", "") or ""
        ).lstrip("/")
        lidar_frame = str(
            getattr(store.streams[streams["lidar"]].first().data, "frame_id", "") or ""
        ).lstrip("/")
        ts = float(store.streams[streams["depth"]].first().ts)

        measured = measure_camera_from_lidar(store, streams, args.samples)
        if measured is None:
            raise SystemExit("not enough matched depth/lidar frames to measure the mount")
        lidar_T_camera, residual, inliers = measured
        print(_report(f"measured {lidar_frame} <- {camera_frame}", lidar_T_camera))
        print(f"  residual {residual:.4f} m, inlier fraction {np.round(inliers, 2)}")

        as_recorded = tree.lookup(lidar_frame, camera_frame, ts, 0.5)
        if as_recorded is not None:
            print(_report("the recording's own tf ", np.asarray(as_recorded)))
            print(
                f"  residual {_residual(np.asarray(as_recorded), _pairs(store, streams, 6), 0.3):.4f} m"
            )

        fixed = corrected_mount(tree, lidar_T_camera, camera_frame, lidar_frame, ts)
        if fixed is None:
            raise SystemExit(f"no mount edge above {camera_frame}; nothing to correct")
        mount, child, matrix = fixed
        print(_report(f"corrected {mount} -> {child}", matrix))
        if args.dry_run:
            print("dry run: nothing written")
            return
        write_corrected_static(store, mount, child, matrix, ts)
        print(f"wrote {CORRECTED_STATIC_STREAM} to {args.recording}")
    finally:
        store.stop()


if __name__ == "__main__":
    main()
