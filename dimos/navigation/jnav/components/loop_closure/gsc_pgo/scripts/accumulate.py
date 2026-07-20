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

# Untyped analysis script: open3d / the rust voxel mapper lack type stubs.
# mypy: ignore-errors
"""Accumulate a per-scan lidar stream into ONE voxel cloud with raycast clearing.

Each scan is world-registered (already-world scans pass through, sensor-frame
scans go through the recording tf), then fed to the Rust ``VoxelRayMapper``,
which carves free space along every ray so dynamic objects and registration
ghosts get cleared instead of smeared into the map. The healthy voxel centers
are the final, already voxel-downsampled cloud, written back as a single
``<stream>_accumulated`` PointCloud2 event.

Kept deliberately thin so it runs standalone (fast, re-runnable) and is also
imported by post_process.py to build the original + corrected accumulated maps.

Usage: python .../scripts/accumulate.py --rec=PATH [--stream=pointlio_lidar]
       [--out=<stream>_accumulated] [--world-frame=world] [--lidar-frame=mid360_link]
       [--voxel=0.05] [--max-range=20] [--no-tf]
"""

from __future__ import annotations

from pathlib import Path
import sys
import time
from typing import TYPE_CHECKING

import numpy as np

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.jnav.utils import recording_db as rdb
from dimos.navigation.jnav.utils.recording_tf import RecordingTF

if TYPE_CHECKING:
    from dimos.memory2.store.sqlite import SqliteStore

DEFAULT_VOXEL = 0.05  # accumulation resolution == the final voxel-downsample size (m)
DEFAULT_MAX_RANGE = 20.0  # ignore returns / clearing beyond this range (m)
DEFAULT_LIDAR_FRAME = "mid360_link"  # frame sensor-frame scans live in
DEFAULT_WORLD_FRAME = "world"  # frame to register scans into (scans already in it pass through)
PROGRESS_EVERY = 2000  # scans between progress prints


def _quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5 or 1.0
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        float,
    )


def _world_register(observation, store_tf, world_frame, lidar_frame, origin_lookup=None):
    """``(points_world_f32, sensor_origin_world)`` for a lidar observation.

    A scan already in ``world_frame`` is returned untouched with its stored pose
    position as the ray origin. A sensor-frame scan is brought into the world via
    tf (``world_frame <- scan_frame``); the tf translation IS the sensor origin.
    Falls back to the stored pose, then to assuming the scan is already world.
    Returns ``(points, None)`` when no origin can be resolved (scan is skipped).

    ``origin_lookup(ts) -> (x, y, z)`` supplies the ray origin for already-world
    scans that carry no per-scan pose (e.g. Point-LIO world clouds), sourced from
    the matching odometry trajectory.
    """
    points = np.asarray(observation.data.points_f32())
    if not len(points):
        return points, None
    pose = observation.pose_tuple
    scan_frame = getattr(observation.data, "frame_id", "") or lidar_frame
    if scan_frame == world_frame:
        if pose is not None:
            origin = (float(pose[0]), float(pose[1]), float(pose[2]))
        elif origin_lookup is not None:
            origin = origin_lookup(float(observation.ts))
        else:
            origin = None
        return points.astype(np.float32), origin
    if store_tf is not None:
        transform = store_tf.get(world_frame, scan_frame, float(observation.ts), None)
        if transform is not None:
            rotation = np.asarray(transform.rotation.to_rotation_matrix(), float).reshape(3, 3)
            translation = np.array(
                [transform.translation.x, transform.translation.y, transform.translation.z], float
            )
            world = points @ rotation.T + translation
            return world.astype(np.float32), (
                float(translation[0]),
                float(translation[1]),
                float(translation[2]),
            )
    if pose is not None:
        rotation = _quat_to_matrix(pose[3], pose[4], pose[5], pose[6])
        world = points @ rotation.T + np.array(pose[:3], float)
        return world.astype(np.float32), (float(pose[0]), float(pose[1]), float(pose[2]))
    return points.astype(np.float32), None


def _odom_origin_lookup(store: SqliteStore, odom_stream: str):
    """``lookup(ts) -> (x, y, z)`` returning the nearest odometry position in time."""
    samples = np.array(
        [
            (
                float(observation.ts),
                observation.data.pose.position.x,
                observation.data.pose.position.y,
                observation.data.pose.position.z,
            )
            for observation in store.stream(odom_stream)
        ],
        float,
    )
    times = samples[:, 0]

    def lookup(ts: float) -> tuple[float, float, float]:
        index = int(np.searchsorted(times, ts))
        index = min(max(index, 0), len(samples) - 1)
        if index > 0 and abs(times[index - 1] - ts) < abs(times[index] - ts):
            index -= 1
        return float(samples[index, 1]), float(samples[index, 2]), float(samples[index, 3])

    return lookup


def accumulate_stream(
    store: SqliteStore,
    in_stream: str,
    out_stream: str,
    *,
    world_frame: str = DEFAULT_WORLD_FRAME,
    lidar_frame: str = DEFAULT_LIDAR_FRAME,
    use_tf: bool = True,
    voxel_size: float = DEFAULT_VOXEL,
    max_range: float = DEFAULT_MAX_RANGE,
    origin_stream: str | None = None,
) -> int:
    """Raycast-accumulate ``in_stream`` into a single ``out_stream`` cloud; returns point count.

    ``origin_stream`` names an odometry stream whose trajectory supplies the ray
    origin for already-world scans that lack a per-scan pose (needed to raycast
    Point-LIO world clouds, which carry no origin of their own).
    """
    from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper

    store_tf = RecordingTF.from_store(store) if use_tf else None
    origin_lookup = _odom_origin_lookup(store, origin_stream) if origin_stream else None
    mapper = VoxelRayMapper(voxel_size=voxel_size, max_range=max_range)

    print(
        f"accumulating {in_stream} -> {out_stream} "
        f"(voxel={voxel_size} m, max_range={max_range} m, "
        f"world_frame={world_frame}, tf={'on' if store_tf is not None else 'off'})",
        flush=True,
    )
    scan_count = skipped = 0
    last_ts = 0.0
    start_time = time.time()
    for observation in store.stream(in_stream):
        world_points, origin = _world_register(
            observation, store_tf, world_frame, lidar_frame, origin_lookup
        )
        if origin is None or not len(world_points):
            skipped += 1
            continue
        mapper.add_frame(world_points, origin)
        last_ts = float(observation.ts)
        scan_count += 1
        if scan_count % PROGRESS_EVERY == 0:
            print(
                f"  {scan_count} scans, {mapper.voxel_count():,} voxels, "
                f"{time.time() - start_time:.0f}s",
                flush=True,
            )

    accumulated = np.asarray(mapper.global_map(), np.float32)
    if out_stream in store.list_streams():
        store.delete_stream(out_stream)
    cloud = PointCloud2.from_numpy(accumulated, frame_id=world_frame, timestamp=last_ts)
    store.stream(out_stream, PointCloud2).append(cloud, ts=last_ts, pose=None)
    print(
        f"wrote {out_stream}: {len(accumulated):,} pts from {scan_count} scans "
        f"({skipped} skipped) in {time.time() - start_time:.0f}s",
        flush=True,
    )
    return len(accumulated)


def _arg(flag: str, default: str = "") -> str:
    return next(
        (item.split("=", 1)[1] for item in sys.argv if item.startswith(flag + "=")), default
    )


def main() -> None:
    rec_arg = _arg("--rec")
    if not rec_arg:
        sys.exit(
            "usage: python .../scripts/accumulate.py --rec=PATH [--stream=pointlio_lidar] "
            "[--out=...] [--world-frame=world] [--lidar-frame=mid360_link] "
            "[--origin=pointlio_odometry] [--voxel=0.05] [--max-range=20] [--no-tf]   (--rec required)"
        )
    rec = Path(rec_arg).expanduser()
    in_stream = _arg("--stream", "pointlio_lidar")
    out_stream = _arg("--out", f"{in_stream}_accumulated")
    store = rdb.store(rec / "mem2.db")
    accumulate_stream(
        store,
        in_stream,
        out_stream,
        world_frame=_arg("--world-frame", DEFAULT_WORLD_FRAME),
        lidar_frame=_arg("--lidar-frame", DEFAULT_LIDAR_FRAME),
        use_tf="--no-tf" not in sys.argv,
        voxel_size=float(_arg("--voxel", str(DEFAULT_VOXEL))),
        max_range=float(_arg("--max-range", str(DEFAULT_MAX_RANGE))),
        origin_stream=_arg("--origin") or None,
    )


if __name__ == "__main__":
    main()
