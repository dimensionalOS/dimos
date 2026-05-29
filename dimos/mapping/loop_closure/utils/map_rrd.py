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

"""Dump a recorded dataset to .rrd: lidar point clouds + camera frames.

Lidar clouds are assumed to be in world frame and logged directly under
their entity path (no parent transform). Entities written:

- ``world/lidar``         — Go2 L1 per-frame point cloud
- ``world/lidar_voxels``  — accumulated voxel map of the primary lidar (``--map``)
- ``world/fastlio_lidar`` — fastlio_lidar raw cloud (if present)
- ``world/fastlio_voxels``— accumulated voxel map of fastlio_lidar (``--map``)
- ``world/fastlio``       — fastlio_odometry pose axis (if present)
- ``world/fastlio_path``  — fastlio_odometry trajectory (growing LineStrips3D)
- ``world/odom``          — Go2 onboard odom pose axis (if present)
- ``world/odom_path``     — Go2 onboard odom trajectory (growing LineStrips3D)
- ``world/camera``        — color_image camera pose (static pinhole + Transform3D)
- ``world/camera/image``  — color_image frames

Usage:
    uv run python -m dimos.mapping.loop_closure.utils.map_rrd mid360 --out map.rrd
    uv run python -m dimos.mapping.loop_closure.utils.map_rrd mid360 --out map.rrd --map
    rerun map.rrd
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import time
from typing import Any

import rerun as rr
import typer

from dimos.mapping.voxels import VoxelMapTransformer
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.stream import Stream
from dimos.memory2.transform import throttle
from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2, register_colormap_annotation
from dimos.robot.unitree.go2.connection import _camera_info_static
from dimos.utils.data import resolve_named_path

TIMELINE = "ts"


def _progress(total: int, label: str) -> Callable[[Observation[Any]], None]:
    """Matches dimos/utils/cli/map.py:progress."""
    seen = 0
    wall_start: float | None = None
    last_wall: float | None = None
    first_ts: float | None = None

    def tick(obs: Observation[Any]) -> None:
        nonlocal seen, wall_start, last_wall, first_ts
        now = time.monotonic()
        if wall_start is None:
            wall_start = now
            first_ts = obs.ts
        assert first_ts is not None
        frame_ms = (now - last_wall) * 1000 if last_wall is not None else 0.0
        last_wall = now
        seen += 1
        pct = 100 * seen // total if total else 100
        wall = now - wall_start
        data = obs.ts - first_ts
        speed = data / wall if wall > 0 else 0.0
        end = "\n" if seen >= total else ""
        print(
            f"\r{label} {pct:>3}% [{seen}/{total}] {data:.1f}s ({speed:.1f} x rt) {frame_ms:.0f}ms/frame",
            end=end,
            flush=True,
        )

    return tick


def _log_clouds(
    label: str,
    stream: Stream[PointCloud2],
    entity: str,
    voxel: float,
    point_mode: str,
    *,
    total: int | None = None,
) -> None:
    """Iterate a PointCloud2 stream and log each obs to ``entity``.

    ``total`` overrides the progress denominator — useful for transform
    pipelines where calling :py:meth:`Stream.count` would materialize the
    whole pipeline.
    """
    n = total if total is not None else stream.count()
    cb = _progress(n, label)
    for obs in stream:
        cb(obs)
        rr.set_time(TIMELINE, timestamp=obs.ts)
        rr.log(entity, obs.data.to_rerun(voxel_size=voxel, mode=point_mode))


def _log_path(
    label: str,
    stream: Stream[Any],
    entity: str,
    color: tuple[int, int, int],
    *,
    emit_every: int = 10,
) -> None:
    """Iterate a pose-bearing stream and log a growing :class:`LineStrips3D` to
    ``entity`` every ``emit_every`` poses (and once more at the end). Frames
    without a pose are skipped.
    """
    n = stream.count()
    cb = _progress(n, label)
    points: list[tuple[float, float, float]] = []
    last_ts: float | None = None
    emit_count = 0
    for obs in stream:
        cb(obs)
        if obs.pose_tuple is None:
            continue
        points.append(
            (float(obs.pose_tuple[0]), float(obs.pose_tuple[1]), float(obs.pose_tuple[2]))
        )
        last_ts = obs.ts
        emit_count += 1
        if emit_every > 0 and emit_count % emit_every == 0 and len(points) >= 2:
            rr.set_time(TIMELINE, timestamp=obs.ts)
            rr.log(entity, rr.LineStrips3D([points], colors=[color]))
    if (
        last_ts is not None
        and len(points) >= 2
        and (emit_every <= 0 or emit_count % emit_every != 0)
    ):
        rr.set_time(TIMELINE, timestamp=last_ts)
        rr.log(entity, rr.LineStrips3D([points], colors=[color]))


def main(
    dataset: str = typer.Argument(..., help="Dataset .db: bare name (cwd or data/) or path"),
    out: Path = typer.Option(..., "--out", help="Output .rrd path"),
    voxel: float = typer.Option(
        0.05, "--voxel", help="Voxel size hint for the point cloud renderer"
    ),
    point_mode: str = typer.Option(
        "spheres", "--point-mode", help="Render mode: 'spheres', 'boxes', or 'points'"
    ),
    camera_hz: float = typer.Option(
        0.0,
        "--camera-hz",
        help="Throttle color_image to at most this rate; 0 (default) logs all frames",
    ),
    map: bool = typer.Option(
        False,
        "--map",
        help="Accumulate each lidar stream into a VoxelGrid and log only the final map",
    ),
    map_voxel: float = typer.Option(
        0.05, "--map-voxel", help="Voxel size for the accumulated map (m); --map only"
    ),
    map_device: str = typer.Option(
        "CUDA:0", "--map-device", help="Open3D device for the VoxelGrid; --map only"
    ),
    map_emit_every: int = typer.Option(
        10,
        "--map-emit-every",
        help="Emit accumulated map every N frames (0 = only at end); --map only",
    ),
) -> None:
    db_path = resolve_named_path(dataset, ".db")
    cam_info = _camera_info_static()

    rr.init("dimos map_rrd", recording_id=db_path.stem)
    rr.save(str(out))
    register_colormap_annotation("turbo")

    # Static pinhole on the camera entity; per-frame Transform3D goes on the
    # same entity. Image is the child so it projects through the pinhole.
    pinhole = cam_info.to_rerun()
    assert not isinstance(pinhole, list)
    rr.log("world/camera", pinhole, static=True)

    # Static axis triads as children of each moving Transform3D, so the
    # transforms are actually visible in the 3D view.
    axes = rr.Arrows3D(
        vectors=[[0.3, 0, 0], [0, 0.3, 0], [0, 0, 0.3]],
        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
    )
    rr.log("world/fastlio/axes", axes, static=True)
    rr.log("world/odom/axes", axes, static=True)

    store = SqliteStore(path=str(db_path))
    with store:
        print(store.summary())

        lidar = store.stream("lidar", PointCloud2)
        color_image = store.stream("color_image", Image)
        has_livox = "fastlio_lidar" in store.streams
        livox = store.stream("fastlio_lidar", PointCloud2) if has_livox else None

        # ---- per-frame raw clouds ----
        _log_clouds("       lidar", lidar, "world/lidar", voxel, point_mode)
        if livox is not None:
            _log_clouds("fastlio_lidar", livox, "world/fastlio_lidar", voxel, point_mode)

        # ---- accumulated voxel maps (--map only) ----
        # Go2 L1 forward-facing → column carving on.
        # Mid360 spherical → column carving off, just aggregate.
        if map:
            grid_kwargs = {"voxel_size": map_voxel, "device": map_device, "show_startup_log": False}
            _log_clouds(
                " lidar_voxels",
                lidar.transform(
                    VoxelMapTransformer(
                        emit_every=map_emit_every, carve_columns=True, **grid_kwargs
                    )
                ),
                "world/lidar_voxels",
                voxel,
                point_mode,
                total=max(1, lidar.count() // max(map_emit_every, 1)),
            )
            if livox is not None:
                _log_clouds(
                    "fastlio_voxels",
                    livox.transform(
                        VoxelMapTransformer(
                            emit_every=map_emit_every, carve_columns=False, **grid_kwargs
                        )
                    ),
                    "world/fastlio_voxels",
                    voxel,
                    point_mode,
                    total=max(1, livox.count() // max(map_emit_every, 1)),
                )

        # ---- fastlio pose axis + path from fastlio_odometry stream ----
        if "fastlio_odometry" in store.streams:
            odometry = store.stream("fastlio_odometry", Odometry)
            cb = _progress(odometry.count(), "fastlio_odometry")
            for obs in odometry:
                cb(obs)
                if obs.pose_tuple is None:
                    continue
                rr.set_time(TIMELINE, timestamp=obs.ts)
                x, y, z, qx, qy, qz, qw = obs.pose_tuple
                rr.log(
                    "world/fastlio",
                    rr.Transform3D(
                        translation=[x, y, z],
                        quaternion=rr.Quaternion(xyzw=[qx, qy, qz, qw]),
                    ),
                )
            _log_path(
                "  fastlio_path",
                store.stream("fastlio_odometry", Odometry),
                "world/fastlio_path",
                color=(255, 165, 0),  # orange
            )

        # ---- Go2 native odom pose axis + path ----
        if "odom" in store.streams:
            odom = store.stream("odom", PoseStamped)
            cb = _progress(odom.count(), "        odom")
            for odom_obs in odom:
                cb(odom_obs)
                if odom_obs.pose_tuple is None:
                    continue
                rr.set_time(TIMELINE, timestamp=odom_obs.ts)
                x, y, z, qx, qy, qz, qw = odom_obs.pose_tuple
                rr.log(
                    "world/odom",
                    rr.Transform3D(
                        translation=[x, y, z],
                        quaternion=rr.Quaternion(xyzw=[qx, qy, qz, qw]),
                    ),
                )
            _log_path(
                "     odom_path",
                store.stream("odom", PoseStamped),
                "world/odom_path",
                color=(0, 200, 100),  # green
            )

        # ---- pass 2: camera pose + image per color_image ----
        cam_pipeline = (
            color_image.transform(throttle(1.0 / camera_hz)) if camera_hz > 0 else color_image
        )
        n_img = cam_pipeline.count()
        cb = _progress(n_img, "  color_image")
        for img_obs in cam_pipeline:
            cb(img_obs)
            rr.set_time(TIMELINE, timestamp=img_obs.ts)
            if img_obs.pose_tuple is not None:
                x, y, z, qx, qy, qz, qw = img_obs.pose_tuple
                rr.log(
                    "world/camera",
                    rr.Transform3D(
                        translation=[x, y, z], quaternion=rr.Quaternion(xyzw=[qx, qy, qz, qw])
                    ),
                )
            rr.log("world/camera/image", img_obs.data.to_rerun())

    print(f"wrote {out}")
    print(f"open with: rerun {out}")


if __name__ == "__main__":
    typer.run(main)
