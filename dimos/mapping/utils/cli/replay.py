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
- ``world/fastlio_lidar`` — fastlio_lidar raw cloud (if present)
- ``world/<stream>_voxels`` — growing voxel map, one per PointCloud2 stream (``--map``)
- ``world/<stream>_map``    — single static voxel map, one per PointCloud2 stream (``--map-final``)
- ``world/fastlio``       — fastlio_odometry pose axis (if present)
- ``world/fastlio_path``  — fastlio_odometry trajectory (growing LineStrips3D)
- ``world/odom``          — Go2 onboard odom pose axis (if present)
- ``world/odom_path``     — Go2 onboard odom trajectory (growing LineStrips3D)
- ``world/camera``        — color_image camera pose (static pinhole + Transform3D)
- ``world/camera/image``  — color_image frames
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import subprocess
import time
from typing import TYPE_CHECKING, Any

import rerun as rr
import typer

# Heavy dimos imports (mapping/memory2 → torch, scipy, open3d) are deferred into
# main() so that `dimos map --help` stays fast. See test_cli_startup.py and the
# same pattern in dimos/mapping/utils/cli/map.py.
if TYPE_CHECKING:
    from dimos.memory2.stream import Stream
    from dimos.memory2.type.observation import Observation
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

TIMELINE = "ts"


def _progress(total: int, label: str) -> Callable[[Observation[Any]], None]:
    """Matches dimos/mapping/utils/cli/map.py:progress."""
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
    bottom_cutoff: float | None = None,
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
        rr.log(
            entity,
            obs.data.to_rerun(voxel_size=voxel, mode=point_mode, bottom_cutoff=bottom_cutoff),
        )


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
    out: Path | None = typer.Option(
        None, "--out", help="Output .rrd path (default: ./<dataset>.rrd)"
    ),
    no_gui: bool = typer.Option(False, "--no-gui", help="Don't launch rerun on the result"),
    seek: float = typer.Option(0.0, "--seek", help="Skip the first N seconds of the recording"),
    duration: float | None = typer.Option(
        None, "--duration", help="Use only N seconds from --seek (default: to the end)"
    ),
    voxel: float = typer.Option(
        0.05,
        "--voxel",
        help="Voxel grid resolution (m) for --map/--map-final; rendering follows the same size",
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
        help="Accumulate each lidar stream into a VoxelGrid, logging a growing map over the timeline",
    ),
    map_final: bool = typer.Option(
        False,
        "--map-final",
        help="Log a single static accumulated map of the whole recording (independent of --map)",
    ),
    map_source: list[str] = typer.Option(
        [],
        "--map-source",
        help="PointCloud2 stream(s) to map; repeatable. Default: all PointCloud2 streams",
    ),
    map_carve_columns: bool = typer.Option(
        False,
        "--map-carve-columns/--no-map-carve-columns",
        help="Clear the full Z column under each new voxel, keeping only the latest surface "
        "(good for forward-facing lidar like the Go2 L1); --map/--map-final only",
    ),
    map_device: str = typer.Option(
        "CUDA:0", "--map-device", help="Open3D device for the VoxelGrid; --map/--map-final only"
    ),
    map_emit_every: int = typer.Option(
        10,
        "--map-emit-every",
        help="Emit accumulated map every N frames (0 = only at end); --map only",
    ),
    bottom_cutoff: float | None = typer.Option(
        None,
        "--bottom-cutoff",
        help="Drop accumulated-map points below this Z (m) when rendering; e.g. 0 strips the floor; --map/--map-final only",
    ),
) -> None:
    """Dump a recording to .rrd (lidar clouds + camera frames) and open it in rerun."""
    from dimos.mapping.utils.cli.summary import _stream_payload_types
    from dimos.mapping.voxels import VoxelMapTransformer
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.memory2.transform import throttle
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.nav_msgs.Odometry import Odometry
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2, register_colormap_annotation
    from dimos.robot.unitree.go2.config import camera_info_static
    from dimos.utils.data import resolve_named_path

    db_path = resolve_named_path(dataset, ".db")
    if out is None:
        out = Path.cwd() / f"{db_path.stem}.rrd"
    cam_info = camera_info_static()

    # Resolve which streams to voxelize: all PointCloud2 streams, or the
    # explicit --map-source subset. Validate up front so typos fail fast.
    pc_streams = [n for n, t in _stream_payload_types(db_path).items() if t is PointCloud2]
    map_sources = list(map_source) or pc_streams
    if (map or map_final) and (bad := [s for s in map_sources if s not in pc_streams]):
        raise typer.BadParameter(f"--map-source: not PointCloud2 stream(s): {', '.join(bad)}")

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

        def clipped(name: str, ptype: type[Any]) -> Stream[Any]:
            return store.stream(name, ptype).from_time(seek or None).to_time(duration)

        lidar = clipped("lidar", PointCloud2)
        color_image = clipped("color_image", Image)
        has_livox = "fastlio_lidar" in store.streams
        livox = clipped("fastlio_lidar", PointCloud2) if has_livox else None

        # Per-frame raw clouds.
        _log_clouds("       lidar", lidar, "world/lidar", voxel, point_mode)
        if livox is not None:
            _log_clouds("fastlio_lidar", livox, "world/fastlio_lidar", voxel, point_mode)

        # Accumulated voxel maps over the selected PointCloud2 streams.
        # --map logs a growing map per stream; --map-final logs one static map
        # per stream. --map-carve-columns clears the Z column under each surface
        # voxel (good for forward-facing lidar like the Go2 L1); off by default.
        if map or map_final:
            grid_kwargs = {"voxel_size": voxel, "device": map_device, "show_startup_log": False}
            for name in map_sources:
                src = clipped(name, PointCloud2)
                if not src.exists():
                    continue
                if map:
                    _log_clouds(
                        f"{name}_voxels",
                        src.transform(
                            VoxelMapTransformer(
                                emit_every=map_emit_every,
                                carve_columns=map_carve_columns,
                                **grid_kwargs,
                            )
                        ),
                        f"world/{name}_voxels",
                        voxel / 4,  # render smaller than the grid → gaps read as transparency
                        point_mode,
                        total=max(1, src.count() // max(map_emit_every, 1)),
                        bottom_cutoff=bottom_cutoff,
                    )
                if map_final:
                    # emit_every=0 → one accumulated obs at exhaustion
                    final = src.transform(
                        VoxelMapTransformer(
                            emit_every=0, carve_columns=map_carve_columns, **grid_kwargs
                        )
                    ).last()
                    rr.log(
                        f"world/{name}_map",
                        final.data.to_rerun(
                            voxel_size=voxel / 4, mode=point_mode, bottom_cutoff=bottom_cutoff
                        ),
                        static=True,
                    )

        # fastlio pose axis + path from fastlio_odometry stream.
        if "fastlio_odometry" in store.streams:
            odometry = clipped("fastlio_odometry", Odometry)
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
                clipped("fastlio_odometry", Odometry),
                "world/fastlio_path",
                color=(255, 165, 0),  # orange
            )

        # Go2 native odom pose axis + path.
        if "odom" in store.streams:
            odom = clipped("odom", PoseStamped)
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
                clipped("odom", PoseStamped),
                "world/odom_path",
                color=(0, 200, 100),  # green
            )

        # Pass 2: camera pose + image per color_image.
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
    if no_gui:
        print(f"open with: rerun {out}")
    else:
        subprocess.Popen(["rerun", str(out)])


if __name__ == "__main__":
    typer.run(main)
