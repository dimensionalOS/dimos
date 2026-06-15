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

"""Replay a lidar+odometry .db through several voxel-mapper variants into rerun.

Each variant's global map is a separate, toggleable entity under world/maps.

Usage:
    uv run python -m dimos.mapping.ray_tracing.utils.raytrace_rrd go2_mid360_stairs
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rerun as rr
import typer

from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import FnTransformer
from dimos.memory2.type.observation import Observation
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import resolve_named_path

TIMELINE = "ts"

PairObs = Observation[tuple[Observation[PointCloud2], Observation[Odometry]]]

COLORS = {
    "naive": [90, 200, 90],
    "no_normal_gate": [235, 120, 60],
    "defaults": [70, 170, 235],
    "no_recency": [200, 80, 200],
}

# A window large enough to never expire, i.e. recency gating off.
RECENCY_OFF = 1_000_000_000

# Variants whose normal gate is active, so their normals are worth drawing.
NORMAL_VARIANTS = {"defaults", "no_recency"}


def _height_colors(centers: np.ndarray, base: list[int]) -> np.ndarray:
    """Shade each voxel by height, keeping the method's base hue."""
    if len(centers) == 0:
        return np.empty((0, 3), np.uint8)
    z = centers[:, 2]
    span = float(z.max() - z.min())
    # Only the top half of the brightness scale, so the low end stays visible.
    t = (z - z.min()) / span if span > 1e-6 else np.zeros(len(z), np.float32)
    brightness = 0.5 + 0.5 * t
    return (np.asarray(base, np.float32) * brightness[:, None]).astype(np.uint8)


def _attach_pose_from_odom(pair_obs: PairObs) -> Observation[PointCloud2]:
    lidar_obs, odom_obs = pair_obs.data
    odom = odom_obs.data
    pose_tuple = (
        float(odom.position.x),
        float(odom.position.y),
        float(odom.position.z),
        float(odom.orientation.x),
        float(odom.orientation.y),
        float(odom.orientation.z),
        float(odom.orientation.w),
    )
    return lidar_obs.with_pose(pose_tuple)


def main(
    dataset: str = typer.Argument(..., help="Dataset .db: bare name (cwd or data/) or path"),
    out: Path | None = typer.Option(
        None, "--out", help="Output .rrd path. If omitted, spawn rerun live."
    ),
    lidar_stream: str = typer.Option("fastlio_lidar", "--lidar-stream"),
    odom_stream: str = typer.Option("fastlio_odometry", "--odom-stream"),
    align_tol: float = typer.Option(0.05, "--align-tol", help="Lidar/odom alignment tolerance (s)"),
    voxel_size: float = typer.Option(0.1, "--voxel-size", help="Voxel edge length (m)"),
    max_range: float = typer.Option(30.0, "--max-range", help="Max ray cast distance (m)"),
    emit_every: int = typer.Option(1, "--emit-every", help="Log the maps every N frames"),
    render_voxel: float = typer.Option(0.05, "--render-voxel", help="Voxel render size (m)"),
    normal_scale: float = typer.Option(0.08, "--normal-scale", help="Normal arrow length (m)"),
    from_time: float | None = typer.Option(
        None, "--from-time", help="Start replay at this stream timestamp (s)"
    ),
) -> None:
    db_path = resolve_named_path(dataset, ".db")

    rr.init("raytrace_rrd", recording_id=db_path.stem)
    if out is not None:
        rr.save(str(out))
    else:
        rr.spawn()

    rr.log(
        "world/robot/axes",
        rr.Arrows3D(
            vectors=[[0.3, 0, 0], [0, 0.3, 0], [0, 0, 0.3]],
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
        ),
        static=True,
    )

    mappers = {
        "naive": VoxelRayMapper(
            voxel_size=voxel_size,
            max_range=max_range,
            shadow_depth=0.0,
            grace_depth=max_range,
            min_health=0,
        ),
        "no_normal_gate": VoxelRayMapper(voxel_size=voxel_size, max_range=max_range, graze_cos=0.0),
        "defaults": VoxelRayMapper(voxel_size=voxel_size, max_range=max_range),
        "no_recency": VoxelRayMapper(
            voxel_size=voxel_size, max_range=max_range, recency_window=RECENCY_OFF
        ),
    }

    store = SqliteStore(path=str(db_path))
    with store:
        lidar = store.stream(lidar_stream, PointCloud2).order_by("ts")
        if from_time is not None:
            lidar = lidar.from_time(from_time)
        odom = store.stream(odom_stream, Odometry).order_by("ts")
        pose_tagged = lidar.align(odom, tolerance=align_tol).transform(
            FnTransformer(_attach_pose_from_odom)
        )

        trajectory: list[tuple[float, float, float]] = []
        count = 0
        for obs in pose_tagged:
            if obs.pose_tuple is None:
                continue
            x, y, z, qx, qy, qz, qw = obs.pose_tuple
            pts = obs.data.points_f32()
            for mapper in mappers.values():
                mapper.add_frame(pts, (x, y, z))
            count += 1

            if count % emit_every != 0:
                continue

            rr.set_time(TIMELINE, timestamp=obs.ts)
            robot = np.asarray([x, y, z], np.float32)
            for name, mapper in mappers.items():
                if name not in NORMAL_VARIANTS:
                    centers = mapper.global_map()
                    rr.log(
                        f"world/maps/{name}",
                        rr.Points3D(
                            centers,
                            colors=_height_colors(centers, COLORS[name]),
                            radii=render_voxel / 2,
                        ),
                    )
                    continue
                centers, normals = mapper.global_map_normals()
                rr.log(
                    f"world/maps/{name}",
                    rr.Points3D(
                        centers,
                        colors=_height_colors(centers, COLORS[name]),
                        radii=render_voxel / 2,
                    ),
                )
                keep = np.any(normals != 0.0, axis=1)
                origins, vectors = centers[keep], normals[keep]
                flip = np.sum(vectors * (robot - origins), axis=1) < 0
                vectors = np.where(flip[:, None], -vectors, vectors)
                rr.log(
                    f"world/maps/{name}/normals",
                    rr.Arrows3D(
                        origins=origins,
                        vectors=vectors * normal_scale,
                        colors=[COLORS[name]],
                        radii=0.005,
                    ),
                )
            rr.log("world/raw_points", rr.Points3D(pts, colors=[[90, 90, 90]], radii=0.01))
            rr.log(
                "world/robot",
                rr.Transform3D(
                    translation=[x, y, z], quaternion=rr.Quaternion(xyzw=[qx, qy, qz, qw])
                ),
            )
            trajectory.append((x, y, z))
            if len(trajectory) >= 2:
                rr.log("world/robot_path", rr.LineStrips3D([trajectory], colors=[[255, 165, 0]]))
            print(f"frame={count}", end="\r", flush=True)
        print()

    if out is not None:
        print(f"wrote {out}\nopen with: rerun {out}")


if __name__ == "__main__":
    typer.run(main)
