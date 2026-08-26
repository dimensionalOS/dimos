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

"""Replay a lidar .db through several voxel-mapper variants into rerun.

Clouds are registered by the recorded tf stream at the cloud stamp, exactly as
the live module registers them. Each variant's global map is a separate,
toggleable entity under world/maps.

Usage:
    uv run python -m dimos.mapping.ray_tracing.utils.raytrace_rrd mid360_athens_stairs
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from numpy.typing import NDArray
import typer

from dimos.mapping.ray_tracing.module import TF_MATCH_TOLERANCE_S
from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tf import StreamTF
from dimos.memory.vis.utils import DEFAULT_RENDER_VOXEL, default_render_voxel
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import resolve_named_path

TIMELINE = "ts"

# --voxel-size default, and the render size --render-voxel scales from when unset.
DEFAULT_VOXEL_SIZE = 0.1

COLORS = {
    "naive": [90, 200, 90],
}

# Variants whose normal gate is active, so their normals are worth drawing.
NORMAL_VARIANTS = {"defaults"}

# Z half-extent of the fine local map query around the robot (m).
FINE_Z_HALF_EXTENT_M = 50.0

# Z-gradient stops: coarse map climbs a cool family, fine map a warm one, so
# the clouds stay separable at every height.
COARSE_RAMP = np.array(
    [[150, 70, 255], [60, 130, 255], [70, 220, 255], [215, 250, 255]], np.float32
)
FINE_RAMP = np.array([[255, 60, 50], [255, 130, 30], [255, 200, 60], [255, 250, 170]], np.float32)
# Normal arrows blend toward magenta, the one hue family neither ramp uses.
NORMAL_TINT = np.array([255, 60, 235], np.float32)
NORMAL_TINT_BLEND = 0.45


def _normal_colors(voxel_colors: NDArray[np.uint8]) -> NDArray[np.uint8]:
    """Magenta-cast copies of the voxel colors for their normal arrows."""
    mixed = (1.0 - NORMAL_TINT_BLEND) * voxel_colors + NORMAL_TINT_BLEND * NORMAL_TINT
    return mixed.astype(np.uint8)


def _planarity_scale(min_eigs: NDArray[np.float32]) -> NDArray[np.float32]:
    """Arrow length factors, larger for more planar fits."""
    if len(min_eigs) == 0:
        return np.empty(0, np.float32)
    inv = 1.0 / np.maximum(min_eigs, 1e-12)
    return np.clip(inv / np.median(inv), 0.25, 2.0).astype(np.float32)


def _height_colors(centers: NDArray[np.float32], base: list[int]) -> NDArray[np.uint8]:
    """Shade each voxel by height, keeping the method's base hue."""
    if len(centers) == 0:
        return np.empty((0, 3), np.uint8)
    z = centers[:, 2]
    span = float(z.max() - z.min())
    # Only the top half of the brightness scale, so the low end stays visible.
    t = (z - z.min()) / span if span > 1e-6 else np.zeros(len(z), np.float32)
    brightness = 0.5 + 0.5 * t
    return (np.asarray(base, np.float32) * brightness[:, None]).astype(np.uint8)


def _z_gradient(centers: NDArray[np.float32], ramp: NDArray[np.float32]) -> NDArray[np.uint8]:
    """Color each point by height, normalized to the 2nd-98th percentiles
    so stray points cannot compress the cloud into the middle of the ramp."""
    if len(centers) == 0:
        return np.empty((0, 3), np.uint8)
    z = centers[:, 2]
    quantiles = np.percentile(z, [2.0, 98.0])
    z_lo, z_hi = float(quantiles[0]), float(quantiles[1])
    span = z_hi - z_lo
    if span > 1e-6:
        t = np.clip((z - z_lo) / span, 0.0, 1.0)
    else:
        t = np.zeros(len(z), np.float32)
    pos = t * (len(ramp) - 1)
    lo = np.minimum(pos.astype(np.int32), len(ramp) - 2)
    frac = (pos - lo)[:, None]
    mixed: NDArray[np.float32] = ramp[lo] * (1.0 - frac) + ramp[lo + 1] * frac
    return mixed.astype(np.uint8)


def main(
    dataset: str = typer.Argument(..., help="Dataset .db: bare name (cwd or data/) or path"),
    out: Path | None = typer.Option(
        None, "--out", help="Output .rrd path. If omitted, spawn rerun live."
    ),
    lidar_stream: str = typer.Option("pointlio_lidar", "--lidar-stream"),
    world_frame: str = typer.Option(
        "world", "--world-frame", help="Fixed frame clouds are registered in"
    ),
    voxel_size: float = typer.Option(
        DEFAULT_VOXEL_SIZE, "--voxel-size", help="Voxel edge length (m)"
    ),
    fine_divisor: int = typer.Option(
        3,
        "--fine-divisor",
        help="Fine cells per voxel edge; logs the defaults variant's fine map when set. "
        "Zero disables it",
    ),
    max_range: float = typer.Option(30.0, "--max-range", help="Max ray cast distance (m)"),
    emit_every: int = typer.Option(1, "--emit-every", help="Log the maps every N frames"),
    render_voxel: float | None = typer.Option(
        None,
        "--render-voxel",
        help="Voxel render size (m); scales with --voxel-size when unset "
        f"({DEFAULT_RENDER_VOXEL} at the default voxel size of {DEFAULT_VOXEL_SIZE})",
    ),
    normal_scale: float = typer.Option(
        0.08, "--normal-scale", help="Median normal arrow length (m); flatter fits draw longer"
    ),
    fit_normals: bool = typer.Option(
        False,
        "--fit-normals",
        help="Refit normals every emitted frame to scale arrows by planarity. "
        "Whole-map refit cost. Off draws cached normals at a fixed length",
    ),
    from_time: float | None = typer.Option(
        None, "--from-time", help="Start replay at this stream timestamp (s)"
    ),
    viewer_memory: str = typer.Option(
        "25%",
        "--viewer-memory",
        help="Spawned viewer memory limit before it drops old frames, e.g. 4GB or 25%",
    ),
) -> None:
    import rerun as rr

    if render_voxel is None:
        render_voxel = default_render_voxel(voxel_size, DEFAULT_VOXEL_SIZE)

    db_path = resolve_named_path(dataset, ".db")

    rr.init("raytrace_rrd", recording_id=db_path.stem)
    if out is not None:
        rr.save(str(out))
    else:
        rr.spawn(memory_limit=viewer_memory)

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
        "defaults": VoxelRayMapper(
            voxel_size=voxel_size, max_range=max_range, fine_divisor=fine_divisor
        ),
    }

    store = SqliteStore(path=str(db_path))
    with store:
        lidar = store.stream(lidar_stream, PointCloud2).order_by("ts")
        if from_time is not None:
            lidar = lidar.from_time(from_time)
        tf = StreamTF.from_store(store)
        if tf is None:
            raise typer.BadParameter(f"{db_path} has no tf stream to register clouds from")

        trajectory: list[tuple[float, float, float]] = []
        count = 0
        dropped = 0
        for obs in lidar:
            t = tf.get(
                world_frame,
                obs.data.frame_id,
                time_point=obs.ts,
                time_tolerance=TF_MATCH_TOLERANCE_S,
            )
            if t is None:
                dropped += 1
                continue
            x, y, z = float(t.translation.x), float(t.translation.y), float(t.translation.z)
            qx, qy, qz, qw = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
            # Sensor-frame cloud: the mapper registers it by the tf pose.
            raw = obs.data.points_f32()
            for mapper in mappers.values():
                mapper.add_frame(raw, (x, y, z), (qx, qy, qz, qw))
            count += 1

            if count % emit_every != 0:
                continue

            # Both mappers register the same cloud, so any one's copy serves.
            pts = next(iter(mappers.values())).registered_points()

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
                min_eigs: NDArray[np.float32] | None = None
                if fit_normals:
                    centers, normals, min_eigs = mapper.global_map_normal_fits()
                else:
                    centers, normals = mapper.global_map_normals()
                colors = _z_gradient(centers, COARSE_RAMP)
                rr.log(
                    f"world/maps/{name}",
                    rr.Points3D(centers, colors=colors, radii=render_voxel / 2),
                )
                keep = np.any(normals != 0.0, axis=1)
                origins, vectors = centers[keep], normals[keep]
                flip = np.sum(vectors * (robot - origins), axis=1) < 0
                vectors = np.where(flip[:, None], -vectors, vectors)
                if min_eigs is None:
                    lengths = np.full(len(origins), normal_scale, np.float32)
                else:
                    lengths = normal_scale * _planarity_scale(min_eigs[keep])
                rr.log(
                    f"world/maps/{name}/normals",
                    rr.Arrows3D(
                        origins=origins,
                        vectors=vectors * lengths[:, None],
                        colors=_normal_colors(colors[keep]),
                        radii=0.005,
                    ),
                )
            if fine_divisor:
                fine_centers = mappers["defaults"].local_map_fine(
                    (x, y, z), max_range, z - FINE_Z_HALF_EXTENT_M, z + FINE_Z_HALF_EXTENT_M
                )
                rr.log(
                    "world/maps/fine",
                    rr.Points3D(
                        fine_centers,
                        colors=_z_gradient(fine_centers, FINE_RAMP),
                        radii=render_voxel / (2 * fine_divisor),
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
        if dropped:
            print(f"dropped {dropped} clouds with no transform within tolerance")

    if out is not None:
        print(f"wrote {out}\nopen with: rerun {out}")


if __name__ == "__main__":
    typer.run(main)
