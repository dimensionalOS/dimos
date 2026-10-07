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

"""Replay lidar through the ray-traced voxel map and LiveMesh, faster than realtime, into an .rrd."""

from __future__ import annotations

from collections.abc import Iterator
import itertools

import numpy as np
import typer

from dimos.mapping.experimental.live_mesh import LiveMesh
from dimos.mapping.experimental.simplify import parse_chain
from dimos.mapping.ray_tracing.transformer import RayTraceMap, pose_from_tf
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tf import StreamTF
from dimos.memory.transform import FnTransformer
from dimos.memory.type.observation import Observation
from dimos.memory.utils.progress import progress
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data

RECOLOUR_M = 0.5  # re-colour every chunk once the height range moves this much


def main(
    dataset: str = typer.Argument("mid360_athens_stairs", help="recording name in data/ or a .db"),
    out: str | None = typer.Option(None, "--out", help=".rrd to write; without it a viewer opens"),
    live: bool = typer.Option(
        True, "--live/--no-live", help="open a viewer as well as writing --out"
    ),
    lidar_stream: str = typer.Option("pointlio_lidar", "--lidar-stream"),
    world_frame: str = typer.Option("odom", "--world-frame"),
    voxel_size: float = typer.Option(0.05, "--voxel-size"),
    every: int = typer.Option(100, "--every", help="lidar frames per mesh update"),
    from_time: float | None = typer.Option(None, "--from-time"),
    to_time: float | None = typer.Option(None, "--to-time"),
    simplify: bool = typer.Option(True, "--simplify/--no-simplify", help="simplify with --chain"),
    chain: str = typer.Option(
        "planes,collapse", "--chain", help="simplifiers in order, name[:tol][:knob=x]"
    ),
    tol: float | None = typer.Option(
        None, "--tol", help="error bound for every simplifier, metres"
    ),
    static: bool = typer.Option(
        True, "--static/--timeline", help="keep only the latest mesh, or every version to scrub"
    ),
    alpha: float = typer.Option(0.4, "--alpha", help="mesh opacity, 0 to 1"),
    show_lidar: bool = typer.Option(False, "--lidar", help="also log every lidar scan"),
    camera_stream: str = typer.Option(
        "color_image", "--camera-stream", help="shown beside the 3D view when present"
    ),
    z_min: float | None = typer.Option(None, "--z-min", help="fix the turbo bottom, metres"),
    z_max: float | None = typer.Option(None, "--z-max", help="fix the turbo top, metres"),
) -> None:
    import matplotlib
    import rerun as rr

    turbo = (matplotlib.colormaps["turbo"](np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)
    simplifiers = parse_chain(chain, tol) if simplify else []
    albedo = [255, 255, 255, int(alpha * 255)]
    trail: list[tuple[float, float, float]] = []

    def log_odom(obs: Observation[PointCloud2]) -> None:
        if obs.pose_tuple is None:
            return
        trail.append(obs.pose_tuple[:3])
        rr.set_time("time", timestamp=obs.ts)
        rr.log("world/odom", rr.Points3D([trail[-1]], radii=0.12, colors=[0, 255, 0]))
        if len(trail) % every == 0:
            rr.log("world/odom/trail", rr.LineStrips3D([trail], colors=[0, 255, 0]))
        if show_lidar:
            x, y, z, qx, qy, qz, qw = obs.pose_tuple
            rr.log(
                "world/lidar",
                rr.Transform3D(
                    translation=[x, y, z], quaternion=rr.Quaternion(xyzw=[qx, qy, qz, qw])
                ),
            )
            rr.log(
                "world/lidar/scan",
                rr.Points3D(obs.data.points_f32(), radii=0.01, colors=[255, 255, 255]),
            )

    db = dataset if dataset.endswith(".db") else str(get_data(f"{dataset}.db"))
    rr.init("live_mesh")
    if out is not None and live:
        rr.spawn(connect=False)
        rr.set_sinks(rr.GrpcSink(), rr.FileSink(out))
    elif out is not None:
        rr.save(out)
    else:
        rr.spawn()
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    with SqliteStore(path=db) as store:
        tf = StreamTF.from_store(store)
        if tf is None:
            raise typer.BadParameter(f"{db} has no tf stream to register clouds from")
        ray = RayTraceMap(voxel_size=voxel_size, emit_every=every)
        lidar = (
            store.stream(lidar_stream, PointCloud2).order_by("ts").range_time(from_time, to_time)
        )
        camera: Iterator[Observation[Image]] = iter(())
        if camera_stream in store.list_streams():
            import rerun.blueprint as rrb

            camera = iter(
                store.stream(camera_stream, Image).order_by("ts").range_time(from_time, to_time)
            )
            rr.send_blueprint(
                rrb.Horizontal(
                    rrb.Spatial3DView(origin="world"), rrb.Spatial2DView(origin="camera")
                )
            )
        pending: list[Observation[Image]] = []

        def log_camera(obs: Observation[PointCloud2]) -> None:
            # images up to this scan's time, so both streams stay in step
            for img in itertools.chain(pending, camera):
                if img.ts > obs.ts:
                    pending[:] = [img]
                    return
                rr.set_time("time", timestamp=img.ts)
                rr.log("camera/image", img.data.to_rerun())
            pending.clear()

        map_z: list[float] = [0.0, 1.0]

        def to_global_map(o: Observation[PointCloud2]) -> Observation[PointCloud2]:
            points = ray.mapper.global_map()
            if len(points):
                # turbo over the bulk of the map's height, not its outliers
                map_z[:] = np.percentile(points[:, 2], [2, 98]).tolist()
            return o.derive(data=PointCloud2.from_numpy(points))

        global_map: FnTransformer[PointCloud2, PointCloud2] = FnTransformer(to_global_map)
        with progress(lidar.count(), "meshing") as bar:
            meshes = (
                lidar.tap(bar)
                .tap(log_camera)
                .transform(pose_from_tf(tf, world_frame))
                .tap(log_odom)
                .transform(ray)
                .transform(global_map)
                .transform(LiveMesh(voxel_size=voxel_size, simplify=simplifiers))
            )
            # colours follow the map's height range; when it moves, every chunk is
            # re-coloured (colours only, geometry stays)
            heights: dict[tuple[int, int, int], np.ndarray] = {}
            col_lo, col_hi = 0.0, 0.0

            def colour(path: str, z: np.ndarray) -> None:
                t = np.clip((z - col_lo) / max(col_hi - col_lo, 1e-6), 0, 1)
                rr.log(
                    path,
                    rr.Mesh3D.from_fields(
                        vertex_colors=turbo[(t * 255).astype(int)], albedo_factor=albedo
                    ),
                    static=static,
                )

            for obs in meshes:
                m = obs.data
                rr.set_time("time", timestamp=obs.ts)
                path = f"world/mesh/{m.key[0]}_{m.key[1]}_{m.key[2]}"
                rr.log(path, m.to_rerun(), static=static)
                if not len(m.faces):
                    heights.pop(m.key, None)
                    continue
                heights[m.key] = m.vertices[:, 2].copy()
                lo = z_min if z_min is not None else map_z[0]
                hi = z_max if z_max is not None else map_z[1]
                if abs(lo - col_lo) > RECOLOUR_M or abs(hi - col_hi) > RECOLOUR_M:
                    col_lo, col_hi = lo, hi
                    for k, z in heights.items():
                        colour(f"world/mesh/{k[0]}_{k[1]}_{k[2]}", z)
                else:
                    colour(path, heights[m.key])
        # a half-read cursor must not outlive the store
        getattr(camera, "close", lambda: None)()
    if out is not None:
        print(f"-> {out}")


if __name__ == "__main__":
    typer.run(main)
