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

"""Replay lidar through the ray-traced voxel map and Mesh into rerun, faster than realtime."""

from __future__ import annotations

from collections.abc import Iterator
import itertools

import typer

from dimos.mapping.experimental.mesh import Mesh, MeshColours
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
    trail_every: int = typer.Option(10, "--trail-every", help="lidar frames per path update"),
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
    z_min: float | None = typer.Option(None, "--z-min", help="pin the turbo bottom, metres"),
    z_max: float | None = typer.Option(None, "--z-max", help="pin the turbo top, metres"),
) -> None:
    import rerun as rr

    simplifiers = parse_chain(chain, tol) if simplify else []
    z_range = (z_min, z_max) if z_min is not None and z_max is not None else None
    colours = MeshColours(alpha=alpha, z_range=z_range, static=static)
    trail: list[tuple[float, float, float]] = []

    def log_odom(obs: Observation[PointCloud2]) -> None:
        if obs.pose_tuple is None:
            return
        trail.append(obs.pose_tuple[:3])
        rr.set_time("time", timestamp=obs.ts)
        rr.log("world/odom", rr.Points3D([trail[-1]], radii=0.12, colors=[0, 255, 0]))
        if len(trail) % trail_every == 0:
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
    rr.init("mesh")
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
            # log images up to this scan's time
            for img in itertools.chain(pending, camera):
                if img.ts > obs.ts:
                    pending[:] = [img]
                    return
                rr.set_time("time", timestamp=img.ts)
                rr.log("camera/image", img.data.to_rerun())
            pending.clear()

        global_map: FnTransformer[PointCloud2, PointCloud2] = FnTransformer(
            lambda o: o.derive(data=PointCloud2.from_numpy(ray.mapper.global_map()))
        )
        with progress(lidar.count(), "meshing") as bar:
            meshes = (
                lidar.tap(bar)
                .tap(log_camera)
                .transform(pose_from_tf(tf, world_frame))
                .tap(log_odom)
                .transform(ray)
                .transform(global_map)
                .transform(Mesh(voxel_size=voxel_size, simplify=simplifiers))
            )
            for obs in meshes:
                rr.set_time("time", timestamp=obs.ts)
                for entry in colours(obs.data):
                    rr.log(entry.path, entry.archetype, static=entry.static)
        # close the camera cursor before the store
        getattr(camera, "close", lambda: None)()
    if out is not None:
        print(f"-> {out}")


if __name__ == "__main__":
    typer.run(main)
