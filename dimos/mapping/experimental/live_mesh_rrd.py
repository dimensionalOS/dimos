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
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data


def main(
    dataset: str = typer.Argument("mid360_athens_stairs", help="recording name in data/ or a .db"),
    out: str | None = typer.Option(None, "--out", help=".rrd to write; without it a viewer opens"),
    live: bool = typer.Option(False, "--live", help="open a viewer as well as writing --out"),
    lidar_stream: str = typer.Option("pointlio_lidar", "--lidar-stream"),
    world_frame: str = typer.Option("odom", "--world-frame"),
    voxel_size: float = typer.Option(0.05, "--voxel-size"),
    every: int = typer.Option(10, "--every", help="lidar frames per mesh update"),
    from_time: float | None = typer.Option(None, "--from-time"),
    to_time: float | None = typer.Option(None, "--to-time"),
    simplify: str = typer.Option(
        "", "--simplify", help="run in order, name[:tol][:knob=x]: e.g. planes,collapse"
    ),
    tol: float | None = typer.Option(
        None, "--tol", help="error bound for every simplifier, metres"
    ),
    static: bool = typer.Option(
        True, "--static/--timeline", help="keep only the latest mesh, or every version to scrub"
    ),
    alpha: float = typer.Option(1.0, "--alpha", help="mesh opacity, 0 to 1"),
    z_min: float = typer.Option(-2.0, "--z-min", help="turbo colormap bottom"),
    z_max: float = typer.Option(4.0, "--z-max", help="turbo colormap top"),
) -> None:
    import matplotlib
    import rerun as rr

    turbo = (matplotlib.colormaps["turbo"](np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)
    simplifiers = parse_chain(simplify, tol)
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
        global_map: FnTransformer[PointCloud2, PointCloud2] = FnTransformer(
            lambda o: o.derive(data=PointCloud2.from_numpy(ray.mapper.global_map()))
        )
        with progress(lidar.count(), "meshing") as bar:
            meshes = (
                lidar.tap(bar)
                .transform(pose_from_tf(tf, world_frame))
                .tap(log_odom)
                .transform(ray)
                .transform(global_map)
                .transform(LiveMesh(voxel_size=voxel_size, simplify=simplifiers))
            )
            for obs in meshes:
                m = obs.data
                z = (m.vertices[:, 2] - z_min) / (z_max - z_min)
                rr.set_time("time", timestamp=obs.ts)
                path = f"world/mesh/{m.key[0]}_{m.key[1]}_{m.key[2]}"
                rr.log(path, m.to_rerun(), static=static)
                if len(m.faces):
                    idx = (np.clip(z, 0, 1) * 255).astype(int)
                    rr.log(
                        path,
                        rr.Mesh3D.from_fields(vertex_colors=turbo[idx], albedo_factor=albedo),
                        static=static,
                    )
    if out is not None:
        print(f"-> {out}")


if __name__ == "__main__":
    typer.run(main)
