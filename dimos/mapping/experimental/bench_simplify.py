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

"""Mesh one recording's whole voxel map with each simplifier chain: time, triangles, error."""

from __future__ import annotations

import re
import time

import numpy as np
import typer

from dimos.mapping.experimental.live_mesh import Chunk, OccupancyMesher
from dimos.mapping.experimental.simplify import parse_chain
from dimos.mapping.ray_tracing.transformer import RayTraceMap, pose_from_tf
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tf import StreamTF
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data


def _merge(chunks: list[Chunk]) -> tuple[np.ndarray, np.ndarray]:
    vs, fs, n = [], [], 0
    for _, v, _, f in chunks:
        vs.append(v)
        fs.append(f.astype(np.int64) + n)
        n += len(v)
    return np.concatenate(vs), np.concatenate(fs)


def _error(ref: tuple[np.ndarray, np.ndarray], mesh: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    """Distance from points sampled on the reference mesh to the simplified one, metres."""
    import open3d as o3d  # type: ignore[import-untyped]

    r = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(ref[0]), o3d.utility.Vector3iVector(ref[1])
    )
    pts = np.asarray(r.sample_points_uniformly(200_000).points, np.float32)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh[0].astype(np.float32), mesh[1].astype(np.uint32))
    return scene.compute_distance(o3d.core.Tensor(pts)).numpy()  # type: ignore[no-any-return]


def _cm(tol: float | None) -> str:
    return "default" if tol is None else f"{tol * 100:g}cm"


def main(
    dataset: str = typer.Argument("mid360_athens_stairs"),
    to_time: float = typer.Option(60.0, "--to-time"),
    voxel_size: float = typer.Option(0.05, "--voxel-size"),
    lidar_stream: str = typer.Option("pointlio_lidar", "--lidar-stream"),
    chains: str = typer.Option(
        "collapse;planes,collapse", "--chains", help="';'-separated simplifier chains"
    ),
    tols: str = typer.Option("", "--tols", help="comma list of metres; empty keeps defaults"),
    rrd: str | None = typer.Option(None, "--rrd", help="write every variant side by side here"),
) -> None:
    db = dataset if dataset.endswith(".db") else str(get_data(f"{dataset}.db"))
    with SqliteStore(path=db) as store:
        tf = StreamTF.from_store(store)
        assert tf is not None
        ray = RayTraceMap(voxel_size=voxel_size, emit_every=0)
        lidar = store.stream(lidar_stream, PointCloud2).order_by("ts").range_time(None, to_time)
        for _ in lidar.transform(pose_from_tf(tf, "odom")).transform(ray):
            pass
    points = ray.mapper.global_map()
    print(f"{len(points)} voxels")

    def run(chain: str, tol: float | None) -> tuple[float, list[Chunk]]:
        sims = parse_chain(chain, tol)
        mesher = OccupancyMesher(voxel_size, 0.2, "cuda", sims)
        mesher.mesh(points)  # warm up kernels
        mesher.hashes = {}
        t = time.perf_counter()
        chunks = mesher.mesh(points)
        return time.perf_counter() - t, [c for c in chunks if len(c[3])]

    if rrd is not None:
        import rerun as rr

        rr.init("bench_simplify")
        rr.save(rrd)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    shown: list[str] = []

    def show(label: str, chunks: list[Chunk]) -> None:
        """Per chunk with normals, as live_mesh_rrd logs it."""
        if rrd is None:
            return
        import matplotlib
        import rerun as rr

        # turbo over the bulk of the model's height, not its outliers
        lo, hi = np.percentile(ref[0][:, 2], [2, 98])
        path = f"world/{len(shown):02d}_" + re.sub(r"[^\w.]+", "_", label)
        for key, v, n, f in chunks:
            z = np.clip((v[:, 2] - lo) / (hi - lo), 0, 1)
            colors = (matplotlib.colormaps["turbo"](z)[:, :3] * 255).astype(np.uint8)
            rr.log(
                f"{path}/{key[0]}_{key[1]}_{key[2]}",
                rr.Mesh3D(
                    vertex_positions=v, vertex_normals=n, triangle_indices=f, vertex_colors=colors
                ),
                static=True,
            )
        shown.append(path)

    dt, ref_chunks = run("", None)
    ref = _merge(ref_chunks)
    print(f"{'none':>18s}            {len(ref[1]):8d} tris {dt * 1e3:7.1f} ms")
    show(f"none {len(ref[1])} tris", ref_chunks)
    for chain in chains.split(";"):
        tol_list: list[float | None] = [float(t) for t in tols.split(",") if t]
        for tol in tol_list or [None]:
            dt, chunks = run(chain, tol)
            mesh = _merge(chunks)
            err = _error(ref, mesh) * 100
            print(
                f"{chain:>18s} tol {_cm(tol)} {len(mesh[1]):8d} tris "
                f"({len(mesh[1]) / len(ref[1]):5.1%}) {dt * 1e3:7.1f} ms   "
                f"err median {np.median(err):.2f} p99 {np.percentile(err, 99):.2f} "
                f"max {err.max():.2f} cm"
            )
            show(
                f"{chain} tol {_cm(tol)} {len(mesh[1])} tris p99 {np.percentile(err, 99):.1f}cm",
                chunks,
            )

    if rrd is not None:
        import rerun as rr
        import rerun.blueprint as rrb

        hidden = {p: rrb.EntityBehavior(visible=False) for p in shown[1:]}
        rr.send_blueprint(rrb.Spatial3DView(origin="world", overrides=hidden))  # type: ignore[arg-type]


if __name__ == "__main__":
    typer.run(main)
