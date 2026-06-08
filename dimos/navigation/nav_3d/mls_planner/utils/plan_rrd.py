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

"""Dev replay util: run a lidar+odometry .db through RayTraceMap and MLSPlan into rerun."""

from __future__ import annotations

from pathlib import Path as FsPath
from typing import TYPE_CHECKING

import rerun as rr
import typer

from dimos.mapping.ray_tracing.transformer import RayTraceMap
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import FnTransformer
from dimos.memory2.type.observation import Observation
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2, register_colormap_annotation
from dimos.navigation.nav_3d.mls_planner.transformer import MLSPlan
from dimos.utils.data import resolve_named_path

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

TIMELINE = "ts"

PairObs = Observation[tuple[Observation[PointCloud2], Observation[Odometry]]]


def _attach_pose_from_odom(pair_obs: PairObs) -> Observation[PointCloud2]:
    lidar_obs = pair_obs.data[0]
    odom_obs = pair_obs.data[1]
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


def _log_edges(edges: NDArray[np.float32], entity: str) -> None:
    if edges.size == 0:
        rr.log(entity, rr.LineStrips3D([]))
        return
    segments = [
        [(float(r[0]), float(r[1]), float(r[2])), (float(r[3]), float(r[4]), float(r[5]))]
        for r in edges
    ]
    rr.log(entity, rr.LineStrips3D(segments))


def _log_path(path: Path, entity: str) -> None:
    if not path.poses:
        rr.log(entity, rr.LineStrips3D([]))
        return
    points = [(float(p.position.x), float(p.position.y), float(p.position.z)) for p in path.poses]
    rr.log(entity, rr.LineStrips3D([points], colors=[[0, 255, 0]], radii=0.05))


def main(
    dataset: str = typer.Argument(..., help="Dataset .db: bare name (cwd or data/) or path"),
    out: FsPath | None = typer.Option(
        None, "--out", help="Output .rrd path. If omitted, spawn rerun live."
    ),
    lidar_stream: str = typer.Option(
        "fastlio_lidar", "--lidar-stream", help="Lidar stream in the recording"
    ),
    odom_stream: str = typer.Option(
        "fastlio_odometry", "--odom-stream", help="Odometry stream in the recording"
    ),
    align_tol: float = typer.Option(0.05, "--align-tol", help="Lidar/odom alignment tolerance (s)"),
    voxel_size: float = typer.Option(0.1, "--voxel-size", help="Voxel edge length (m)"),
    max_range: float = typer.Option(30.0, "--max-range", help="Max ray cast distance (m)"),
    ray_subsample: int = typer.Option(1, "--ray-subsample", help="Keep every Nth ray"),
    emit_every: int = typer.Option(1, "--emit-every", help="Replan every N lidar frames"),
    robot_height: float = typer.Option(0.3, "--robot-height", help="Robot height (m)"),
    node_spacing: float = typer.Option(1.0, "--node-spacing", help="Graph node spacing (m)"),
    goal: tuple[float, float, float] = typer.Option(
        (0.0, 0.0, 0.0),
        "--goal",
        help="Planner goal xyz. Default is dataset-specific; override per recording.",
    ),
    live: bool = typer.Option(
        False, "--live", help="Also spawn the rerun viewer when --out is set"
    ),
    render_voxel: float = typer.Option(0.05, "--render-voxel", help="Rerun voxel render size (m)"),
) -> None:
    db_path = resolve_named_path(dataset, ".db")

    rr.init("plan_rrd", recording_id=db_path.stem)
    if out is not None and live:
        # Generous viewer memory so the gRPC sink never backpressures the writer.
        rr.spawn(connect=False, memory_limit="16GB", server_memory_limit="16GB")
        rr.set_sinks(rr.GrpcSink(), rr.FileSink(str(out)))
    elif out is not None:
        rr.save(str(out))
    else:
        rr.spawn()
    register_colormap_annotation("turbo")

    store = SqliteStore(path=str(db_path))
    with store:
        lidar = store.stream(lidar_stream, PointCloud2).order_by("ts")
        odom = store.stream(odom_stream, Odometry).order_by("ts")

        pose_tagged = lidar.align(odom, tolerance=align_tol).transform(
            FnTransformer(_attach_pose_from_odom)
        )
        pipeline = pose_tagged.transform(
            RayTraceMap(
                voxel_size=voxel_size,
                max_range=max_range,
                ray_subsample=ray_subsample,
                emit_every=emit_every,
            )
        ).transform(
            MLSPlan(
                goal=goal,
                voxel_size=voxel_size,
                robot_height=robot_height,
                node_spacing_m=node_spacing,
            )
        )

        rr.log("world/goal", rr.Points3D([goal], colors=[[255, 0, 0]], radii=0.1), static=True)

        for obs in pipeline:
            rr.set_time(TIMELINE, timestamp=obs.ts)

            start = obs.tags["start"]
            rr.log("world/start", rr.Points3D([start], colors=[[0, 255, 0]], radii=0.1))

            voxel_map = obs.tags["voxel_map"]
            rr.log("world/voxel_map", voxel_map.to_rerun(voxel_size=render_voxel))

            surface = obs.tags["surface_map"]
            if surface.size:
                rr.log(
                    "world/surface_map",
                    rr.Points3D(surface, colors=[[120, 120, 200]], radii=render_voxel / 2),
                )

            nodes = obs.tags["nodes"]
            if nodes.size:
                rr.log("world/nodes", rr.Points3D(nodes, colors=[[255, 200, 0]], radii=0.05))

            _log_edges(obs.tags["node_edges"], "world/node_edges")
            _log_path(obs.data, "world/path")

            count = obs.tags.get("frame_count", "?")
            planned = obs.tags.get("planned", False)
            print(
                f"frame_count={count} planned={planned} waypoints={len(obs.data.poses)}",
                end="\r",
                flush=True,
            )
        print()

    if out is not None:
        print(f"wrote {out}")
        print(f"open with: rerun {out}")


if __name__ == "__main__":
    typer.run(main)
