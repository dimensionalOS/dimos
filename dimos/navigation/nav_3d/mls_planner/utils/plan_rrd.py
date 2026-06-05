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
import re
from typing import TYPE_CHECKING

import rerun as rr
import typer

from dimos.mapping.ray_tracing.transformer import RayTraceMap
from dimos.memory2.store.memory import MemoryStore
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.stream import Stream
from dimos.memory2.transform import FnTransformer
from dimos.memory2.type.observation import Observation
from dimos.memory2.vis.plot.plot import Plot
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

TIMING_KEYS = ["voxelize_ms", "surfaces_ms", "graph_ms", "plan_ms", "total_ms"]
SIZE_KEYS = ["voxels", "surface_cells", "nodes", "edges"]


def _print_summary(streams: dict[str, dict[str, Stream[float]]]) -> None:
    print("\nper-frame summary (mean / p50 / p95 / max):")
    for kind, by_key in streams.items():
        for key, stream in by_key.items():
            values = [obs.data for obs in stream]
            if not values:
                continue
            arr = np.asarray(values, dtype=np.float64)
            mean, p50, p95, peak = (
                arr.mean(),
                np.percentile(arr, 50),
                np.percentile(arr, 95),
                arr.max(),
            )
            print(f"  {kind}/{key:<14} {mean:9.2f} {p50:9.2f} {p95:9.2f} {peak:9.2f}")


def _stitch_svgs(svgs: list[str]) -> str:
    """Stack standalone SVGs vertically into one. Namespaces each panel's ids
    so matplotlib's reused ids (axes_1, clip paths, glyphs) don't collide."""
    panels: list[str] = []
    widths: list[float] = []
    offset = 0.0
    for i, svg in enumerate(svgs):
        body = svg[svg.index("<svg") :]
        m = re.search(r'width="([\d.]+)pt"\s+height="([\d.]+)pt"', body)
        if m is None:
            raise ValueError("could not parse SVG dimensions")
        width, height = float(m.group(1)), float(m.group(2))
        prefix = f"s{i}_"
        body = re.sub(r'id="([^"]+)"', rf'id="{prefix}\1"', body)
        body = re.sub(r"url\(#([^)]+)\)", rf"url(#{prefix}\1)", body)
        body = re.sub(r'xlink:href="#([^"]+)"', rf'xlink:href="#{prefix}\1"', body)
        # Drop the "pt" unit so the nested width/height are read as parent user
        # units (the outer viewBox is unitless); otherwise pt->px conversion
        # overflows the parent viewport and clips each panel on the right.
        body = body.replace(m.group(0), f'width="{width}" height="{height}"', 1)
        body = body.replace("<svg", f'<svg x="0" y="{offset}"', 1)
        panels.append(body)
        widths.append(width)
        offset += height
    return (
        '<?xml version="1.0" encoding="utf-8" standalone="no"?>\n'
        '<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" '
        f'width="{max(widths)}pt" height="{offset}pt" '
        f'viewBox="0 0 {max(widths)} {offset}" version="1.1">\n' + "\n".join(panels) + "\n</svg>\n"
    )


def _save_plot(timing_streams: dict[str, Stream[float]], voxels: Stream[float], path: str) -> None:
    panels: list[str] = []
    for key in TIMING_KEYS:
        plot = Plot()
        plot.add(timing_streams[key], label=key, connect=None)
        panels.append(plot.to_svg())
    voxel_plot = Plot()
    voxel_plot.add(voxels, label="voxels", connect=None)
    panels.append(voxel_plot.to_svg())
    with open(path, "w") as f:
        f.write(_stitch_svgs(panels))
    print(f"wrote {path}")


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
    plot_out: FsPath | None = typer.Option(
        None, "--plot-out", help="Write an SVG timing/size plot here when the run ends"
    ),
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
        lidar = store.stream(lidar_stream, PointCloud2).order_by("ts").from_time(110).to_time(120)
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

        metrics = MemoryStore()
        timing_streams = {k: metrics.stream(f"timing_{k}", float) for k in TIMING_KEYS}
        size_streams = {k: metrics.stream(f"size_{k}", float) for k in SIZE_KEYS}

        try:
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

                edges = obs.tags["node_edges"]
                _log_edges(edges, "world/node_edges")
                _log_path(obs.data, "world/path")

                timings = obs.tags["timings"]
                sizes = {
                    "voxels": obs.tags["voxels"],
                    "surface_cells": len(surface),
                    "nodes": len(nodes),
                    "edges": len(edges),
                }
                for key, value in timings.items():
                    timing_streams[key].append(float(value), ts=obs.ts)
                    rr.log(f"metrics/timing/{key}", rr.Scalars(value))
                for key, value in sizes.items():
                    size_streams[key].append(float(value), ts=obs.ts)
                    rr.log(f"metrics/size/{key}", rr.Scalars(value))

                count = obs.tags.get("frame_count", "?")
                planned = obs.tags.get("planned", False)
                print(
                    f"frame_count={count} planned={planned} "
                    f"waypoints={len(obs.data.poses)} "
                    f"rebuild={timings['total_ms'] - timings['plan_ms']:.1f}ms "
                    f"plan={timings['plan_ms']:.1f}ms",
                    end="\r",
                    flush=True,
                )
        except KeyboardInterrupt:
            print("\ninterrupted; reporting metrics for completed frames")
        finally:
            _print_summary({"timing": timing_streams, "size": size_streams})
            if plot_out is not None:
                _save_plot(timing_streams, size_streams["voxels"], str(plot_out))

    if out is not None:
        print(f"wrote {out}")
        print(f"open with: rerun {out}")


if __name__ == "__main__":
    typer.run(main)
