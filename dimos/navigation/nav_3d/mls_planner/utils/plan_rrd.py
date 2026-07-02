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

"""Replay a lidar+odometry .db through RayTraceMap and the MLS planner into rerun.

Pass one or more --config clearance,buffer,weight to overlay each as a colored path.
"""

from __future__ import annotations

from pathlib import Path as FsPath
from time import perf_counter

import numpy as np
from numpy.typing import NDArray
import rerun as rr
import typer

from dimos.mapping.ray_tracing.transformer import RayTraceMap
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import FnTransformer
from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2, register_colormap_annotation
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.nav_3d.mls_planner.mls_planner import MLSPlanner
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.utils.data import resolve_named_path

TIMELINE = "ts"

# Body-frame axis-triad length for the odometry transform (m).
ODOM_AXIS_LEN = 0.5

# Mount frames as recorded on the tf stream.
BASE_FRAME = "base_link"
SENSOR_FRAME = "mid360_link"

# Distinct path colors for overlaid configurations, config 0 first.
PATH_PALETTE = [
    [0, 255, 0],
    [255, 0, 255],
    [0, 200, 255],
    [255, 180, 0],
    [255, 80, 80],
    [160, 120, 255],
    [120, 255, 200],
    [255, 255, 120],
]


def _parse_configs(
    specs: list[str] | None,
    clearance: float,
    buffer: float,
    weight: float,
) -> list[tuple[float, float, float]]:
    """Each spec is 'clearance,buffer,weight'. Falls back to the single flags."""
    if not specs:
        return [(clearance, buffer, weight)]
    out: list[tuple[float, float, float]] = []
    for spec in specs:
        parts = spec.replace(" ", "").split(",")
        if len(parts) != 3:
            raise typer.BadParameter(f"--config must be 'clearance,buffer,weight'; got {spec!r}")
        c, b, w = (float(p) for p in parts)
        out.append((c, b, w))
    return out


PairObs = Observation[tuple[Observation[PointCloud2], Observation[Odometry]]]


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


def _log_edges(edges: NDArray[np.float32], entity: str) -> None:
    if edges.size == 0:
        rr.log(entity, rr.LineStrips3D([]))
        return
    segments = [
        [(float(r[0]), float(r[1]), float(r[2])), (float(r[3]), float(r[4]), float(r[5]))]
        for r in edges
    ]
    rr.log(entity, rr.LineStrips3D(segments))


def _log_path_wp(waypoints: NDArray[np.float32] | None, entity: str, color: list[int]) -> None:
    if waypoints is None or len(waypoints) == 0:
        rr.log(entity, rr.LineStrips3D([]))
        return
    points = [(float(p[0]), float(p[1]), float(p[2])) for p in waypoints]
    rr.log(entity, rr.LineStrips3D([points], colors=[color], radii=0.05))


def _base_from_body(store: SqliteStore) -> Transform | None:
    """The body -> base_link transform from the recording's static mount frames.

    Treats the LIO body frame as the sensor frame. The few-cm IMU-to-lidar
    extrinsic is invisible at triad scale.
    """
    buffer = MultiTBuffer()
    try:
        for i, obs in enumerate(store.stream("tf", TFMessage).order_by("ts")):
            buffer.receive_transform(*obs.data.transforms)
            if i >= 20:
                break
    except Exception as e:
        print(f"no usable tf stream in the recording ({e}); skipping the base_link triad")
        return None
    return buffer.get(SENSOR_FRAME, BASE_FRAME)


def _log_odometry(
    pose: tuple[float, ...],
    ts: float,
    trail: list[tuple[float, float, float]],
    base_from_body: Transform | None,
) -> None:
    """Log the odometry pose as a moving body-frame transform with an XYZ axis
    triad, plus the trajectory trail growing over time. The triad is a static
    child of world/odom, so it inherits this transform and sweeps along the path."""
    px, py, pz, qx, qy, qz, qw = pose
    rr.set_time(TIMELINE, timestamp=ts)
    rr.log(
        "world/odom",
        rr.Transform3D(translation=[px, py, pz], quaternion=rr.Quaternion(xyzw=[qx, qy, qz, qw])),
    )
    trail.append((px, py, pz))
    if len(trail) > 1:
        rr.log("world/odom_path", rr.LineStrips3D([trail], colors=[[255, 255, 255]], radii=0.015))
    if base_from_body is None:
        return
    body = Transform(
        translation=Vector3(px, py, pz),
        rotation=Quaternion(qx, qy, qz, qw),
        frame_id="world",
        child_frame_id=base_from_body.frame_id,
        ts=ts,
    )
    base = body + base_from_body
    rr.log(
        "world/base_link",
        rr.Transform3D(
            translation=[base.translation.x, base.translation.y, base.translation.z],
            quaternion=rr.Quaternion(
                xyzw=[base.rotation.x, base.rotation.y, base.rotation.z, base.rotation.w]
            ),
        ),
    )


def _clearance_colors(clearance: NDArray[np.float32], clamp_m: float) -> NDArray[np.uint8]:
    """Map per-cell wall clearance to a blue ramp, clamped so it resolves near walls."""
    norm = np.clip(np.nan_to_num(clearance / clamp_m, nan=1.0, posinf=1.0), 0.0, 1.0)
    blocked = np.array([4.0, 8.0, 48.0], dtype=np.float64)
    clear = np.array([150.0, 200.0, 255.0], dtype=np.float64)
    rgb: NDArray[np.float64] = blocked + norm[:, None] * (clear - blocked)
    return rgb.astype(np.uint8)


def _log_shared(
    start: tuple[float, float, float],
    planner: MLSPlanner,
    render_voxel: float,
    clearance_clamp: float,
) -> tuple[NDArray[np.float32], NDArray[np.float32], NDArray[np.float32]]:
    """Log the map artifacts shared by every config from a reference planner.

    Returns (surface, nodes, edges) for metric sizing.
    """
    rr.log("world/start", rr.Points3D([start], colors=[[0, 255, 0]], radii=0.1))

    voxel_map = planner.voxel_map()
    if voxel_map.size:
        rr.log(
            "world/voxel_map",
            rr.Points3D(voxel_map, colors=[[180, 125, 125]], radii=render_voxel / 2),
        )

    surface = planner.surface_clearance_map()
    if surface.size:
        rr.log(
            "world/surface_map",
            rr.Points3D(
                surface[:, :3],
                colors=_clearance_colors(surface[:, 3], clearance_clamp),
                radii=render_voxel / 2,
            ),
        )

    nodes = planner.nodes()
    if nodes.size:
        rr.log("world/nodes", rr.Points3D(nodes, colors=[[255, 200, 0]], radii=0.05))

    edges = planner.node_edges()
    _log_edges(edges, "world/node_edges")
    return surface, nodes, edges


def _init_recording(db_path: FsPath, out: FsPath | None, live: bool) -> None:
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


def _build_planners(
    configs: list[tuple[float, float, float]],
    voxel_size: float,
    robot_height: float,
    surface_closing_radius: float,
    node_spacing: float,
    step_height: float,
    step_penalty_weight: float,
) -> list[tuple[str, list[int], MLSPlanner]]:
    planners: list[tuple[str, list[int], MLSPlanner]] = []
    for i, (clr, buf, wgt) in enumerate(configs):
        planner = MLSPlanner(
            voxel_size=voxel_size,
            robot_height=robot_height,
            surface_closing_radius=surface_closing_radius,
            node_spacing_m=node_spacing,
            wall_clearance_m=clr,
            wall_buffer_m=buf,
            wall_buffer_weight=wgt,
            step_threshold_m=step_height,
            step_penalty_weight=step_penalty_weight,
        )
        color = PATH_PALETTE[i % len(PATH_PALETTE)]
        label = f"cfg{i}_c{clr:g}_b{buf:g}_w{wgt:g}"
        planners.append((label, color, planner))
        print(f"config {i}: clearance={clr} buffer={buf} weight={wgt} color={color} -> {label}")
    return planners


def _process_frame(
    ray_obs: Observation[PointCloud2],
    planners: list[tuple[str, list[int], MLSPlanner]],
    goal: tuple[float, float, float],
    robot_height: float,
    render_voxel: float,
    clearance_clamp: float,
) -> dict[str, float]:
    """Plan every config for one frame, log paths/map/metrics, return the ref timing."""
    assert ray_obs.pose_tuple is not None
    bounds = ray_obs.tags["region_bounds"]
    px, py, pz, *_ = ray_obs.pose_tuple
    start = (float(px), float(py), float(pz) - robot_height)
    ox, oy, radius, z_min, z_max = bounds
    pts = ray_obs.data.points_f32()
    rr.set_time(TIMELINE, timestamp=ray_obs.ts)

    ref_timing: dict[str, float] = {}
    surface = nodes = edges = np.empty((0,), dtype=np.float32)
    for j, (label, color, planner) in enumerate(planners):
        t0 = perf_counter()
        planner.update_region(pts, (ox, oy), radius, z_min, z_max)
        t1 = perf_counter()
        waypoints = planner.plan(start, goal)
        t2 = perf_counter()
        _log_path_wp(waypoints, f"world/paths/{label}", color)
        if j == 0:
            ref_timing = {
                "update_ms": (t1 - t0) * 1000,
                "plan_ms": (t2 - t1) * 1000,
                "total_ms": (t2 - t0) * 1000,
            }
            surface, nodes, edges = _log_shared(start, planner, render_voxel, clearance_clamp)

    for key, value in ref_timing.items():
        rr.log(f"metrics/timing/{key}", rr.Scalars(value))
    sizes = {
        "voxels": planners[0][2].voxel_count(),
        "surface_cells": len(surface),
        "nodes": len(nodes),
        "edges": len(edges),
    }
    for key, value in sizes.items():
        rr.log(f"metrics/size/{key}", rr.Scalars(value))
    return ref_timing


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
    robot_height: float = typer.Option(1.0, "--robot-height", help="Robot height (m)"),
    surface_closing_radius: float = typer.Option(
        0.3,
        "--surface-closing-radius",
        help="Hole-fill radius (m); morphological closing fills holes up to twice this wide",
    ),
    node_spacing: float = typer.Option(1.0, "--node-spacing", help="Graph node spacing (m)"),
    wall_clearance: float = typer.Option(
        0.3,
        "--wall-clearance",
        help="Hard clearance; cells closer to a wall or edge are impassable (m)",
    ),
    wall_buffer: float = typer.Option(
        0.75, "--wall-buffer", help="Width of the soft standoff zone beyond clearance (m)"
    ),
    wall_buffer_weight: float = typer.Option(
        100.0, "--wall-buffer-weight", help="Peak soft wall penalty at the clearance edge"
    ),
    step_height: float = typer.Option(
        0.25,
        "--step-height",
        help="Max traversable vertical step (m); taller steps are impassable",
    ),
    step_penalty_weight: float = typer.Option(
        4.0, "--step-penalty-weight", help="Soft cost per meter of vertical climb"
    ),
    config: list[str] = typer.Option(
        None,
        "--config",
        help="Repeatable 'clearance,buffer,weight' to overlay as colored paths; "
        "overrides the single --wall-* flags",
    ),
    goal: tuple[float, float, float] = typer.Option(
        (0.0, 0.0, 0.0), "--goal", help="Planner goal xyz; override per recording"
    ),
    live: bool = typer.Option(
        False, "--live", help="Also spawn the rerun viewer when --out is set"
    ),
    render_voxel: float = typer.Option(0.05, "--render-voxel", help="Rerun voxel render size (m)"),
    clearance_clamp: float = typer.Option(
        1.0, "--clearance-clamp", help="Max clearance (m) for the surface color scale"
    ),
    from_time: float | None = typer.Option(
        None, "--from-time", help="Start timestamp (s); default is the stream start"
    ),
    to_time: float | None = typer.Option(
        None, "--to-time", help="End timestamp (s); default is the stream end"
    ),
) -> None:
    db_path = resolve_named_path(dataset, ".db")
    _init_recording(db_path, out, live)

    store = SqliteStore(path=str(db_path))
    with store:
        lidar = store.stream(lidar_stream, PointCloud2).order_by("ts")
        if from_time is not None:
            lidar = lidar.from_time(from_time)
        if to_time is not None:
            lidar = lidar.to_time(to_time)
        odom = store.stream(odom_stream, Odometry).order_by("ts")

        pose_tagged = lidar.align(odom, tolerance=align_tol).transform(
            FnTransformer(_attach_pose_from_odom)
        )
        ray_pipeline = pose_tagged.transform(
            RayTraceMap(
                voxel_size=voxel_size,
                max_range=max_range,
                ray_subsample=ray_subsample,
                emit_every=emit_every,
            )
        )

        configs = _parse_configs(config, wall_clearance, wall_buffer, wall_buffer_weight)
        planners = _build_planners(
            configs,
            voxel_size,
            robot_height,
            surface_closing_radius,
            node_spacing,
            step_height,
            step_penalty_weight,
        )

        rr.log("world/goal", rr.Points3D([goal], colors=[[255, 0, 0]], radii=0.1), static=True)

        # Static XYZ axis triads in the odometry body frame and the derived
        # robot base frame.
        base_from_body = _base_from_body(store)
        entities = ["world/odom/axes"] + (["world/base_link/axes"] if base_from_body else [])
        for entity in entities:
            rr.log(
                entity,
                rr.Arrows3D(
                    origins=[[0.0, 0.0, 0.0]] * 3,
                    vectors=[
                        [ODOM_AXIS_LEN, 0.0, 0.0],
                        [0.0, ODOM_AXIS_LEN, 0.0],
                        [0.0, 0.0, ODOM_AXIS_LEN],
                    ],
                    colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                    radii=ODOM_AXIS_LEN / 25,
                ),
                static=True,
            )
        odom_trail: list[tuple[float, float, float]] = []

        try:
            frame = 0
            for ray_obs in ray_pipeline:
                if ray_obs.pose_tuple is None:
                    continue
                ref_timing = _process_frame(
                    ray_obs, planners, goal, robot_height, render_voxel, clearance_clamp
                )
                _log_odometry(ray_obs.pose_tuple, ray_obs.ts, odom_trail, base_from_body)
                frame += 1
                print(
                    f"frame={frame} configs={len(planners)} "
                    f"rebuild(ref)={ref_timing['total_ms'] - ref_timing['plan_ms']:.1f}ms "
                    f"plan(ref)={ref_timing['plan_ms']:.1f}ms",
                    end="\r",
                    flush=True,
                )
        except KeyboardInterrupt:
            print("\ninterrupted")

    if out is not None:
        print(f"wrote {out}")
        print(f"open with: rerun {out}")


if __name__ == "__main__":
    typer.run(main)
