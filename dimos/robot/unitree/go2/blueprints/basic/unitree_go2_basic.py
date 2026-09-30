#!/usr/bin/env python3

# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

from functools import partial
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.vis_module import vis_module

if TYPE_CHECKING:
    from types import ModuleType

    from rerun._baseclasses import Archetype
    from rerun.blueprint import Blueprint

    from dimos.msgs.geometry_msgs.PoseArray import PoseArray
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Twist import Twist
    from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.msgs.std_msgs.Float32 import Float32
    from dimos.visualization.rerun.bridge import RerunData, RerunMulti


def _convert_camera_info(camera_info: CameraInfo) -> RerunData:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def _convert_global_map(grid: PointCloud2) -> Archetype:
    return grid.to_rerun(bottom_cutoff=0)


def _convert_navigation_costmap(grid: OccupancyGrid) -> Archetype:
    return grid.to_rerun(
        colormap="Accent",
        z_offset=0.015,
        opacity=0.2,
        background="#484981",
    )


def _convert_pgo_keyframes(keyframes: PoseArray) -> RerunMulti:
    import rerun as rr

    positions = keyframes.positions()
    return [
        ("world/pgo/keyframes", rr.Points3D(positions, colors=[[255, 0, 0]], radii=[0.025])),
        ("world/pgo/path", rr.LineStrips3D([positions], colors=[[255, 255, 255]], radii=[0.01])),
    ]


def _convert_pgo_loops(edges: LineSegments3D) -> RerunMulti:
    import rerun as rr

    segments = edges.segments.astype(np.float32)
    strips = rr.LineStrips3D(segments, colors=[[231, 76, 60]], radii=[0.008])
    return [("world/pgo/loops", strips)]


def _plot_telemetry(name: str, msg: Float32) -> RerunMulti:
    import rerun as rr

    return [(f"plots/telemetry/{name}", rr.Scalars(msg.data))]


def _plot_odom(odom: PoseStamped) -> RerunMulti:
    import rerun as rr

    return [
        ("world/odom", odom.to_rerun()),
        ("plots/odom/x", rr.Scalars(odom.x)),
        ("plots/odom/y", rr.Scalars(odom.y)),
    ]


def _plot_cmd_vel(t: Twist) -> RerunMulti:
    import rerun as rr

    return [
        ("plots/cmd_vel/linear_x", rr.Scalars(t.linear.x)),
        ("plots/cmd_vel/angular_z", rr.Scalars(t.angular.z)),
    ]


def _static_robot_body(rr: ModuleType) -> list[Archetype]:
    return [
        rr.Boxes3D(
            half_sizes=[0.35, 0.155, 0.2],
            colors=[(0, 255, 127)],
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _go2_rerun_blueprint() -> Blueprint:
    """Split layout: camera feed + 3D world view side by side."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image", name="Camera"),
                rrb.TimeSeriesView(
                    origin="plots",
                    contents=["plots/odom/**", "plots/cmd_vel/**"],
                    name="odom + cmd_vel",
                ),
                rrb.TimeSeriesView(origin="plots/telemetry", name="telemetry"),
            ),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
                overrides={
                    "world/lidar": rrb.EntityBehavior(visible=False),
                },
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


rerun_config: dict[str, Any] = {
    "blueprint": _go2_rerun_blueprint,
    # Custom converters for specific rerun entity paths
    # Normally all these would be specified in their respectative modules
    # Until this is implemented we have central overrides here
    #
    # This is unsustainable once we move to multi robot etc
    "visual_override": {
        "world/camera_info": _convert_camera_info,
        "world/odom": _plot_odom,
        "world/cmd_vel": _plot_cmd_vel,
        "world/global_map": _convert_global_map,
        "world/merged_map": _convert_global_map,
        "world/navigation_costmap": _convert_navigation_costmap,
        "world/pgo_keyframes": _convert_pgo_keyframes,
        "world/pgo_loops": _convert_pgo_loops,
        # partial, not a closure: this config is pickled to the viewer's worker
        "world/mapper_frame_ms": partial(_plot_telemetry, "mapper_frame_ms"),
        "world/pgo_loop_ms": partial(_plot_telemetry, "pgo_loop_ms"),
        "world/pgo_rebuild_ms": partial(_plot_telemetry, "pgo_rebuild_ms"),
    },
    "max_hz": {
        "world/global_map": 0,  # publishes at ~7.8 Hz
        "world/color_image": 0,  # publishes at ~14 Hz
        "world/global_costmap": 0,  # publishes at ~7.6 Hz
        "world/lidar": 1,  # publishes at ~7.7 Hz; hidden by default in the blueprint
    },
    "tf_axes": 0.5,
    # slapping a go2 shaped box on the base_link frame
    "static": {
        "world/robot_body": _static_robot_body,
    },
}

_with_vis = autoconnect(
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=rerun_config,
    ),
)


unitree_go2_basic = (
    autoconnect(
        _with_vis,
        GO2Connection.blueprint(),
    ).global_config(n_workers=4, robot_model="unitree_go2")
    # we temporarily disabled sensor timestamps
    # and are derriving all timestmaps upon reception
    # this is because image webrtc stream doesn't have timestamps,
    # so it's difficult to corelate the streams otherwise
    #
    #    .configurators(ClockSyncConfigurator())
)
