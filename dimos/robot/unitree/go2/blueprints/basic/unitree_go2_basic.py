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

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs.Twist import RERUN_SERIES_NAMES
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.rerun.websocket_server import JOY_AXES
from dimos.visualization.vis_module import vis_module


def _convert_camera_info(camera_info: Any) -> Any:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def _convert_global_map(grid: Any) -> Any:
    return grid.to_rerun(bottom_cutoff=0)


def _convert_navigation_costmap(grid: Any) -> Any:
    return grid.to_rerun(
        colormap="Accent",
        z_offset=0.015,
        opacity=0.2,
        background="#484981",
    )


def _plot_odom(odom: Any) -> Any:
    import rerun as rr

    return [
        ("world/odom", odom.to_rerun()),
        ("plots/odom/x", rr.Scalars(odom.x)),
        ("plots/odom/y", rr.Scalars(odom.y)),
    ]


_AXIS_COLORS = {
    "linear_x": (80, 200, 120),
    "linear_y": (240, 170, 60),
    "angular_z": (90, 160, 255),
}
_UNUSED_AXIS_COLOR = (90, 90, 90)


def _velocity_series(rr: Any, names: tuple[str, ...]) -> Any:
    return rr.SeriesLines(
        names=list(names),
        colors=[_AXIS_COLORS.get(name, _UNUSED_AXIS_COLOR) for name in names],
        visible_series=[name in _AXIS_COLORS for name in names],
        interpolation_mode=rr.components.InterpolationMode.StepAfter,
    )


def _twist_series(rr: Any) -> Any:
    return _velocity_series(rr, RERUN_SERIES_NAMES)


def _joystick_series(rr: Any) -> Any:
    return _velocity_series(rr, JOY_AXES)


def _static_robot_body(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[0.35, 0.155, 0.2],
            colors=[(0, 255, 127)],
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _go2_rerun_blueprint() -> Any:
    """Split layout: camera feed + 3D world view side by side."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image", name="Camera"),
                rrb.TimeSeriesView(origin="plots/odom", name="odom"),
                rrb.TimeSeriesView(origin="world/joystick", name="joystick"),
                rrb.TimeSeriesView(origin="world/tele_cmd_vel", name="tele_cmd_vel"),
                rrb.TimeSeriesView(origin="world/cmd_vel", name="cmd_vel"),
                row_shares=[3, 1, 1, 1, 1],
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
                    "world/joystick": rrb.EntityBehavior(visible=False),
                    "world/tele_cmd_vel": rrb.EntityBehavior(visible=False),
                    "world/cmd_vel": rrb.EntityBehavior(visible=False),
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
        "world/global_map": _convert_global_map,
        "world/merged_map": _convert_global_map,
        "world/navigation_costmap": _convert_navigation_costmap,
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
        "world/joystick": _joystick_series,
        "world/tele_cmd_vel": _twist_series,
        "world/cmd_vel": _twist_series,
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
