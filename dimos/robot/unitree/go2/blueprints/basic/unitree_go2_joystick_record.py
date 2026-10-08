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

"""Drive-and-record blueprint for the Go2: the operator's input next to what the dog gets.

The viewer's WASD panel publishes ``tele_cmd_vel``, MovementManager turns
that into ``cmd_vel``, and the Go2 streams lidar, odom, tf and the camera. Run it with
``--record`` to store all of them. Built for long runs: there is no mapper, so nothing grows
with the distance covered, and the viewer holds a fixed memory limit. A map can be rebuilt
from the recording with ``dimos map global``.

Usage:
    dimos --record run unitree-go2-joystick-record --robot-ip <ip>
    dimos --replay-db recordings/<run-id>/memory.db run replay
"""

from functools import partial
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.vis_module import vis_module

_VELOCITY_AXES = ("linear_x", "linear_y", "angular_z")
_VELOCITY_COLORS = [(80, 200, 120), (240, 170, 60), (90, 160, 255)]
_VIEWER_MEMORY_LIMIT = "2GB"


def _plot_twist(name: str, twist: Any) -> Any:
    import rerun as rr

    return [(f"plots/{name}", rr.Scalars([twist.linear.x, twist.linear.y, twist.angular.z]))]


def _velocity_series(rr: Any) -> Any:
    return rr.SeriesLines(
        names=list(_VELOCITY_AXES),
        colors=_VELOCITY_COLORS,
        interpolation_mode=rr.components.InterpolationMode.StepAfter,
    )


def _record_rerun_blueprint() -> Any:
    """Camera and the operator plots on the left, the live lidar scan around the dog on the right."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image", name="Camera"),
                rrb.TimeSeriesView(origin="plots/odom", name="odom"),
                rrb.TimeSeriesView(origin="plots/tele_cmd_vel", name="tele_cmd_vel"),
                rrb.TimeSeriesView(origin="plots/cmd_vel", name="cmd_vel"),
                row_shares=[3, 1, 1, 1],
            ),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


record_rerun_config: dict[str, Any] = {
    **rerun_config,
    "blueprint": _record_rerun_blueprint,
    "visual_override": {
        **rerun_config["visual_override"],
        "world/tele_cmd_vel": partial(_plot_twist, "tele_cmd_vel"),
        "world/cmd_vel": partial(_plot_twist, "cmd_vel"),
    },
    "static": {
        **rerun_config["static"],
        "plots/tele_cmd_vel": _velocity_series,
        "plots/cmd_vel": _velocity_series,
    },
    "memory_limit": _VIEWER_MEMORY_LIMIT,
}

unitree_go2_joystick_record = autoconnect(
    GO2Connection.blueprint(),
    MovementManager.blueprint(),
    vis_module(viewer_backend=global_config.viewer, rerun_config=record_rerun_config),
).global_config(n_workers=5, robot_model="unitree_go2")
