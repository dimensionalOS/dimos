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

"""The smart go2 with the operator's input plotted next to the velocity it turns into.

``unitree-go2-joystick`` is ``unitree-go2`` with its own viewer window: joystick,
tele_cmd_vel and cmd_vel time series under the camera and the odom plot. Run it with
``--record`` to store all three streams; ``unitree-go2`` itself is unchanged.

``unitree-go2-joystick-replay`` plays a recording back with the same window, no robot
needed. ``run replay`` finds the window from the recording's run folder name, which a
dataset pulled from LFS does not have. The dataset is resolved the way
``dimos.memory.blueprints`` does it: a bare name only in the process running the replay
blueprint, and exported as ``REPLAY_DB`` because workers import this module with the
default global config.

Usage:
    dimos --record run unitree-go2-joystick --robot-ip <ip>
    dimos --replay-db go2_teleop_sf_office_2026-09-18 run unitree-go2-joystick-replay
"""

from functools import partial
import os
import sys
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.memory.replay_module import dataset_path, replay_module
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.visualization.rerun.websocket_server import JOY_AXES
from dimos.visualization.vis_module import vis_module

_VELOCITY_AXES = ("linear_x", "linear_y", "angular_z")
_VELOCITY_COLORS = [(80, 200, 120), (240, 170, 60), (90, 160, 255)]


def _plot_twist(name: str, twist: Any) -> Any:
    import rerun as rr

    return [(f"plots/{name}", rr.Scalars([twist.linear.x, twist.linear.y, twist.angular.z]))]


def _plot_joystick(joy: Any) -> Any:
    import rerun as rr

    axes = dict(zip(JOY_AXES, joy.axes, strict=True))
    return [("plots/joystick", rr.Scalars([axes[axis] for axis in _VELOCITY_AXES]))]


def _velocity_series(rr: Any) -> Any:
    return rr.SeriesLines(
        names=list(_VELOCITY_AXES),
        colors=_VELOCITY_COLORS,
        interpolation_mode=rr.components.InterpolationMode.StepAfter,
    )


def _joystick_rerun_blueprint() -> Any:
    """The Go2 layout with joystick, tele_cmd_vel and cmd_vel plots under the odom plot."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/color_image", name="Camera"),
                rrb.TimeSeriesView(origin="plots/odom", name="odom"),
                rrb.TimeSeriesView(origin="plots/joystick", name="joystick"),
                rrb.TimeSeriesView(origin="plots/tele_cmd_vel", name="tele_cmd_vel"),
                rrb.TimeSeriesView(origin="plots/cmd_vel", name="cmd_vel"),
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
                },
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


joystick_rerun_config: dict[str, Any] = {
    **rerun_config,
    "blueprint": _joystick_rerun_blueprint,
    "visual_override": {
        **rerun_config["visual_override"],
        "world/joystick": _plot_joystick,
        "world/tele_cmd_vel": partial(_plot_twist, "tele_cmd_vel"),
        "world/cmd_vel": partial(_plot_twist, "cmd_vel"),
    },
    "static": {
        **rerun_config["static"],
        "plots/joystick": _velocity_series,
        "plots/tele_cmd_vel": _velocity_series,
        "plots/cmd_vel": _velocity_series,
    },
}

unitree_go2_joystick = autoconnect(
    unitree_go2,
    vis_module(viewer_backend=global_config.viewer, rerun_config=joystick_rerun_config),
)

_DATASET = dataset_path(
    global_config.replay_db, explicit="unitree-go2-joystick-replay" in sys.argv[1:]
)
if _DATASET:
    os.environ["REPLAY_DB"] = _DATASET

Replay = replay_module(_DATASET)

unitree_go2_joystick_replay = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=joystick_rerun_config),
    Replay.blueprint(dataset=_DATASET),
)
