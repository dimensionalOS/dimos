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

"""TARS (MuJoCo sim via tars_sdk) blueprints.

Usage:
    uv pip install -e experimental/tars_sdk
    dimos run tars-sim                                # TarsConnection + lidar global map in rerun
    dimos run tars-sim-keyboard-teleop                # same + WASD pygame teleop (Linux)
    dimos run coordinator-tars-sim                    # TARS in the Go2 office scene + viewer, /cmd_vel
    dimos run coordinator-tars-sim-keyboard-teleop    # + WASD pygame teleop (Linux; crashes on macOS)
    python -m dimos.robot.tars.demo_keyboard_cmd_vel  # macOS: run next to coordinator-tars-sim
"""

from __future__ import annotations

from typing import Any

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.robot.tars.connection import TarsConnection
from dimos.robot.tars.lidar_registration import LidarRegistration
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module

_tars_joints = make_twist_base_joints("tars")

_tars_hw = HardwareComponent(
    hardware_id="tars",
    hardware_type=HardwareType.BASE,
    joints=_tars_joints,
    adapter_type="tars",
    # same room as `dimos --simulation run unitree-go2`, in an open aisle facing +x
    adapter_kwargs={"scene": "office1", "spawn": (-4.5, 1.1, 0.0), "viewer": True},
)

_tars_vel_task = TaskConfig(
    name="vel_tars",
    type="velocity",
    joint_names=_tars_joints,
    priority=10,
)

coordinator_tars_sim = ControlCoordinator.blueprint(
    hardware=[_tars_hw],
    tasks=[_tars_vel_task],
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])

coordinator_tars_sim_keyboard_teleop = autoconnect(
    ControlCoordinator.blueprint(hardware=[_tars_hw], tasks=[_tars_vel_task]),
    KeyboardTeleop.blueprint(),
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])


def _convert_global_map(grid: Any) -> Any:
    return grid.to_rerun(bottom_cutoff=0)


def _static_tars_body(rr: Any) -> list[Any]:
    # the slab stack at half size, hanging from the hub (base_link)
    return [
        rr.Boxes3D(
            half_sizes=[0.07, 0.24, 0.38], centers=[[0.0, 0.0, -0.305]], colors=[(160, 160, 165)]
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _tars_rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.0)),
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


_tars_rerun_config: dict[str, Any] = {
    "blueprint": _tars_rerun_blueprint,
    # world/lidar is drawn in lidar_link (it rides the slab); the registered copy is a duplicate
    "visual_override": {"world/global_map": _convert_global_map, "world/registered_lidar": None},
    "max_hz": {"world/global_map": 0, "world/color_image": 0, "world/lidar": 2},
    "tf_axes": 0.3,
    "static": {"world/robot_body": _static_tars_body},
}

tars_sim = autoconnect(
    # WASD in the rerun viewer publishes tele_cmd_vel: drive TARS with it
    vis_module(viewer_backend=global_config.viewer, rerun_config=_tars_rerun_config).remappings(
        [(RerunWebSocketServer, "tele_cmd_vel", "cmd_vel")]
    ),
    # lidar arrives in lidar_link (on a swinging slab); TF at each scan instant registers it
    TarsConnection.blueprint(),
    LidarRegistration.blueprint(),
    VoxelGridMapper.blueprint(emit_every=5).remappings(
        [(VoxelGridMapper, "lidar", "registered_lidar")]
    ),
).global_config(n_workers=4)

tars_sim_keyboard_teleop = autoconnect(tars_sim, KeyboardTeleop.blueprint())
