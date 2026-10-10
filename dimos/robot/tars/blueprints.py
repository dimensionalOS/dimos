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
    dimos run tars-sim-nav                            # + costmap, A* to a clicked goal, exploration
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
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.navigation.experimental.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.experimental.patrolling.module import PatrollingModule
from dimos.navigation.go2.replanning_a_star.module import ReplanningAStarPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.tars.connection import TarsConnection
from dimos.robot.tars.height_crop import HeightCrop
from dimos.robot.tars.lidar_registration import LidarRegistration
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.visualization.vis_module import vis_module

# Footprint for the planner (half-size TARS): 0.48 m across the slabs; walking spreads the
# feet ~0.3 m fore-aft, so turning in place sweeps a ~0.6 m circle.
TARS_WIDTH = 0.5
TARS_ROTATION_DIAMETER = 0.65

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


def _convert_navigation_costmap(grid: Any) -> Any:
    return grid.to_rerun(colormap="Accent", z_offset=0.015, opacity=0.2, background="#484981")


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
    "visual_override": {
        "world/global_map": _convert_global_map,
        "world/navigation_costmap": _convert_navigation_costmap,
        "world/registered_lidar": None,
    },
    "max_hz": {
        "world/global_map": 0,
        "world/global_costmap": 0,
        "world/color_image": 0,
        "world/lidar": 2,
    },
    "tf_axes": 0.3,
    "static": {"world/robot_body": _static_tars_body},
}

# Teleop (rerun viewer WASD -> tele_cmd_vel) and navigation (nav_cmd_vel) both go through
# MovementManager, which owns cmd_vel and gives teleop priority.
tars_sim = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=_tars_rerun_config),
    # lidar arrives in lidar_link (on a swinging slab); TF at each scan instant registers it
    TarsConnection.blueprint(),
    LidarRegistration.blueprint(),
    VoxelGridMapper.blueprint(emit_every=5).remappings(
        [(VoxelGridMapper, "lidar", "registered_lidar")]
    ),
    MovementManager.blueprint(),
).global_config(n_workers=4)

tars_sim_keyboard_teleop = autoconnect(
    tars_sim,
    KeyboardTeleop.blueprint().remappings([(KeyboardTeleop, "cmd_vel", "tele_cmd_vel")]),
)

# Go2's 2D navigation stack (unitree-go2) with the TARS footprint. Its controller drives
# with (vx, wz) only, which suits TARS (no strafing). Click a point in rerun to send a goal.
tars_sim_nav = autoconnect(
    tars_sim,
    # the lidar can't see the floor right under TARS: treat the start area as free
    # the costmap sees the map below TARS's height only: the tilted lidar's ceiling hits
    # would otherwise read as walls (see HeightCrop)
    HeightCrop.blueprint(max_z=1.0),
    # TODO: can_pass_under should be ~0.9 m for TARS, but a HeightCostConfig in the
    # blueprint currently fails config validation; the default (0.6 m) is used for now.
    # The lidar can't see the floor right under TARS: treat the start area as free.
    CostMapper.blueprint(initial_safe_radius_meters=0.8).remappings(
        [(CostMapper, "global_map", "nav_map")]
    ),
    ReplanningAStarPlanner.blueprint(
        robot_width=TARS_WIDTH, robot_rotation_diameter=TARS_ROTATION_DIAMETER
    ),
    WavefrontFrontierExplorer.blueprint(),
    PatrollingModule.blueprint(),
).global_config(n_workers=8)
