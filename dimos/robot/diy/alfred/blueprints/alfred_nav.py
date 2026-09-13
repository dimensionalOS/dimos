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

"""Alfred: lidar click-and-go navigation plus the pillar and both arms planned from viser.

    dimos --rerun-host 0.0.0.0 run alfred-nav

Navigation is the Go2 pattern on Point-LIO odometry (voxel map, MLS planner, dannav
holonomic follower, MovementManager) with AlfredHighLevel as the only FlowBase writer and
the alfred_v1 sensor mounts published rooted at the lidar. The pillar and the OpenArms sit on a
ControlCoordinator and are planned through viser on the alfred_v1 model; the arms are real
when OPENARM_LEFT_CAN and OPENARM_RIGHT_CAN are set, mock otherwise. Teleop comes from the
viewer. Transport is pinned to LCM because the Point-LIO C++ native does not speak zenoh.
"""

from __future__ import annotations

from functools import partial
import os
from typing import Any

from dimos.control.components import HardwareComponent
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.nav_3d.mls_planner.start_relay import StartRelay
from dimos.navigation.nav_3d.mls_planner.viz import planner_visual_override
from dimos.robot.diy.alfred.alfred_model import (
    alfred_arm_joints,
    alfred_model_config,
    alfred_rerun_urdf,
)
from dimos.robot.diy.alfred.blueprints.pillar import (
    PILLAR_LIFT_VELOCITY_LIMIT_M_S,
    PILLAR_MOTOR_TRANSPORTS,
)
from dimos.robot.diy.alfred.config import ALFRED
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_LIFT_JOINT,
    PillarConnection,
    pillar_hardware,
)
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.openarm.config import openarm_hardware
from dimos.visualization.rerun.urdf_robot import (
    UrdfRobotJointStateRerunFactory,
    UrdfRobotStaticRerunFactory,
)
from dimos.visualization.vis_module import vis_module

OPENARM_LEFT_CAN_ENV = "OPENARM_LEFT_CAN"
OPENARM_RIGHT_CAN_ENV = "OPENARM_RIGHT_CAN"
ARM_VELOCITY_LIMIT_RAD_S = 0.5  # the speed the arms were hardware-tested at

ODOM_FRAME = "odom"
LIDAR_FRAME = "mid360_link"
VOXEL_SIZE_M = 0.08
MAP_MAX_RANGE_M = 15.0  # far returns are the costliest to raytrace and the least reliable
STEP_THRESHOLD_M = 0.06  # wheeled base: a kerb is an obstacle (Go2 uses 0.16)
WALL_CLEARANCE_M = 0.2
PLANNER_VIZ_HZ = 0.0  # raise to draw the planner's search (nodes, edges, surface)
ALFRED_RERUN_ROOT = "world/alfred"


def _openarm_hardware_from_env() -> HardwareComponent:
    """Real Damiao arms only when both CAN ports are set; the mock adapter otherwise."""
    return openarm_hardware(
        left_can_port=os.environ.get(OPENARM_LEFT_CAN_ENV) or None,
        right_can_port=os.environ.get(OPENARM_RIGHT_CAN_ENV) or None,
    )


def alfred_manipulation_tasks() -> list[TaskConfig]:
    """One trajectory task for arms and lift; a limit is required per joint once any is set."""
    return [
        joint_trajectory_task(
            [*alfred_arm_joints(), PILLAR_LIFT_JOINT],
            velocity_limits={
                **dict.fromkeys(alfred_arm_joints(), ARM_VELOCITY_LIMIT_RAD_S),
                PILLAR_LIFT_JOINT: PILLAR_LIFT_VELOCITY_LIMIT_M_S,
            },
        ),
    ]


def _path_colored(msg: Any, color: tuple[int, int, int]) -> Any:
    return msg.to_rerun(color=color)


def _empty_path_dropped(msg: Any) -> Any:
    """The planner emits an empty path when there is no route; keep the last one drawn."""
    return None if len(msg.poses) == 0 else msg.to_rerun(color=(170, 60, 220))


def _alfred_urdf_static(rr: Any) -> list[tuple[str, Any]]:
    factory = UrdfRobotStaticRerunFactory(
        urdf_path=alfred_rerun_urdf(), root_path=ALFRED_RERUN_ROOT
    )
    return [
        *factory(rr),
        (ALFRED_RERUN_ROOT, rr.Transform3D(parent_frame="tf#/base_link")),
    ]


class _AlfredJointStateVisual:
    """Lift and arm links follow coordinator_joint_state; loaded on the first message."""

    def __init__(self) -> None:
        self._factory: UrdfRobotJointStateRerunFactory | None = None

    def __call__(self, msg: Any) -> list[tuple[str, Any]]:
        if self._factory is None:
            self._factory = UrdfRobotJointStateRerunFactory(
                urdf_path=alfred_rerun_urdf(),
                root_path=ALFRED_RERUN_ROOT,
                joint_name_mapper=lambda name: name,
            )
        return self._factory(msg)


def _rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            name="Alfred nav",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
        collapse_panels=True,
    )


_rerun_config = {
    "blueprint": _rerun_blueprint,
    "tf_axes": 0.35,
    "static": {ALFRED_RERUN_ROOT: _alfred_urdf_static},
    # The viewer rides wifi and Tailscale: small ring buffer, maps only, low rates.
    "memory_limit": "64MB",
    "max_hz": {
        "world/tf": 2.0,
        "world/global_map": 0.5,
        "world/local_map": 1.0,
        "world/wheel_odometry": 1.0,
        "world/coordinator_joint_state": 2.0,
    },
    "visual_override": {
        "world/lidar": None,
        "world/planner_path": _empty_path_dropped,
        "world/path": partial(_path_colored, color=(60, 220, 120)),
        "world/coordinator_joint_state": _AlfredJointStateVisual(),
        **planner_visual_override(
            PLANNER_VIZ_HZ, voxel_size=VOXEL_SIZE_M, wall_clearance_m=WALL_CLEARANCE_M
        ),
    },
}


alfred_nav = (
    autoconnect(
        vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config),
        AlfredHighLevel.blueprint(),
        PointLio.blueprint(
            lidar_ip=ALFRED.mid360_ip,
            frame_id=ODOM_FRAME,
            sensor_frame_id=LIDAR_FRAME,
        ),
        RayTracingVoxelMap.blueprint(
            voxel_size=VOXEL_SIZE_M,
            max_range=MAP_MAX_RANGE_M,
            emit_every=1,
            global_emit_every=50,
            min_health=-1,
            max_health=5,
            support_min=4,
            world_frame=ODOM_FRAME,
        ),
        MLSPlannerNative.blueprint(
            world_frame=ODOM_FRAME,
            base_frame="base_link",
            voxel_size=VOXEL_SIZE_M,
            robot_height=ALFRED.body_height,
            start_z_offset_m=0.0,
            wall_clearance_m=WALL_CLEARANCE_M,
            wall_buffer_m=0.75,
            wall_buffer_weight=100.0,
            step_threshold_m=STEP_THRESHOLD_M,
            step_penalty_weight=4.0,
            viz_publish_hz=PLANNER_VIZ_HZ,
        ).remappings(
            [
                (MLSPlannerNative, "global_map", "global_map_unused"),
                (MLSPlannerNative, "path", "planner_path"),
            ]
        ),
        StartRelay.blueprint(world_frame=ODOM_FRAME, base_frame="base_link"),
        DanLocalPlanner.blueprint(resample_spacing_m=0.1).remappings(
            [(DanLocalPlanner, "odom", "start_pose")]
        ),
        DanHolonomicTC.blueprint().remappings([(DanHolonomicTC, "odom", "start_pose")]),
        MovementManager.blueprint(),
        PillarConnection.blueprint(),
        planner(
            model=alfred_model_config(),
            visualization={"backend": "viser"},
        ),
        ControlCoordinator.blueprint(
            instance_name="ControlCoordinator",
            hardware=[pillar_hardware(), _openarm_hardware_from_env()],
            tasks=alfred_manipulation_tasks(),
        ),
    )
    .transports(dict(PILLAR_MOTOR_TRANSPORTS))
    # Point-LIO is a C++ native and speaks LCM only; the Rust natives accept either.
    .global_config(n_workers=12, robot_model="alfred", transport="lcm")
)
