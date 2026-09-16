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

Navigation is the Go2 pattern on Point-LIO odometry (voxel map, MLS planner, holonomic
pose follower), with AlfredHighLevel as the only FlowBase writer. Teleop, a plan's base
segment and navigation contend for the same base joints on one coordinator by priority.
The pillar and OpenArms are planned through viser on the alfred_v1 model; the arms are
real when OPENARM_LEFT_CAN and OPENARM_RIGHT_CAN are set, mock otherwise. Transport is
LCM: the Point-LIO native does not speak zenoh.
"""

from __future__ import annotations

from functools import partial
import math
import os
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.path_following_coordinator import PathFollowingCoordinator
from dimos.control.tasks.joint_hold_task.joint_hold_task import joint_hold_task
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.nav_3d.mls_planner.start_relay import StartRelay
from dimos.navigation.nav_3d.mls_planner.viz import planner_visual_override
from dimos.robot.diy.alfred.alfred_model import (
    ALFRED_HEIGHT_M,
    ALFRED_PLANAR_BASE,
    alfred_arm_joints,
    alfred_follower_artifact,
    alfred_planar_model_config,
    alfred_rerun_urdf,
)
from dimos.robot.diy.alfred.blueprints.pillar import (
    PILLAR_LIFT_VELOCITY_LIMIT_M_S,
    PILLAR_MOTOR_TRANSPORTS,
)
from dimos.robot.diy.alfred.config import ALFRED
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.robot.diy.alfred.mount_tf import AlfredMountTf
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
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.visualization.vis_module import vis_module

OPENARM_LEFT_CAN_ENV = "OPENARM_LEFT_CAN"
OPENARM_RIGHT_CAN_ENV = "OPENARM_RIGHT_CAN"
ARM_VELOCITY_LIMIT_RAD_S = 0.5  # the speed the arms were hardware-tested at

ODOM_FRAME = "odom"
LIDAR_FRAME = "mid360_link"
VOXEL_SIZE_M = 0.08
MAP_MAX_RANGE_M = 15.0  # far returns are the costliest to raytrace and the least reliable
STEP_THRESHOLD_M = 0.06  # wheeled base: a kerb is an obstacle (Go2 uses 0.16)
# Hard clearance from base_link. Commissioned value: below the inscribed radius
# (0.255) risks gaps Alfred cannot fit; nearer the circumscribed (0.373) costs routes.
WALL_CLEARANCE_M = 0.2
# Headroom a cell needs to count as floor. Rounds up to 2.0 m at VOXEL_SIZE_M, so
# a lower doorway reads as blocked - drop this first if routes go missing there.
PLANNER_CLEARANCE_HEIGHT_M = ALFRED_HEIGHT_M
PLANNER_VIZ_HZ = 0.0  # raise to draw the planner's search (nodes, edges, surface)
ALFRED_RERUN_ROOT = "world/alfred"


def _openarm_hardware_from_env() -> HardwareComponent:
    """Real Damiao arms only when both CAN ports are set; the mock adapter otherwise."""
    return openarm_hardware(
        left_can_port=os.environ.get(OPENARM_LEFT_CAN_ENV) or None,
        right_can_port=os.environ.get(OPENARM_RIGHT_CAN_ENV) or None,
    )


ALFRED_BASE_HARDWARE_ID = "flowbase"
BASE_TRAJECTORY_TASK_NAME = "base_trajectory"
BASE_VELOCITY_TASK_NAME = "vel_flowbase"
NAV_FOLLOWER_TASK_NAME = "holonomic_follower"
JOINT_TRAJECTORY_TASK_NAME = "joint_trajectory"
# The coordinator publishes the arbitrated twist; AlfredHighLevel consumes it and
# stays the only Portal writer.
_flowbase_hardware = HardwareComponent(
    hardware_id=ALFRED_BASE_HARDWARE_ID,
    hardware_type=HardwareType.BASE,
    joints=make_twist_base_joints(ALFRED_BASE_HARDWARE_ID),
    adapter_type="transport_lcm",
    auto_enable=True,
)
_BASE_VX_LIMIT, _BASE_VY_LIMIT, _BASE_WZ_LIMIT = ALFRED_PLANAR_BASE.velocity_limits


class AlfredNavCoordinator(PathFollowingCoordinator):
    """Carries the ``path`` and ``speed`` ports the follower binds to."""


def alfred_manipulation_tasks() -> list[TaskConfig]:
    """Arms and lift on one trajectory task, plus a lower-priority hold on the arms.

    The lift is left out of the hold: it is a leadscrew under its own brake.
    """
    return [
        joint_trajectory_task(
            [*alfred_arm_joints(), PILLAR_LIFT_JOINT],
            velocity_limits={
                **dict.fromkeys(alfred_arm_joints(), ARM_VELOCITY_LIMIT_RAD_S),
                PILLAR_LIFT_JOINT: PILLAR_LIFT_VELOCITY_LIMIT_M_S,
            },
        ),
        joint_hold_task(alfred_arm_joints(), list(_flowbase_hardware.joints), name="hold_arms"),
        # Teleop outranks both the plan and the follower.
        TaskConfig(
            name=BASE_VELOCITY_TASK_NAME,
            type="velocity",
            joint_names=list(_flowbase_hardware.joints),
            priority=20,
            # The base coasts to a stop on its own ramp; zeroing on stream gaps fights it.
            params={"zero_on_timeout": False},
        ),
        # Navigation: lowest of the three base claims.
        TaskConfig(
            name=NAV_FOLLOWER_TASK_NAME,
            type="holonomic_pose_follower",
            joint_names=list(_flowbase_hardware.joints),
            priority=10,
            params={
                "speed": 0.4,
                "goal_tolerance": 0.20,
                "orientation_tolerance": 0.25,
                # Alfred's measured plant model, or the Go2's with a warning.
                "artifact_path": alfred_follower_artifact(),
            },
        ),
        TaskConfig(
            name=BASE_TRAJECTORY_TASK_NAME,
            type="planar_base_trajectory",
            joint_names=list(_flowbase_hardware.joints),
            priority=15,
            # The planner limits x and y separately, so a diagonal may exceed either.
            params={
                "max_linear": math.hypot(_BASE_VX_LIMIT, _BASE_VY_LIMIT),
                "max_angular": _BASE_WZ_LIMIT,
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
    # The viewer rides wifi and Tailscale: small buffer, low rates.
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
        # A click in the viewer is the goal; teleop override is handled by priority.
        vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config).remappings(
            [(RerunWebSocketServer, "clicked_point", "goal")]
        ),
        AlfredHighLevel.blueprint(),
        AlfredMountTf.blueprint(root_frame=LIDAR_FRAME),
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
            # Gap a column needs above a cell for it to be floor (surfaces.rs
            # is_standable). Ceilings clear it; shelves and low doorways do not.
            robot_height=PLANNER_CLEARANCE_HEIGHT_M,
            start_z_offset_m=0.0,
            # Cells this close to a wall are impassable, measured from base_link.
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
        PillarConnection.blueprint(),
        planner(
            model=alfred_planar_model_config(),
            visualization={"backend": "viser"},
            # Same three numbers: twist joints on the coordinator, planar on the planner.
            joint_state_aliases=dict(
                zip(_flowbase_hardware.joints, ALFRED_PLANAR_BASE.joint_names, strict=True)
            ),
            # Splits a whole-body plan; either side aborting cancels the other.
            trajectory_tasks={
                JOINT_TRAJECTORY_TASK_NAME: [*alfred_arm_joints(), PILLAR_LIFT_JOINT],
                BASE_TRAJECTORY_TASK_NAME: list(ALFRED_PLANAR_BASE.joint_names),
            },
        ),
        # Teleop arrives as a Twist onto the base's virtual joints; navigation as a
        # Path on the follower's port. Both are ordinary joint claims from here.
        AlfredNavCoordinator.blueprint(
            instance_name="ControlCoordinator",
            hardware=[pillar_hardware(), _openarm_hardware_from_env(), _flowbase_hardware],
            tasks=alfred_manipulation_tasks(),
        ).remappings([(AlfredNavCoordinator, "twist_command", "tele_cmd_vel")]),
    )
    .transports(
        {
            **dict(PILLAR_MOTOR_TRANSPORTS),
            # cmd_vel is the coordinator's single arbitrated base command; everything
            # that drives the base leaves by it, and AlfredHighLevel turns it into wheels.
            ("cmd_vel", Twist): LCMTransport.spec(f"/{ALFRED_BASE_HARDWARE_ID}/cmd_vel", Twist),
            ("start_pose", PoseStamped): LCMTransport.spec(
                f"/{ALFRED_BASE_HARDWARE_ID}/odom", PoseStamped
            ),
        }
    )
    # Point-LIO is a C++ native and speaks LCM only; the Rust natives accept either.
    .global_config(n_workers=12, robot_model="alfred", transport="lcm")
)
