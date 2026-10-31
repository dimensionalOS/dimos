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

"""Alfred, the whole robot: lidar navigation on the coordinator's holonomic follower, the pillar and
both arms planned on that same coordinator, and Desktop's Controller (or the rerun web viewer) as the UI.

    dimos run alfred-full                                                # arms mocked, pillar real
    OPENARM_LEFT_CAN=can2 OPENARM_RIGHT_CAN=can3 dimos run alfred-full   # real arms; `dimos hardware can setup` first

Navigation is the Go2 pattern on Point-LIO odometry: voxel map, MLS global planner, native local
planner, then ``path`` into the coordinator's holonomic pose follower, which runs on Alfred's measured
plant model (alfred_posedomain.json). The follower and teleop (``tele_cmd_vel``) are ordinary joint
claims on the FlowBase's three virtual joints, arbitrated by priority on one coordinator. The
arbitrated twist leaves on ``/flowbase/cmd_vel`` and AlfredHighLevel stays the only FlowBase writer.
The pillar, the OpenArms and the base are planned by the ManipulationModule on the whole-body model in
the robot's own frame (``base_link``): arm targets ride with the robot and never depend on odometry,
and a plan that drives the base is a motion from where the robot stands, which the coordinator's base
task lays down from its odometry pose only while it runs. The Controller's arm view drives that
planner, so there is no viser.

Everything rides the default transport (zenoh), which is what Desktop reads.
"""

from __future__ import annotations

import math
import os
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.operator_hold_coordinator import OperatorHoldCoordinator
from dimos.control.path_following_coordinator import PathFollowingCoordinator
from dimos.control.tasks.joint_hold_task.joint_hold_task import joint_hold_task
from dimos.control.tasks.operator_hold_task.operator_hold_task import operator_hold_task
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JOINT_TRAJECTORY_TASK_NAME,
    joint_trajectory_task,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.drive_trains.flowbase.config import FLOWBASE_MAX_LINEAR_CMD_M_S
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.pointlio_blueprints import mid360_for_pointlio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.relocalization.lidar.module import LocalMapRelocalization
from dimos.mapping.relocalization.lidar.relocalize import MID360_NAV
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.embodiment.alfred import ALFRED
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.global_planner.mls_planner.start_relay import StartRelay
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.robot.diy.alfred.alfred_model import (
    ALFRED_ARM_READY,
    ALFRED_MOVING_LINKS,
    ALFRED_PLANAR_BASE,
    ALFRED_V1_MODEL,
    alfred_arm_joints,
    alfred_base_command_gains,
    alfred_follower_artifact,
    alfred_planar_model_config,
    alfred_rerun_urdf,
)
from dimos.robot.diy.alfred.blueprints._nav import (
    ALFRED_RERUN_ROOT,
    PLANNER_VIZ_HZ,
    STEP_THRESHOLD_M,
    VOXEL_SIZE_METERS,
    WALL_CLEARANCE_M,
    _alfred_rerun_config,
)
from dimos.robot.diy.alfred.blueprints.alfred_hardware import _alfred_lidar_hardware
from dimos.robot.diy.alfred.blueprints.alfred_nav import MID360_MAX_RANGE_METERS
from dimos.robot.diy.alfred.blueprints.pillar import (
    PILLAR_LIFT_VELOCITY_LIMIT_M_S,
    pillar_motor_transports,
)
from dimos.robot.diy.alfred.channels import channel_spec
from dimos.robot.diy.alfred.config import ALFRED_HARDWARE, LIO_BLIND_M
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_LIFT_JOINT,
    PillarConnection,
    pillar_hardware,
)
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.openarm.config import OPENARM_GRIPPER_JOINTS, openarm_hardware
from dimos.robot.urdf_manager import UrdfManager
from dimos.visualization.rerun.urdf_robot import (
    UrdfRobotJointStateRerunFactory,
    UrdfRobotStaticRerunFactory,
)
from dimos.visualization.vis_module import vis_module

OPENARM_LEFT_CAN_ENV = "OPENARM_LEFT_CAN"
OPENARM_RIGHT_CAN_ENV = "OPENARM_RIGHT_CAN"
ARM_VELOCITY_LIMIT_RAD_S = 0.5  # the speed the arms were hardware-tested at

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
# Edge of the arm map's voxels (alfred_full_perception's D455 map) and of the octree the
# planner builds from them: the two must be equal. At 2 cm the bench's layer reaches 1.5 cm
# above the real surface (3 cm at 5 cm voxels), a 5 cm can spans 3 cells instead of 1 or 2,
# and a collision check costs 2.5 ms against 2.1 ms with no map (2026-10-25, see the README).
MANIP_VOXEL_M = 0.02

ALFRED_BASE_HARDWARE_ID = "flowbase"
BASE_TRAJECTORY_TASK_NAME = "base_trajectory"
BASE_VELOCITY_TASK_NAME = "vel_flowbase"
NAV_FOLLOWER_TASK_NAME = "holonomic_follower"
# Three claims on the same base joints: a person outranks a plan, a plan outranks navigation.
TELEOP_PRIORITY = 20
PLAN_PRIORITY = 15
NAV_PRIORITY = 10
GRIPPER_PRIORITY = 20
# Each arm's hand is its own coordinator gripper task; the planner opens and closes a
# planning group's hand through this map (one gripper per robot is the default).
ALFRED_GRIPPER_TASKS = {
    "left_manipulator": "left_arm_gripper",
    "right_manipulator": "right_arm_gripper",
}


def alfred_openarm_hardware(
    *, left_can: str | None = None, right_can: str | None = None
) -> HardwareComponent:
    """Real Damiao arms on both CAN ports; with neither, the mock adapter.

    Mock arms start in the ready pose: hanging straight down is a singular configuration
    the planner's IK cannot move a hand out of.
    """
    hardware = openarm_hardware(left_can_port=left_can, right_can_port=right_can)
    if hardware.adapter_type == "mock_whole_body":
        hardware.adapter_kwargs = {
            "initial_positions": [ALFRED_ARM_READY.get(joint, 0.0) for joint in hardware.joints]
        }
    return hardware


def _openarm_hardware_from_env() -> HardwareComponent:
    """Real Damiao arms only when both CAN ports are set (the checkout's .env sets them on the
    robot); the mock adapter otherwise."""
    return alfred_openarm_hardware(
        left_can=os.environ.get(OPENARM_LEFT_CAN_ENV) or None,
        right_can=os.environ.get(OPENARM_RIGHT_CAN_ENV) or None,
    )


# The coordinator publishes the arbitrated twist through this adapter; AlfredHighLevel
# consumes it and stays the only Portal writer.
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


class AlfredOperatorCoordinator(OperatorHoldCoordinator, AlfredNavCoordinator):
    """AlfredNavCoordinator an operator can hold and e-stop (``OperatorHoldCoordinator``)."""


def alfred_coordinator_tasks(
    *, arms: bool = True, follower: bool = True, operator_hold: bool = False
) -> list[TaskConfig]:
    """The coordinator's claims for one stack: with ``arms``, the arms and lift on one trajectory
    task with a hold on the arms under it; always teleop and a plan's base segment on the base's
    three joints; with ``follower``, navigation's follower under them; with ``operator_hold``,
    the operator hold over every joint (priority 100, above all of them).

    The lift is left out of the hold: it is a leadscrew under its own brake.
    """
    base_joints = list(_flowbase_hardware.joints)
    tasks: list[TaskConfig] = []
    if arms:
        tasks += [
            joint_trajectory_task(
                [*alfred_arm_joints(), PILLAR_LIFT_JOINT],
                velocity_limits={
                    **dict.fromkeys(alfred_arm_joints(), ARM_VELOCITY_LIMIT_RAD_S),
                    PILLAR_LIFT_JOINT: PILLAR_LIFT_VELOCITY_LIMIT_M_S,
                },
            ),
            # The Damiao motors are torque-free until their first command: held from the first
            # tick, the arms stay where they are through a restart instead of sagging.
            joint_hold_task(alfred_arm_joints(), base_joints, name="hold_arms", always=True),
            # The hands: 0 closed, 1 open on the adapter's normalized joint (mock and Damiao).
            *(
                TaskConfig(
                    name=ALFRED_GRIPPER_TASKS[group],
                    type="gripper",
                    joint_names=[joint],
                    priority=GRIPPER_PRIORITY,
                )
                for group, joint in zip(
                    ("left_manipulator", "right_manipulator"), OPENARM_GRIPPER_JOINTS, strict=True
                )
            ),
        ]
    # 1/K of Alfred's measured plant (or unity with a warning): the base yaws 2.85x and
    # translates 0.77x what it is told, and fed the plan's rates as they are a turn runs
    # away from the task's feedback and aborts.
    command_gains = alfred_base_command_gains()
    tasks += [
        TaskConfig(
            name=BASE_VELOCITY_TASK_NAME,
            type="velocity",
            joint_names=base_joints,
            priority=TELEOP_PRIORITY,
            # The base coasts to a stop on its own ramp; zeroing on stream gaps fights it.
            params={"zero_on_timeout": False},
        ),
        TaskConfig(
            name=BASE_TRAJECTORY_TASK_NAME,
            type="planar_base_trajectory",
            joint_names=base_joints,
            priority=PLAN_PRIORITY,
            params={
                # The planner works in base_link: its base segment is a motion from where
                # the robot stands, laid down from the odometry pose when it starts.
                "relative": True,
                # The planner limits x and y separately, so a diagonal may exceed either;
                # and what the base is told is the limit times the gain, which must stay
                # under the firmware's cap or a fast diagonal is clipped off its heading.
                "max_linear": min(
                    math.hypot(_BASE_VX_LIMIT, _BASE_VY_LIMIT),
                    FLOWBASE_MAX_LINEAR_CMD_M_S / max(command_gains[0], command_gains[1]),
                ),
                "max_angular": _BASE_WZ_LIMIT,
                "command_gains": command_gains,
            },
        ),
    ]
    if follower:
        tasks.append(
            TaskConfig(
                name=NAV_FOLLOWER_TASK_NAME,
                type="holonomic_pose_follower",
                joint_names=base_joints,
                priority=NAV_PRIORITY,
                params={
                    "speed": 0.4,
                    "goal_tolerance": 0.20,
                    "orientation_tolerance": 0.25,
                    # Alfred's measured plant model, or the Go2's with a warning.
                    "artifact_path": alfred_follower_artifact(),
                },
            )
        )
    if operator_hold:
        # Held through an e-stop too: teleop's velocity task ignores the e-stop, and the lift
        # would finish its last move with nothing commanding it.
        tasks.append(operator_hold_task(hold_through_estop=True))
    return tasks


def _alfred_urdf_static(rr: Any) -> list[tuple[str, Any]]:
    factory = UrdfRobotStaticRerunFactory(
        urdf_path=alfred_rerun_urdf(), root_path=ALFRED_RERUN_ROOT
    )
    return [
        *factory(rr),
        (ALFRED_RERUN_ROOT, rr.Transform3D(parent_frame="tf#/base_link")),
    ]


class _AlfredJointStateVisual:
    """Lift and arm links follow coordinator_joint_state; the factory loads on the first message."""

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


def _rerun_config() -> dict[str, Any]:
    """The shared Alfred nav view, drawn on the planner's model so the lift and arms move."""
    shared = _alfred_rerun_config()
    return {
        **shared,
        "static": {ALFRED_RERUN_ROOT: _alfred_urdf_static},
        "max_hz": {**shared["max_hz"], "world/coordinator_joint_state": 2.0},
        "visual_override": {
            **shared["visual_override"],
            "world/coordinator_joint_state": _AlfredJointStateVisual(),
        },
    }


# The planner's world is the robot itself. Anything that calls the planning frame "world"
# (the Controller's arm targets, the stock manipulation tooling) resolves through this
# identity edge under base_link, so a target is a pose on the robot: it rides along when the
# base drives and survives odometry jumps, and the planner's model root is the same base_link
# tf draws the robot at (odom's origin is wherever Point-LIO woke up, lidar height included).
# The whole-body model's planar base joints are the base's motion *from here*: zero whenever
# a plan starts, and executed as a relative base trajectory.
_WORLD_IS_BASE = Transform(
    translation=Vector3(0.0, 0.0, 0.0),
    rotation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
    frame_id=BASE_FRAME,
    child_frame_id="world",
)


def alfred_nav_modules() -> tuple[Any, ...]:
    """Navigation on Point-LIO: the lidar, odometry, the voxel map, both planners, the base pose
    relay for the coordinator's base adapter, and relocalization."""
    return (
        mid360_for_pointlio(lidar_ip=ALFRED_HARDWARE.mid360_ip),
        PointLio.blueprint(blind=LIO_BLIND_M),
        RayTracingVoxelMap.blueprint(
            voxel_size=VOXEL_SIZE_METERS,
            max_range=MID360_MAX_RANGE_METERS,
            global_emit_every=50,
        ),
        MLSPlannerNative.blueprint(
            voxel_size=VOXEL_SIZE_METERS,
            robot_height=ALFRED.height,
            wall_clearance_m=WALL_CLEARANCE_M,
            step_threshold_m=STEP_THRESHOLD_M,
            step_penalty_weight=1.0,
            viz_publish_hz=PLANNER_VIZ_HZ,
        ).remappings(
            [
                (MLSPlannerNative, "global_map", "global_map_unused"),
                (MLSPlannerNative, "path", "planner_path"),
                # A click is the goal: the Controller's "Go here" and the rerun web viewer both
                # publish clicked_point, and there is no MovementManager here to relay it.
                (MLSPlannerNative, "goal", "clicked_point"),
            ]
        ),
        # Its `path` is what the follower task binds to on the coordinator.
        LocalPlannerNative.blueprint(embodiment=ALFRED, step_speed=0.2),
        # The twist adapter reads the base pose off /flowbase/odom; the relay puts tf's
        # odom -> base_link there.
        StartRelay.blueprint(world_frame=ODOM_FRAME, base_frame=BASE_FRAME),
        LocalMapRelocalization.blueprint(world_frame=ODOM_FRAME, relocalize=MID360_NAV),
    )


def alfred_urdf_manager() -> Any:
    """The robot's URDF for Desktop's Controller: the XML and its meshes on request. Joint tf
    stays off (Point-LIO or the wheel relay owns the base's pose, AlfredMountTf the mounts, the
    planner the moving links); the self filter is the perception layer's business."""
    return UrdfManager.blueprint(model=ALFRED_V1_MODEL, publish_joint_tf=False)


def alfred_arm_modules(*, pillar_device_path: str | None = None) -> tuple[Any, ...]:
    """The pillar, the URDF for the Controller, and the whole-body planner in base_link: the
    planar base joints are not fed from odometry, they are held at zero and a plan moves them
    from there. The obstacle map the perception layer adds arrives in base_link too.

    ``pillar_device_path`` overrides the Nano's port (``PILLAR_SIM_DEVICE``: emulated)."""
    pillar_config = {} if pillar_device_path is None else {"device_path": pillar_device_path}
    return (
        PillarConnection.blueprint(**pillar_config),
        alfred_urdf_manager(),
        planner(
            model=alfred_planar_model_config(tf_extra_links=ALFRED_MOVING_LINKS),
            visualization={"backend": "none"},
            world_frame=BASE_FRAME,
            voxel_map_resolution=MANIP_VOXEL_M,
            static_transforms=[_WORLD_IS_BASE],
            joint_state_defaults=dict.fromkeys(ALFRED_PLANAR_BASE.joint_names, 0.0),
            gripper_tasks=ALFRED_GRIPPER_TASKS,
            # Splits a whole-body plan; either side aborting cancels the other.
            trajectory_tasks={
                JOINT_TRAJECTORY_TASK_NAME: [*alfred_arm_joints(), PILLAR_LIFT_JOINT],
                BASE_TRAJECTORY_TASK_NAME: list(ALFRED_PLANAR_BASE.joint_names),
            },
        ),
    )


def alfred_coordinator(
    *,
    arms: bool = True,
    follower: bool = True,
    arm_hardware: HardwareComponent | None = None,
    base_hardware: HardwareComponent | None = None,
    operator_hold: bool = False,
) -> Any:
    """One coordinator for the stack: the pillar, the arms (with ``arms``) and the FlowBase's
    virtual joints. Teleop arrives as a Twist onto the base's joints; navigation (with
    ``follower``) as a Path on the follower's port. Both are ordinary joint claims from here.
    With ``operator_hold`` it is an ``AlfredOperatorCoordinator`` running the operator hold:
    an operator (or the agent) can freeze every joint, and the operator can e-stop.

    ``arm_hardware`` and ``base_hardware`` stand in for the arms the env names and for the
    FlowBase's transport adapter (a simulation's mocks); the base keeps its joints' names."""
    base = _flowbase_hardware if base_hardware is None else base_hardware
    if base.joints != _flowbase_hardware.joints:
        raise ValueError(f"the base's joints are the tasks': {_flowbase_hardware.joints}")
    hardware = [pillar_hardware(), base]
    if arms:
        hardware.insert(1, _openarm_hardware_from_env() if arm_hardware is None else arm_hardware)
    cls = AlfredOperatorCoordinator if operator_hold else AlfredNavCoordinator
    return cls.blueprint(
        instance_name="ControlCoordinator",
        hardware=hardware,
        tasks=alfred_coordinator_tasks(arms=arms, follower=follower, operator_hold=operator_hold),
    ).remappings([(cls, "twist_command", "tele_cmd_vel")])


def alfred_base_transports() -> dict[tuple[str, type], Any]:
    """The pillar's motor channels, the coordinator's one arbitrated base command (AlfredHighLevel
    turns it into wheels), and the base pose it reads back."""
    return {
        **pillar_motor_transports(),
        ("cmd_vel", Twist): channel_spec(f"/{ALFRED_BASE_HARDWARE_ID}/cmd_vel", Twist),
        ("start_pose", PoseStamped): channel_spec(f"/{ALFRED_BASE_HARDWARE_ID}/odom", PoseStamped),
    }


alfred_full = (
    autoconnect(
        _alfred_lidar_hardware,
        *alfred_nav_modules(),
        # The rerun web viewer's clicks land on clicked_point like the Controller's, and its
        # keyboard on tele_cmd_vel. Teleop override is handled by priority.
        vis_module(global_config.viewer, rerun_config=_rerun_config()),
        *alfred_arm_modules(),
        alfred_coordinator(),
    )
    .transports(alfred_base_transports())
    .global_config(n_workers=12, robot_model="alfred")
)
