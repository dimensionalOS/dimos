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

"""Alfred's basics: every connection to the robot, the mast D455 and the URDF, nothing else.

    dimos run alfred-base                                                # arms mocked, the rest real
    OPENARM_LEFT_CAN=can2 OPENARM_RIGHT_CAN=can3 dimos run alfred-base   # real arms; `dimos hardware can setup` first

The bottom layer: ``alfred-perception`` adds localize to it, ``alfred-manipulation`` adds the
planner on top of that. Here the FlowBase (AlfredHighLevel, its wheel odometry as ``odom ->
base_link``), the pillar and the arms answer on one coordinator, and the Controller drives them.
``UrdfManager`` owns the robot on tf: the mounts, the lift and the arms, posed from the
coordinator's joint state, so nothing above this layer has to. The D455's driver keeps its imagers
(``ALFRED_TF_MODEL``).

Every command goes through the coordinator. The pillar and the FlowBase are connection modules, on
its ``<robot>_<kind>`` ports: it arbitrates its tasks each tick and sends each robot its complete
command, and each halts itself on a deadman if those commands stop. The arms stay on an adapter:
their CAN bus runs in the coordinator's own process, on its own tick.

The coordinator's claims are all here too, so a layer adds modules and never touches them: the
trajectory, hold and gripper tasks sit idle until a planner sends them something, and an operator
can hold and e-stop any stack built on this.

Everything rides the default transport (zenoh), which is what Desktop reads. No Rerun bridge: the
Desktop reads zenoh itself, and the bridge's late-viewer buffer is what grew to 6.4 GB and got a
stack OOM-killed (2026-10-25).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from typing import Any

from dimos.control.components import HardwareComponent, make_twist_base_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.operator_hold_coordinator import OperatorHoldCoordinator
from dimos.control.path_following_coordinator import PathFollowingCoordinator
from dimos.control.tasks.joint_hold_task.joint_hold_task import joint_hold_task
from dimos.control.tasks.operator_hold_task.operator_hold_task import operator_hold_task
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In, Out
from dimos.hardware.drive_trains.flowbase.config import FLOWBASE_MAX_LINEAR_CMD_M_S
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.diy.alfred.alfred_model import (
    ALFRED_ARM_READY,
    ALFRED_PLANAR_BASE,
    ALFRED_TF_MODEL,
    ALFRED_V1_MODEL,
    alfred_arm_joints,
    alfred_base_command_gains,
    alfred_follower_artifact,
)
from dimos.robot.diy.alfred.blueprints.pillar import (
    PILLAR_CONNECTION_PORTS,
    PILLAR_LIFT_VELOCITY_LIMIT_M_S,
)
from dimos.robot.diy.alfred.config import ALFRED_HARDWARE
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.robot.diy.alfred.pillar_connection import PILLAR_LIFT_JOINT, PillarConnection
from dimos.robot.diy.alfred.wheel_odometry_relay import WheelOdometryRelay
from dimos.robot.manipulators.openarm.config import OPENARM_GRIPPER_JOINTS, openarm_hardware
from dimos.robot.urdf_manager import UrdfManager

# librealsense leaks usbfs fds (~8/s) in its worker; keep socket-accepting modules out of
# its fd table so EMFILE cannot take down the rest (see alfred_hardware.py).
RealSenseCamera.dedicated_worker = True

OPENARM_LEFT_CAN_ENV = "OPENARM_LEFT_CAN"
OPENARM_RIGHT_CAN_ENV = "OPENARM_RIGHT_CAN"
ARM_VELOCITY_LIMIT_RAD_S = 0.5  # the speed the arms were hardware-tested at

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

ALFRED_BASE_ROBOT = "flowbase"
BASE_TRAJECTORY_TASK_NAME = "base_trajectory"
BASE_VELOCITY_TASK_NAME = "vel_flowbase"
NAV_FOLLOWER_TASK_NAME = "holonomic_follower"
_BASE_JOINTS = make_twist_base_joints(ALFRED_BASE_ROBOT)
# A coordinator's commands come every tick, and on an e-stop it just stops sending them: this is
# how long the base runs on after one.
FLOWBASE_DEADMAN_S = 0.1
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

D455_WIDTH, D455_HEIGHT, D455_FPS = 848, 480, 15
D455_POINTCLOUD_FPS = 5.0
# Every third pixel each way keeps ~45k points a cloud: the self filter and the second voxel
# map each cost about a quarter of a core at that, half a core or more at 2.
D455_POINTCLOUD_DECIMATION = 3
DEPTH_TRUNC_M = 3.0

# How often the whole robot goes out on tf. Localize takes a pose only with a sample of every edge
# within 0.12 s of a frame (rig.TF_TOLERANCE), so an unchanged edge needs more than UrdfManager's
# default 4 Hz. Posing the 29 joints costs ~10 ms on the Orin: a third of a core at the default 30 Hz.
JOINT_TF_HZ = 10.0


@dataclass(frozen=True)
class AlfredHardware:
    """Stand-ins for what drives the robot (a simulation's); a None field is the robot's own."""

    # The FlowBase's connection module (AlfredHighLevel, over Portal to its Pi): cmd_vel in,
    # wheel_odometry out.
    flowbase: Any | None = None
    # PillarConnection's port (the Nano's CH340).
    pillar_device_path: str | None = None
    # The coordinator's arms (Damiao on the CAN ports the env names).
    arms: HardwareComponent | None = None
    # The mast D455.
    camera: Any | None = None


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


_BASE_VX_LIMIT, _BASE_VY_LIMIT, _BASE_WZ_LIMIT = ALFRED_PLANAR_BASE.velocity_limits


class AlfredNavCoordinator(PathFollowingCoordinator):
    """Drives the pillar and the FlowBase through their connection modules, and carries the
    ``path`` and ``speed`` ports the follower binds to."""

    pillar_position_command: Out[JointState]
    pillar_joint_state: In[JointState]
    flowbase_cmd_vel: Out[Twist]
    flowbase_odom: In[PoseStamped]


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
    base_joints = list(_BASE_JOINTS)
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


def alfred_coordinator(
    *,
    arms: bool = True,
    follower: bool = True,
    arm_hardware: HardwareComponent | None = None,
    operator_hold: bool = False,
) -> Any:
    """One coordinator for the stack: the arms on their adapter (with ``arms``), the pillar and
    the FlowBase through their connection modules' ports. Teleop arrives as a Twist onto the
    base's joints; navigation (with ``follower``) as a Path on the follower's port. Both are
    ordinary joint claims from here. With ``operator_hold`` it is an
    ``AlfredOperatorCoordinator`` running the operator hold: an operator (or the agent) can
    freeze every joint, and the operator can e-stop.

    ``arm_hardware`` stands in for the arms the env names (a simulation's mocks)."""
    hardware = []
    if arms:
        hardware.append(_openarm_hardware_from_env() if arm_hardware is None else arm_hardware)
    cls = AlfredOperatorCoordinator if operator_hold else AlfredNavCoordinator
    return cls.blueprint(
        instance_name="ControlCoordinator",
        hardware=hardware,
        tasks=alfred_coordinator_tasks(arms=arms, follower=follower, operator_hold=operator_hold),
        # The lift's joint and its limits.
        model=ALFRED_V1_MODEL,
    ).remappings([(cls, "twist_command", "tele_cmd_vel")])


def alfred_connections(flowbase: type, base_pose: type) -> list[tuple[type, str, str]]:
    """The stack's connection modules onto the coordinator's ports: ``flowbase`` takes the
    arbitrated twist, ``base_pose`` publishes where the base is."""
    return [
        *PILLAR_CONNECTION_PORTS,
        (flowbase, "cmd_vel", "flowbase_cmd_vel"),
        (base_pose, "start_pose", "flowbase_odom"),
    ]


def mast_d455() -> Any:
    """The D455 on the mast: colour for the camera panel and localize, depth aligned to it, and a
    decimated cloud for the arm planner's map."""
    return RealSenseCamera.blueprint(
        width=D455_WIDTH,
        height=D455_HEIGHT,
        fps=D455_FPS,
        enable_color=True,
        enable_depth=True,
        align_depth_to_color=True,
        enable_pointcloud=True,
        pointcloud_fps=D455_POINTCLOUD_FPS,
        pointcloud_decimation=D455_POINTCLOUD_DECIMATION,
        depth_trunc_m=DEPTH_TRUNC_M,
        enable_infrared=False,
        enable_imu=False,
        frame_id="camera_front_link",
        serial_number=ALFRED_HARDWARE.d455_serial,
        # The mast D455 hangs upside down (as alfred_v1.urdf mounts it); pictures come out upright.
        rotate_180=True,
    )


def alfred_base_modules(hardware: AlfredHardware | None = None) -> tuple[Any, ...]:
    """The connections, the camera and the URDF; ``hardware`` swaps what drives the robot for
    stand-ins and leaves the rest as it is."""
    hardware = hardware or AlfredHardware()
    pillar_config = (
        {} if hardware.pillar_device_path is None else {"device_path": hardware.pillar_device_path}
    )
    return (
        mast_d455() if hardware.camera is None else hardware.camera,
        (
            AlfredHighLevel.blueprint(deadman_timeout_s=FLOWBASE_DEADMAN_S)
            if hardware.flowbase is None
            else hardware.flowbase
        ),
        WheelOdometryRelay.blueprint(world_frame=ODOM_FRAME, base_frame=BASE_FRAME),
        PillarConnection.blueprint(**pillar_config),
        alfred_coordinator(
            follower=False,
            arm_hardware=hardware.arms,
            operator_hold=True,
        ),
        UrdfManager.blueprint(
            model=ALFRED_TF_MODEL, joint_tf_hz=JOINT_TF_HZ, unchanged_tf_hz=JOINT_TF_HZ
        ),
    )


alfred_base = (
    autoconnect(*alfred_base_modules())
    .remappings(alfred_connections(AlfredHighLevel, WheelOdometryRelay))
    .global_config(n_workers=4, robot_model="alfred")
)
