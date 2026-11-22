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

"""Coordinator types and explicit composition helpers for Damiao deployments.

Blueprints wire a coordinator and a connection themselves with ``autoconnect``,
exactly like the xArm ones. The only shared helpers are the two declarations
that must agree with the connection: the simulation swap and the stream remaps.
"""

from __future__ import annotations

from dimos_lcm.control_msgs import DynamicJointState

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import PD_INTERFACES
from dimos.control.coordinator import ControlCoordinator
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.damiao.connection import DamiaoConnection
from dimos.robot.manipulators.damiao.robot import DamiaoRobot


class DamiaoCoordinator(ControlCoordinator):
    """One connection named arm, including a robot with two synchronized arms."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]


class DamiaoMotorCommandCoordinator(ControlCoordinator):
    """Like ``DamiaoCoordinator``, but tasks may also set the arm joints'
    stiffness, damping, speed and extra torque."""

    arm_motor_command: Out[DynamicJointState]
    arm_joint_state: In[JointState]
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]


class DamiaoTwistCoordinator(ArmTwistCoordinator):
    """End-effector twists and gripper inputs, with joint commands over streams."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]


class DamiaoTeleopCoordinator(TeleopControlCoordinator):
    """WebXR inputs with a direct connection owning the robot."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


def mock_initial_positions(robot: DamiaoRobot) -> dict[str, float]:
    """The simulated start: the robot's home posture with its jaws closed.

    Loads the robot's model, to find where each jaw is closed.
    """
    home = robot.home_joints or [0.0] * len(robot.arm_joint_names)
    travel = robot.gripper_travel()
    closed = [travel[joint][0] for joint in robot.gripper_joint_names]
    return dict(zip(robot.joint_names, [*home, *closed], strict=True))


def arm_declaration(
    connection: type[DamiaoConnection], *, motor_command: bool = False
) -> ConnectionConfig:
    """The explicit command contract shared by the native and simulated deployment.

    Robot topology already names every driven joint, including the grippers;
    composing a blueprint does not need to load geometry to discover them.

    Args:
        connection: The robot's connection class.
        motor_command: Command the arm joints on ``motor_command``, so tasks
            may also set each one's speed, extra torque, stiffness and damping.
            Stiffness and damping may not exceed the robot's own kp and kd. The
            jaws still take a position only.
    """
    robot = connection.robot
    buses = robot.buses if motor_command else []
    kp = {motor.name: gain for bus in buses for motor, gain in zip(bus.arm, bus.kp, strict=True)}
    kd = {motor.name: gain for bus in buses for motor, gain in zip(bus.arm, bus.kd, strict=True)}
    return ConnectionConfig(
        source="arm",
        model=robot.model,
        joint_commands={
            name: PD_INTERFACES if name in kp else (Interface.POSITION,)
            for name in robot.joint_names
        },
        max_stiffness=kp,
        max_damping=kd,
    )


def arm_connection(
    connection: type[DamiaoConnection],
    *,
    teach: bool = False,
    config_writes: bool = False,
    motor_command: bool = False,
) -> Blueprint:
    """The deployed connection, or an independent mock when ``--simulation`` is on.

    Blueprints call this at the point they compose their modules, so the
    simulation swap is visible next to the coordinator it feeds instead of
    hiding inside a general-purpose composer.

    Args:
        connection: The robot's connection.
        teach: Drive every arm joint with zero stiffness.
        config_writes: Serve ``set_config``, for calibration blueprints only.
            Calibration drives the real arm, so this refuses ``--simulation``.
        motor_command: As for ``arm_declaration``.
    """
    declaration = arm_declaration(connection, motor_command=motor_command)
    if global_config.simulation:
        if config_writes:
            raise ValueError(
                "calibration drives the real arm: run the calibration blueprint without "
                "--simulation"
            )
        return MockJointConnection.blueprint(
            **declaration.model_dump(),
            initial_positions=mock_initial_positions(connection.robot),
        )
    return connection.blueprint(
        **declaration.model_dump(), teach=teach, config_writes=config_writes
    )


def arm_remaps(
    connection: type[DamiaoConnection],
    coordinator: type[ControlCoordinator],
    *,
    motor_command: bool = False,
) -> list[tuple[type[ModuleBase], str, str]]:
    """Wire the connection's command stream and the coordinator's joint state.

    While simulating, the command stream belongs to the mock connection that
    ``arm_connection`` substituted, so the remap names it instead.

    Args:
        connection: The robot's connection class.
        coordinator: The coordinator class that drives it.
        motor_command: Wire ``motor_command`` instead of ``position_command``;
            the coordinator needs an ``arm_motor_command`` port, e.g.
            ``DamiaoMotorCommandCoordinator``.
    """
    module: type[ModuleBase] = MockJointConnection if global_config.simulation else connection
    port = "motor_command" if motor_command else "position_command"
    return [
        (module, port, f"arm_{port}"),
        (coordinator, "arm_joint_state", "joint_state"),
    ]
