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

"""Two YAM leader arms drive the dual OpenYAM.

The leaders' Feetech connection publishes the arms' joint targets and jaw
openings; the coordinator's trajectory and gripper tasks follow them.
"""

from dataclasses import replace
from pathlib import Path
import threading
import time
from typing import Any, ClassVar

from reactivex.disposable import Disposable

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint, autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.hardware.controllers.joycon.joycon_module import JoyconConnectionModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Bool import Bool
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoCoordinator,
    arm_connection,
    arm_declaration,
    arm_remaps,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_ARM_LEADERS,
    DUAL_OPENYAM_HOME_PER_ARM,
    DUAL_OPENYAM_LEADERS,
    DUAL_OPENYAM_SIDES,
    dual_openyam_arm_joints,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.robot.manipulators.feetech.connection import (
    GUARDED_JOINT_FRAME_ID,
    DualFeetechConnectionConfig,
    FeetechConnection,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Following a human arm needs more than the URDF's 2 rad/s.
DUAL_OPENYAM_LEADER_VELOCITY_RAD_S = 3.0
DUAL_OPENYAM_LEADER_WRIST_VELOCITY_RAD_S = 4.0
DUAL_OPENYAM_WRIST_JOINTS = ("joint4", "joint5", "joint6")
# Measured on the OpenArm leaders: a 6 Hz low-pass halves wrist roughness for
# about 10 ms of lag, and a reading every coordinator tick halves each ramp.
DUAL_OPENYAM_LEADER_FILTER_CUTOFF_HZ = 6.0
DUAL_OPENYAM_LEADER_RATE_HZ = 100.0
DUAL_OPENYAM_GRIPPER_TASKS = {
    "left_manipulator": "left_arm_gripper",
    "right_manipulator": "right_arm_gripper",
}
# Every arm joint the leaders drive, as the coordinator names it.
DUAL_OPENYAM_LEADER_JOINTS = [
    f"arm/{motor.joint}" for bus in DUAL_OPENYAM_LEADERS.buses for motor in bus.motors
]


class DualOpenYamLeaderCoordinator(DamiaoCoordinator):
    """The dual OpenYAM coordinator, plus the port the leader arms' joint targets arrive on."""

    # The 100 Hz tick loop must not share a Python process with cameras, Viser
    # and recorders: on a Jetson that cost a quarter of its ticks and the arm
    # moved in steps.
    dedicated_worker: ClassVar[bool] = True

    leader_joint_command: In[JointState]


def dual_openyam_leader_velocity_limits() -> dict[str, float]:
    return {
        joint: (
            DUAL_OPENYAM_LEADER_WRIST_VELOCITY_RAD_S
            if joint.endswith(DUAL_OPENYAM_WRIST_JOINTS)
            else DUAL_OPENYAM_LEADER_VELOCITY_RAD_S
        )
        for joint in DUAL_OPENYAM_LEADER_JOINTS
    }


class DualOpenYamLeaderConnectionConfig(DualFeetechConnectionConfig):
    """Serial ports and calibration files of the two YAM leader arms.

    Attributes:
        left_serial_port: The left leader's serial port, e.g. "/dev/ttyACM1";
            empty leaves that leader unread.
        right_serial_port: The right leader's serial port; empty leaves it unread.
        left_calibration_path: The left leader's calibration file, if not the default.
        right_calibration_path: The right leader's calibration file, if not the default.
    """

    left_calibration_path: Path | None = None
    right_calibration_path: Path | None = None


class DualOpenYamLeaderConnection(FeetechConnection):
    """The two YAM leader arms; the serial ports given select which are read.

    While ``follow`` is off, nothing is sent and the follower holds still, so
    the leaders can be moved freely.
    """

    # Off pauses the leaders; on (the start state) sends their targets again.
    follow: In[Bool]

    config: DualOpenYamLeaderConnectionConfig
    robot = DUAL_OPENYAM_LEADERS

    def serial_ports(self) -> dict[str, str]:
        return self.config.serial_ports()

    def calibration_path(self, bus: str) -> Path:
        explicit = {
            "left": self.config.left_calibration_path,
            "right": self.config.right_calibration_path,
        }.get(bus)
        return explicit if explicit is not None else super().calibration_path(bus)

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._following = True
        self._follow_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.follow.subscribe(self._on_follow)))

    def publish_reading(self, readings: dict[str, dict[str, float]]) -> None:
        with self._follow_lock:
            if self._following:
                super().publish_reading(readings)

    def _on_follow(self, msg: Bool) -> None:
        with self._follow_lock:
            if msg.data == self._following:
                return
            self._following = msg.data
            if not msg.data:
                # A jog with no positions and zero speed tells the trajectory
                # task to hold every joint now, not 0.4 s after the last target.
                joints = [
                    f"{self.config.source}/{motor.joint}"
                    for bus in self.robot.buses
                    for motor in bus.motors
                ]
                self.joint_state.publish(
                    JointState(
                        ts=time.time(),
                        frame_id=GUARDED_JOINT_FRAME_ID,
                        name=joints,
                        velocity=[0.0] * len(joints),
                    )
                )
        logger.info("%s %s", type(self).__name__, "resumed" if msg.data else "paused")


class DualOpenYamLeaderArmConnection(DualOpenYamLeaderConnection):
    """The two YAM leader arms without their trigger servos; the jaws are driven elsewhere.

    Each side's calibration file must list only joint_1 to joint_6.
    """

    robot = DUAL_OPENYAM_ARM_LEADERS


# The follower's jaws by side; each is named <side>_gripper on the robot.
_JAWS = {jaw.name.removesuffix("_gripper"): jaw for jaw in DualOpenYamConnection.robot.grippers}


def dual_openyam_leaders(
    connection: type[DualOpenYamLeaderConnection] = DualOpenYamLeaderConnection,
) -> Blueprint:
    """The leader arms' connection; --simulation without ports parks mock leaders at home."""
    mock_positions = (
        {
            joint: home
            for side in DUAL_OPENYAM_SIDES
            for joint, home in zip(
                dual_openyam_arm_joints(side), DUAL_OPENYAM_HOME_PER_ARM, strict=True
            )
        }
        if global_config.simulation
        else None
    )
    return connection.blueprint(
        state_rate_hz=DUAL_OPENYAM_LEADER_RATE_HZ,
        filter_cutoff_hz=DUAL_OPENYAM_LEADER_FILTER_CUTOFF_HZ,
        mock_positions=mock_positions,
    ).remappings([(connection, "joint_state", "leader_joint_command")])


def dual_openyam_leader_control() -> MetaBlueprint:
    """The trajectory task fed by the leader port, plus a jaw per side, over the two-bus connection."""
    return autoconnect(
        DualOpenYamLeaderCoordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[arm_declaration(DualOpenYamConnection)],
            tasks=[
                # The trajectory task follows the leader's targets instead of joint_command.
                replace(
                    joint_trajectory_task(
                        DUAL_OPENYAM_LEADER_JOINTS,
                        velocity_limits=dual_openyam_leader_velocity_limits(),
                    ),
                    stream_bind={"joint_command": "leader_joint_command"},
                ),
                *[
                    TaskConfig(
                        name=f"{side}_arm_gripper",
                        type="gripper",
                        joint_names=[f"arm/{_JAWS[side].joint}"],
                        priority=20,
                        params={"open_at": _JAWS[side].open_at},
                        stream_bind={"gripper_command": f"{side}_gripper_command"},
                    )
                    for side in DUAL_OPENYAM_SIDES
                ],
            ],
            zero_stiffness_joints=[],
        ),
        arm_connection(DualOpenYamConnection),
    ).remappings(arm_remaps(DualOpenYamConnection, DualOpenYamLeaderCoordinator))


def _manipulation() -> Blueprint:
    return ManipulationModule.blueprint(
        gripper_tasks=DUAL_OPENYAM_GRIPPER_TASKS,
        model=dual_openyam_model_config(robot="arm"),
        visualization=ViserVisualizationConfig(),
    )


# The leader serial ports given select the sides; --simulation mocks the CAN, and
# the leaders too when no serial port is given.
dual_openyam_leader_teleop = autoconnect(
    dual_openyam_leaders(), dual_openyam_leader_control(), _manipulation()
)

# Leaders without trigger servos; Joy-Cons drive the jaws and pause following.
dual_openyam_joycon_leader_teleop = autoconnect(
    dual_openyam_leaders(DualOpenYamLeaderArmConnection),
    JoyconConnectionModule.blueprint(),
    dual_openyam_leader_control(),
    _manipulation(),
)
