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

"""Seeed B601-DM topology for the generic Damiao whole-body adapter.

SAFETY: the arm has no brakes. Disabling the motors (deactivate, disconnect, a
feedback fault) lets a raised arm fall, and the arm motors carry no torque
while activation calibrates the gripper. Rest the arm in its folded pose first.
"""

from __future__ import annotations

import can_motor_control
from can_motor_control import damiao

from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.damiao.adapter import DamiaoWholeBodyAdapter
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.seeed_b601_dm.config import B601_DM_MODEL
from dimos.robot.manipulators.seeed_b601_dm.joints import (
    B601_DM_ARM_JOINTS,
    B601_DM_GRIPPER_JOINT,
)

# Measured on the arm: the gripper motor travels 6.39 rad between its stops,
# and opening moves it toward more negative positions. Calibration drives
# past the full stroke so it reaches both stops from any start.
GRIPPER_CALIBRATION_TRAVEL_RAD = 7.0
GRIPPER_CURRENT = 0.15


class SeeedB601DMAdapter(DamiaoWholeBodyAdapter):
    """Six arm motors and a calibrated gripper on one Damiao CAN bus.

    DM4340 and DM4340P share the DM4340 driver profile. The bus is SocketCAN
    (``can0``) by default; point ``bus_devices`` at the serial port of the
    Damiao USB-CAN bridge shipped with the arm, for example
    ``{"b601_dm": "/dev/ttyACM0"}``.
    """

    bus_name = "b601_dm"
    arm_joints = {"arm": B601_DM_ARM_JOINTS}
    gripper_joints = {"gripper": B601_DM_GRIPPER_JOINT}
    bus_names = (bus_name,)
    kinematic_joint_names = B601_DM_ARM_JOINTS

    def get_limits(self) -> JointLimits:
        """Declare the calibrated gripper opening in its local 0-1 coordinate."""
        unknown = [None] * len(self.kinematic_joint_names)
        return JointLimits(
            position_lower=[*unknown, 0.0],
            position_upper=[*unknown, 1.0],
            velocity_max=[None] * len(self.joint_names),
        )

    @property
    def kinematic_model(self) -> RobotModel:
        """Return the lazy arm model."""
        return B601_DM_MODEL

    def _build_robot(self) -> can_motor_control.Robot:
        # Seeed's motor table. DM4340P uses the DM4340 driver profile.
        arm_motors = [
            can_motor_control.MotorSpec("joint1", damiao.MotorType.DM4340, 0x01, 0x11),
            can_motor_control.MotorSpec("joint2", damiao.MotorType.DM4340, 0x02, 0x12),
            can_motor_control.MotorSpec("joint3", damiao.MotorType.DM4340, 0x03, 0x13),
            can_motor_control.MotorSpec("joint4", damiao.MotorType.DM4310, 0x04, 0x14),
            can_motor_control.MotorSpec("joint5", damiao.MotorType.DM4310, 0x05, 0x15),
            can_motor_control.MotorSpec("joint6", damiao.MotorType.DM4310, 0x06, 0x16),
        ]
        gripper_motor = can_motor_control.MotorSpec(
            "gripper_motor",
            damiao.MotorType.DM4310,
            0x07,
            0x17,
        )
        return (
            can_motor_control.Robot.builder()
            .add_bus(
                self.bus_name,
                self._make_can_bus(self.bus_name),
                damiao.DamiaoCodec(),
            )
            .add_arm("arm", bus=self.bus_name, motors=arm_motors)
            .add_gripper(
                "gripper",
                bus=self.bus_name,
                motor=gripper_motor,
                opening_direction="decreasing_position",
                default_current=GRIPPER_CURRENT,
                calibration_travel=GRIPPER_CALIBRATION_TRAVEL_RAD,
            )
            .build()
        )
