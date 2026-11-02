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

"""Seeed Studio reBot B601-DM topology for the generic Damiao whole-body adapter.

SAFETY: the arm has no brakes. Disabling the motors (deactivate, disconnect, a
feedback fault) lets a raised arm fall. Rest it first.
"""

from __future__ import annotations

import can_motor_control
from can_motor_control import damiao

from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.damiao.adapter import DamiaoWholeBodyAdapter
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.seeedstudio.config import seeedstudio_model
from dimos.robot.manipulators.seeedstudio.joints import (
    SEEEDSTUDIO_ARM_JOINTS,
    SEEEDSTUDIO_GRIPPER_JOINT,
)

# Measured on the arm: the gripper motor travels about 5.8 rad between its
# stops, and opening moves it toward more negative positions. Calibration
# drives past the full stroke so it reaches both stops from any start.
GRIPPER_CALIBRATION_TRAVEL_RAD = 6.5
GRIPPER_CURRENT = 0.15


class SeeedStudioB601DMAdapter(DamiaoWholeBodyAdapter):
    """Six arm motors and a calibrated gripper on one Damiao CAN bus.

    The bus is SocketCAN (``can0``) by default. Point ``bus_devices`` at the
    serial port of the Damiao USB-CAN bridge shipped with the arm, for
    example ``{"b601_dm": "/dev/ttyACM0"}``.
    """

    bus_name = "b601_dm"
    arm_joints = {"arm": SEEEDSTUDIO_ARM_JOINTS}
    gripper_joints = {"gripper": SEEEDSTUDIO_GRIPPER_JOINT}
    bus_names = (bus_name,)
    kinematic_joint_names = SEEEDSTUDIO_ARM_JOINTS

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
        return seeedstudio_model()

    def _build_robot(self) -> can_motor_control.Robot:
        # Seeed's motor table: DM4340P at IDs 1-3 (same driver profile as the
        # DM4340), DM4310 at IDs 4-7. Each motor replies on its ID + 0x10.
        arm_motors = [
            can_motor_control.MotorSpec(
                name,
                damiao.MotorType.DM4340 if motor_id <= 3 else damiao.MotorType.DM4310,
                motor_id,
                motor_id + 0x10,
            )
            for motor_id, name in enumerate(SEEEDSTUDIO_ARM_JOINTS, start=1)
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
