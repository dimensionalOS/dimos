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

"""Seeed B601-DM model, hardware and planning configuration."""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import TaskConfig
from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators.seeed_b601_dm.joints import (
    B601_DM_ARM_JOINTS,
    B601_DM_DOF,
    B601_DM_GRIPPER_JOINT,
    B601_DM_JOINTS,
)

B601_DM_HARDWARE_ID = "b601_dm"
B601_DM_SOURCE = RobotDescriptionSource(
    url="https://github.com/Seeed-Projects/reBot-DevArm",
    ref="8494bdfa91f4e3712bb7e7743e7bbcc63b01b9af",
)
# At its folded mechanical rest this arm's joint2 encoder reads up to about
# +0.008 rad, just past the vendor URDF's 0.0 upper limit, which would put
# the planner's start state out of bounds.
B601_DM_JOINT2_LIMITS = (-3.14, 0.02)
B601_DM_ACCELERATION_LIMIT = 0.5
# Lazy: the description repository is fetched on first load, not on import.
B601_DM_MODEL = (
    RobotModel.from_file(B601_DM_SOURCE / "Rebot_Arm_description/DM/urdf/ReBot_Arm_DM.urdf")
    .with_fixed_joints("finger_left", "finger_right")
    .with_joint_position_limits(
        "joint2", lower=B601_DM_JOINT2_LIMITS[0], upper=B601_DM_JOINT2_LIMITS[1]
    )
    .with_default_joint_acceleration_limit(B601_DM_ACCELERATION_LIMIT)
)
# The vendor URDF lists 50-200 rad/s joint velocities; the motors top out at
# 10-30 rad/s. Scale planned trajectories to 0.2 rad/s (joints 1-3) and
# 0.8 rad/s (joints 4-6).
B601_DM_VELOCITY_SCALE = 0.004


def b601_dm_hardware(
    *,
    can_port: str | None = None,
    gravity_comp: bool = True,
) -> HardwareComponent:
    """Select in-memory hardware, or the physical arm when a port is given.

    ``can_port`` is a SocketCAN interface (``can0``) or the serial port of
    the Damiao USB-CAN bridge shipped with the arm (``/dev/ttyACM0``).
    """
    adapter_kwargs: dict[str, object] = {}
    limits: JointLimits | None = None
    if can_port is None:
        limits = JointLimits(
            position_lower=[*([None] * B601_DM_DOF), 0.0],
            position_upper=[*([None] * B601_DM_DOF), 1.0],
            velocity_max=[None] * len(B601_DM_JOINTS),
        )
    else:
        if not can_port.strip():
            raise ValueError("B601-DM CAN port must not be empty")
        adapter_kwargs["runtime_config"] = DamiaoRuntimeConfig(
            bus_devices={"b601_dm": can_port},
            gravity_comp=gravity_comp,
        )
    return HardwareComponent(
        hardware_id=B601_DM_HARDWARE_ID,
        hardware_type=HardwareType.WHOLE_BODY,
        joints=list(B601_DM_JOINTS),
        adapter_type="mock_whole_body" if can_port is None else "seeed_b601_dm",
        auto_enable=True,
        limits=limits,
        adapter_kwargs=adapter_kwargs,
        # Seeed's SDK MIT gains; the shoulder kd is capped at the shared
        # codec's 5.0 maximum. The gripper runs its own current-limited control.
        wb_config=WholeBodyConfig(
            kp=(120.0, 120.0, 120.0, 18.0, 18.0, 18.0, 0.0),
            kd=(5.0, 5.0, 5.0, 2.0, 2.0, 2.0, 0.0),
        ),
    )


def b601_dm_gripper_task() -> TaskConfig:
    """Normalized open/close control of the gripper joint (0.0 closed, 1.0 open)."""
    return TaskConfig(
        name=f"{B601_DM_HARDWARE_ID}_gripper",
        type="gripper",
        joint_names=[B601_DM_GRIPPER_JOINT],
        priority=20,
    )


def b601_dm_model_config(
    *,
    home_joints: list[float] | None = None,
) -> RobotModelConfig:
    """Build the planning config for the arm joints of the vendor model."""
    return RobotModelConfig(
        model=B601_DM_MODEL,
        joint_names=list(B601_DM_ARM_JOINTS),
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=B601_DM_ARM_JOINTS,
                base_link="base_link",
                tip_link="end_link",
            )
        ],
        auto_convert_meshes=True,
        gripper_hardware_id=B601_DM_HARDWARE_ID,
        home_joints=list(home_joints or [0.0] * B601_DM_DOF),
    )
