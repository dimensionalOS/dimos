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


"""Explicit robot geometry for Dreamlabs protocol conversion."""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator
from typing_extensions import Self

from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.dual_openyam.joints import (
    DUAL_OPENYAM_LEFT_ARM_JOINTS,
    DUAL_OPENYAM_RIGHT_ARM_JOINTS,
)
from dimos.robot.manipulators.dual_openyam.model import DUAL_OPENYAM_MODEL


class DreamlabsArmConfig(BaseModel):
    """Model target to SDK TCP transform, measured in metres and xyzw.

    The SDK pose is arm-base-local, x forward, y left, z up. Configure the
    actual tool geometry explicitly for another robot; no tool is estimated.
    """

    joint_names: list[str] = Field(min_length=1)
    base_frame: str = Field(min_length=1)
    target_frame: str = Field(min_length=1)
    gripper_joint: str = Field(min_length=1)
    target_to_sdk_xyz: tuple[float, float, float]
    target_to_sdk_xyzw: tuple[float, float, float, float]


ArmSide = Literal["left", "right"]


def yam_arms() -> dict[ArmSide, DreamlabsArmConfig]:
    sides: tuple[tuple[ArmSide, list[str]], ...] = (
        ("left", DUAL_OPENYAM_LEFT_ARM_JOINTS),
        ("right", DUAL_OPENYAM_RIGHT_ARM_JOINTS),
    )
    return {
        side: DreamlabsArmConfig(
            joint_names=names,
            base_frame=f"{side}_base",
            target_frame=f"{side}_grasp_frame",
            gripper_joint=f"{side}_arm/gripper",
            target_to_sdk_xyz=(0.0, 0.0, -0.0347),
            target_to_sdk_xyzw=(2**-0.5, -(2**-0.5), 0.0, 0.0),
        )
        for side, names in sides
    }


class DreamlabsBackendConfig(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    mode: Literal["joint", "eef"] = "joint"
    timeout_ms: int = Field(default=1500, gt=0, le=5000)
    deadman_timeout_s: float = Field(default=1.0, gt=0, allow_inf_nan=False)
    model: RobotModel = DUAL_OPENYAM_MODEL
    arms: dict[ArmSide, DreamlabsArmConfig] = Field(default_factory=yam_arms)

    @model_validator(mode="after")
    def validate_robot_mapping(self) -> Self:
        if not self.arms:
            raise ValueError("Dreamlabs requires at least one configured arm")
        names = [
            name for arm in self.arms.values() for name in (*arm.joint_names, arm.gripper_joint)
        ]
        if len(set(names)) != len(names):
            raise ValueError("Dreamlabs arm and gripper joints must be unique")
        targets = [arm.target_frame for arm in self.arms.values()]
        if len(set(targets)) != len(targets):
            raise ValueError("Dreamlabs target frames must be unique")
        if self.model != DUAL_OPENYAM_MODEL and self.arms == yam_arms():
            raise ValueError("Another robot requires explicit measured arm/tool mappings")
        return self
