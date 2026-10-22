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

"""One Piper connection for CAN hardware and sim2."""

from dataclasses import replace
from pathlib import Path
from typing import cast

from pydantic import BaseModel, model_validator
from typing_extensions import Self

from dimos.control.contract.keys import Unit
from dimos.control.contract.ports import JointPort
from dimos.control.contract.presets import GripperSpec
from dimos.robot.manipulators.common.connection import ArmConnection, ArmConnectionConfig
from dimos.robot.manipulators.piper.config import make_piper_model_config
from dimos.robot.manipulators.piper.simulation import PIPER_HOME, piper_definition
from dimos.sim2.spec import RobotInstance
from dimos.sim2.world.scene import scene_robot


class PiperConnectionConfig(ArmConnectionConfig):
    dof: int = 6
    address: str = "can0"
    position_command: JointPort = JointPort()

    @model_validator(mode="after")
    def _arm(self) -> Self:
        if self.dof != 6:
            raise ValueError("Piper has six arm joints")
        self.model = make_piper_model_config(joint_prefix="").model
        self.position_command = JointPort(
            joints=[f"joint{i}" for i in range(1, 7)],
            gripper=GripperSpec(unit=Unit.M, lo=0, hi=0.08) if self.gripper else None,
        )
        return self


class PiperConnection(ArmConnection):
    config: PiperConnectionConfig
    adapter_type = "piper"

    @staticmethod
    def describe_simulated_robot(config: BaseModel, scene: Path) -> RobotInstance:
        config = cast("PiperConnectionConfig", config)
        definition = piper_definition(
            source=config.source,
            gripper=config.gripper,
        )
        home = config.initial_positions if config.initial_positions is not None else PIPER_HOME
        if len(home) != 6:
            raise ValueError("Piper initial_positions must contain one value per arm joint")
        initial = {f"{config.source}/joint{i}": q for i, q in enumerate(home, 1)}
        if config.gripper:
            initial[f"{config.source}/gripper"] = 0.07
        if config.base_pose is not None:
            instance = RobotInstance(
                definition,
                xyz=config.base_pose.position.to_tuple(),
                rpy=config.base_pose.orientation.to_euler().to_tuple(),
            )
        else:
            instance = scene_robot(scene, definition, config.spawn, default=(0.0, 0.0, 0.12))
        return replace(
            instance,
            initial_positions=initial,
            controller={
                "interface": "position",
                "startup_configuration": PiperConnection.describe_simulated_robot,
            },
        )
