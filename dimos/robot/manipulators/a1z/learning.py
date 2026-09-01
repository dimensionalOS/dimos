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

"""The data contract shared by A1Z collection, training, and rollout."""

from pydantic import Field

from dimos.imitation.dataprep.core import (
    DataPrepConfig,
    FeatureSpec,
    OutputConfig,
    SyncConfig,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.robot.manipulators._modeling import joint_names
from dimos.robot.manipulators.a1z.config import A1Z_DOF


class A1ZLearningProfile(BaseConfig):
    """Typed A1Z learning schema used by the dataprep CLI and blueprints."""

    joint_names: tuple[str, ...] = Field(
        default=(*joint_names(A1Z_DOF, prefix="arm_joint"), "arm/gripper")
    )
    camera_width: int = 640
    camera_height: int = 480
    fps: float = 15.0
    robot_type: str = "galaxea_a1z"
    repo_id: str = "local/galaxea-a1z"

    def dataprep_config(self) -> DataPrepConfig:
        """Build the matching recording-to-LeRobot conversion config."""
        names = list(self.joint_names)
        return DataPrepConfig(
            source="",
            observation={
                "image": FeatureSpec(
                    stream="color_image",
                    field="data",
                    dtype="video",
                    shape=(self.camera_height, self.camera_width, 3),
                    names=["height", "width", "channels"],
                ),
                "joint_state": FeatureSpec(
                    stream="coordinator_joint_state",
                    field="position",
                    dtype="float32",
                    shape=(len(names),),
                    names=names,
                ),
            },
            action={
                "joint_target": FeatureSpec(
                    stream="coordinator_joint_state",
                    field="position",
                    dtype="float32",
                    shape=(len(names),),
                    names=names,
                )
            },
            sync=SyncConfig(
                anchor="image",
                rate_hz=self.fps,
                tolerance_ms=80.0,
            ),
            output=OutputConfig(
                format="lerobot",
                path=DataPrepConfig().output.path,
                metadata={"repo_id": self.repo_id, "robot_type": self.robot_type},
            ),
        )


A1Z_LEARNING_PROFILE = A1ZLearningProfile()
