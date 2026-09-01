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

"""The data contract shared by OpenYAM collection, training, and rollout."""

from pathlib import Path

from pydantic import Field

from dimos.imitation.dataprep.core import (
    DataPrepConfig,
    FeatureSpec,
    OutputConfig,
    QualityConfig,
    SyncConfig,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_GRIPPER_JOINT,
    OPENYAM_JOINTS,
)


class OpenYamLearningProfile(BaseConfig):
    """One fixed observation/action schema for OpenYAM learning."""

    robot_type: str = "openyam"
    joint_names: tuple[str, ...] = tuple(OPENYAM_JOINTS)
    gripper_joint_name: str = OPENYAM_GRIPPER_JOINT
    fps: float = Field(default=30.0, gt=0)
    camera_width: int = Field(default=640, gt=0)
    camera_height: int = Field(default=480, gt=0)
    camera_channels: int = Field(default=3, gt=0)
    camera_frame_prefix: str = "wrist"
    camera_frame_id: str = "wrist_camera_link"
    image_feature: str = "observation.images.wrist"

    def dataprep_config(
        self,
        *,
        source: str | Path,
        output: Path,
        repo_id: str = "local/openyam-wrist",
    ) -> DataPrepConfig:
        """Build the matching native-recording to LeRobot conversion config."""
        joint_names = list(self.joint_names)
        return DataPrepConfig(
            source=str(source),
            observation={
                self.image_feature: FeatureSpec(
                    stream="color_image",
                    field="data",
                    dtype="video",
                    shape=(self.camera_height, self.camera_width, self.camera_channels),
                    names=["height", "width", "channels"],
                ),
                "observation.state": FeatureSpec(
                    stream="coordinator_joint_state",
                    field="position",
                    dtype="float32",
                    shape=(len(joint_names),),
                    names=joint_names,
                ),
            },
            action={
                "action": FeatureSpec(
                    stream="applied_joint_position_command",
                    field="position",
                    dtype="float32",
                    shape=(len(joint_names),),
                    names=joint_names,
                )
            },
            sync=SyncConfig(
                anchor=self.image_feature,
                rate_hz=self.fps,
                tolerance_ms=20.0,
            ),
            quality=QualityConfig(
                mode="strict",
                min_source_rate_ratio=0.95,
                max_camera_gap_ms=100.0,
                max_alignment_error_ms=20.0,
            ),
            output=OutputConfig(
                format="lerobot",
                path=output,
                metadata={"repo_id": repo_id, "robot_type": self.robot_type},
            ),
        )


OPENYAM_LEARNING_PROFILE = OpenYamLearningProfile()
