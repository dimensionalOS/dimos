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

from dimos.robot.manipulators.openyam.config import OPENYAM_JOINTS
from dimos.robot.manipulators.openyam.learning import OPENYAM_LEARNING_PROFILE


def test_openyam_profile_builds_matching_observation_and_action_schema() -> None:
    profile = OPENYAM_LEARNING_PROFILE

    config = profile.dataprep_config()

    assert config.sync.rate_hz == profile.fps
    assert config.sync.anchor == profile.image_feature
    assert config.observation["observation.state"].names == OPENYAM_JOINTS
    assert config.action["action"].names == OPENYAM_JOINTS
    assert config.observation[profile.image_feature].shape == (480, 640, 3)
    assert config.output.metadata["robot_type"] == "openyam"
