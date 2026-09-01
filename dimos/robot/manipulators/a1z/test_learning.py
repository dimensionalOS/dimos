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

from dimos.imitation.dataprep.core import DataPrepProfile
from dimos.robot.manipulators.a1z.learning import A1Z_LEARNING_PROFILE


def test_a1z_dataprep_profile_is_valid() -> None:
    assert isinstance(A1Z_LEARNING_PROFILE, DataPrepProfile)

    config = A1Z_LEARNING_PROFILE.dataprep_config()

    assert config.sync.anchor == "image"
    assert config.sync.rate_hz == 15.0
    assert set(config.observation) == {"image", "joint_state"}
    assert set(config.action) == {"joint_target"}
    assert config.output.metadata["robot_type"] == "galaxea_a1z"
    assert config.output.metadata["repo_id"] == "local/galaxea-a1z"
