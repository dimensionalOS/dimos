# Copyright 2025-2026 Dimensional Inc.
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

"""xArm prepared-model configuration tests."""

from dimos.manipulation.planning.spec.validation import validate_robot_model_config
from dimos.robot.manipulators.xarm.config import make_dual_xarm6_model_config


def test_dual_xarm6_is_one_prepared_model_with_canonical_groups() -> None:
    config = make_dual_xarm6_model_config()

    model = validate_robot_model_config(config)

    assert model.root_link == "world"
    assert model.actuated_joint_names == config.joint_names
    assert [group.name for group in config.planning_groups] == [
        "left_arm",
        "right_arm",
        "both_arms",
    ]
    assert config.planning_groups[0].joint_names == tuple(config.joint_names[:6])
    assert config.planning_groups[1].joint_names == tuple(config.joint_names[6:])
