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

from dimos.robot.catalog.ufactory import xarm6, xarm7


def test_xarm7_robot_model_config_includes_limits():
    config = xarm7().to_robot_model_config()

    assert config.joint_limits_lower is not None
    assert config.joint_limits_upper is not None
    assert config.velocity_limits is not None
    assert len(config.joint_limits_lower) == len(config.joint_names) == 7
    assert len(config.joint_limits_upper) == len(config.joint_names) == 7
    assert len(config.velocity_limits) == len(config.joint_names) == 7


def test_xarm6_robot_model_config_includes_limits():
    config = xarm6().to_robot_model_config()

    assert config.joint_limits_lower is not None
    assert config.joint_limits_upper is not None
    assert config.velocity_limits is not None
    assert len(config.joint_limits_lower) == len(config.joint_names) == 6
    assert len(config.joint_limits_upper) == len(config.joint_names) == 6
    assert len(config.velocity_limits) == len(config.joint_names) == 6
