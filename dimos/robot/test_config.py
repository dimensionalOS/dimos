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

from __future__ import annotations

from pydantic import ValidationError
import pytest

from dimos.robot.config import RobotConfig


def test_robot_config_accepts_base_link() -> None:
    config = RobotConfig(name="arm", base_link="base")

    assert config.base_link == "base"


def test_robot_config_rejects_transient_base_pose_link_with_helpful_message() -> None:
    with pytest.raises(ValidationError) as exc_info:
        RobotConfig(name="arm", base_pose_link="base")

    message = str(exc_info.value)
    assert "RobotConfig.base_pose_link was removed" in message
    assert "base_link" in message
    assert "PlanningGroupDefinition.base_link" in message


def test_robot_config_rejects_legacy_end_effector_link_with_helpful_message() -> None:
    with pytest.raises(ValidationError) as exc_info:
        RobotConfig(name="arm", end_effector_link="tool")

    message = str(exc_info.value)
    assert "RobotConfig.end_effector_link was removed" in message
    assert "PlanningGroupDefinition.tip_link" in message


def test_robot_config_forbids_unknown_extra_fields() -> None:
    with pytest.raises(ValidationError) as exc_info:
        RobotConfig(name="arm", unexpected_field=True)

    assert "Extra inputs are not permitted" in str(exc_info.value)
