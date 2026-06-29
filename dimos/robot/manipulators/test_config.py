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

"""Tests for manipulator planning model configs."""

from __future__ import annotations

from dimos.robot.manipulators.a750.config import make_a750_model_config
from dimos.robot.manipulators.piper.config import make_piper_model_config
from dimos.robot.manipulators.xarm.config import make_xarm6_model_config


def test_a750_model_config_applies_joint_prefix_mapping() -> None:
    config = make_a750_model_config(name="arm", joint_prefix="arm_")

    assert config.joint_name_mapping == {
        "arm_joint1": "joint1",
        "arm_joint2": "joint2",
        "arm_joint3": "joint3",
        "arm_joint4": "joint4",
        "arm_joint5": "joint5",
        "arm_joint6": "joint6",
    }
    assert config.get_coordinator_joint_names() == [
        "arm_joint1",
        "arm_joint2",
        "arm_joint3",
        "arm_joint4",
        "arm_joint5",
        "arm_joint6",
    ]


def test_piper_model_config_applies_joint_prefix_mapping() -> None:
    config = make_piper_model_config(name="arm", joint_prefix="arm_")

    assert config.joint_name_mapping == {
        "arm_joint1": "joint1",
        "arm_joint2": "joint2",
        "arm_joint3": "joint3",
        "arm_joint4": "joint4",
        "arm_joint5": "joint5",
        "arm_joint6": "joint6",
    }
    assert config.get_coordinator_joint_names() == [
        "arm_joint1",
        "arm_joint2",
        "arm_joint3",
        "arm_joint4",
        "arm_joint5",
        "arm_joint6",
    ]


def test_xarm_model_config_applies_joint_prefix_mapping() -> None:
    config = make_xarm6_model_config(name="arm", joint_prefix="arm_")

    assert config.joint_name_mapping == {
        "arm_joint1": "joint1",
        "arm_joint2": "joint2",
        "arm_joint3": "joint3",
        "arm_joint4": "joint4",
        "arm_joint5": "joint5",
        "arm_joint6": "joint6",
    }
    assert config.get_coordinator_joint_names() == [
        "arm_joint1",
        "arm_joint2",
        "arm_joint3",
        "arm_joint4",
        "arm_joint5",
        "arm_joint6",
    ]
