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

"""xArm robot-description source contracts."""

import xml.etree.ElementTree as ET

import pytest

from dimos.robot.assets.processing import load_robot_description
from dimos.robot.manipulators.xarm.config import XARM_ROS2_REF, make_xarm6_model_config


def test_xarm_description_is_pinned_before_malformed_gripper_xacro() -> None:
    assert XARM_ROS2_REF == "5bb832f72ca665f1236a9d8ed1c3a82f308db489"


@pytest.mark.self_hosted
def test_xarm_gripper_xacro_has_only_whitespace_element_tails() -> None:
    config = make_xarm6_model_config(name="arm", add_gripper=True)
    description = load_robot_description(
        config.model_path,
        package_paths=config.package_paths,
        xacro_args=config.xacro_args,
    )
    root = ET.fromstring(description.xml)

    assert root.find("link[@name='link_tcp']") is not None
    for element in root.iter():
        assert element.tail is None or not element.tail.strip()
    for child in root:
        ET.fromstring(ET.tostring(child, encoding="unicode"))
