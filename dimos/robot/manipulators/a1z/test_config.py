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

"""A1Z model configuration contracts."""

import xml.etree.ElementTree as ET

import pytest

from dimos.robot.assets.processing import FixedFrameDefinition, load_urdf
from dimos.robot.manipulators.a1z.config import make_a1z_model_config


def test_gripper_model_restores_centered_end_effector_frame() -> None:
    config = make_a1z_model_config(has_gripper=True)

    assert config.planning_groups[0].tip_link == "gripper_eef_link"
    assert config.additional_fixed_frames == [
        FixedFrameDefinition(
            name="gripper_eef_link",
            parent="arm_link6",
            xyz=(0.0727, 0.0, 0.0),
        )
    ]


def test_flange_model_uses_native_flange_tip() -> None:
    config = make_a1z_model_config(has_gripper=False)

    assert config.planning_groups[0].tip_link == "arm_link6"
    assert config.additional_fixed_frames == []


@pytest.mark.self_hosted
def test_gripper_model_contains_configured_end_effector_frame() -> None:
    config = make_a1z_model_config(has_gripper=True)
    description = load_urdf(
        config.urdf_path,
        package_paths=config.package_paths,
        xacro_args=config.xacro_args,
        additional_fixed_frames=tuple(config.additional_fixed_frames),
    )
    root = ET.fromstring(description.urdf_xml)
    joint = root.find("joint[@name='gripper_eef_link_joint']")

    assert root.find("link[@name='gripper_eef_link']") is not None
    assert joint is not None
    assert joint.get("type") == "fixed"
    parent = joint.find("parent")
    child = joint.find("child")
    origin = joint.find("origin")
    assert parent is not None and parent.get("link") == "arm_link6"
    assert child is not None and child.get("link") == "gripper_eef_link"
    assert origin is not None and origin.get("xyz") == "0.0727 0.0 0.0"
