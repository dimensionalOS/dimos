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

from dimos.robot.manipulators.a1z.config import (
    A1Z_GRIPPER_SWEEP_VOLUME,
    a1z_gripper_config,
    make_a1z_model_config,
)


def test_gripper_model_restores_centered_end_effector_frame() -> None:
    config = make_a1z_model_config(has_gripper=True)

    assert config.planning_groups[0].tip_link == "gripper_eef_link"


def test_a1z_gripper_config_closes_from_open_stroke_to_zero() -> None:
    gripper = a1z_gripper_config()

    assert gripper.finger_joints == {
        "gripper_finger_left_joint": (0.05, 0.0),
        "gripper_finger_rIght_joint": (-0.05, 0.0),
    }
    assert gripper.closing_steps == 8
    steps = gripper.joint_positions()
    assert steps[0] == {"gripper_finger_left_joint": 0.05, "gripper_finger_rIght_joint": -0.05}
    assert steps[-1] == {"gripper_finger_left_joint": 0.0, "gripper_finger_rIght_joint": 0.0}


def test_a1z_gripper_sweep_volume_centers_distal_fingertip_pad() -> None:
    fingertip_depth = A1Z_GRIPPER_SWEEP_VOLUME["fingertip_depth"]
    extents_open = A1Z_GRIPPER_SWEEP_VOLUME["extents_open"]
    offset_open = A1Z_GRIPPER_SWEEP_VOLUME["offset_open"]

    assert extents_open == (0.100, 0.036, 0.0350)
    assert offset_open[2] == pytest.approx(fingertip_depth - extents_open[2] / 2.0, abs=1e-4)


def test_flange_model_uses_native_flange_tip() -> None:
    config = make_a1z_model_config(has_gripper=False)

    assert config.planning_groups[0].tip_link == "arm_link6"


@pytest.mark.self_hosted
def test_gripper_model_contains_configured_end_effector_frame() -> None:
    config = make_a1z_model_config(has_gripper=True)
    root = ET.fromstring(config.model.load().xml)
    joint = root.find("joint[@name='gripper_eef_link_joint']")

    assert root.find("link[@name='gripper_eef_link']") is not None
    assert joint is not None
    assert joint.attrib == {"name": "gripper_eef_link_joint", "type": "fixed"}
    assert joint.find("parent").attrib == {"link": "arm_link6"}
    assert joint.find("child").attrib == {"link": "gripper_eef_link"}
    origin = joint.find("origin")
    assert origin is not None
    assert [float(value) for value in origin.attrib["xyz"].split()] == [0.0727, 0.0, 0.0]
    assert [float(value) for value in origin.attrib["rpy"].split()] == [0.0, 0.0, 0.0]
    joint4 = config.model.load().get_joint("arm_joint4")
    assert joint4 is not None
    assert joint4.lower == pytest.approx(-1.484)
    assert joint4.upper == pytest.approx(1.484)
