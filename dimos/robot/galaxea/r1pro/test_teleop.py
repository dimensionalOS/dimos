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

import xml.etree.ElementTree as ET

import pytest

from dimos.control.tasks.teleop_ik_task.teleop_ik_task import create_task
from dimos.robot.galaxea.r1pro import config
from dimos.robot.galaxea.r1pro.teleop import arm_tasks


@pytest.fixture
def arm_source(tmp_path, monkeypatch):
    root = ET.Element("robot", name="r1_fixture")
    ET.SubElement(root, "link", name="base_link")

    def joint(name, parent, child, kind="revolute"):
        ET.SubElement(root, "link", name=child)
        node = ET.SubElement(root, "joint", name=name, type=kind)
        ET.SubElement(node, "parent", link=parent)
        ET.SubElement(node, "child", link=child)
        ET.SubElement(node, "origin", xyz="0 0 0.1")
        ET.SubElement(node, "axis", xyz="0 1 0")
        if kind != "fixed":
            ET.SubElement(node, "limit", lower="-2", upper="2", velocity="1", effort="10")

    joint("steer_motor_joint1", "base_link", "wheel")
    for side in ("left", "right"):
        parent = f"{side}_arm_base_link"
        joint(f"{side}_mount", "base_link", parent, "fixed")
        for i in range(1, 8):
            child = f"{side}_arm_link{i}"
            joint(f"{side}_arm_joint{i}", parent, child)
            parent = child
        joint(f"{side}_tool", parent, f"{side}_gripper_link", "fixed")
        for i in (1, 2):
            joint(f"{side}_gripper_finger_joint{i}", f"{side}_gripper_link", f"{side}_finger{i}")
    source = tmp_path / "robot.urdf"
    source.write_text(ET.tostring(root, encoding="unicode"))
    monkeypatch.setattr(config, "R1PRO_MODEL_PATH", source)
    monkeypatch.setattr(config, "R1PRO_PACKAGE_ROOT", tmp_path)


def test_each_arm_initializes_ik_without_chassis_or_opposite_arm_edits(arm_source):
    for task_config in arm_tasks(config.make_r1pro_arm_models()):
        task = create_task(task_config, {})
        try:
            task.start()
            model = task_config.params["robot_model"].model.load()
            movable = {joint.name for joint in model.joints if joint.type != "fixed"}
            assert movable == set(task_config.joint_names)
            assert len(movable) == 7
        finally:
            task.stop()
