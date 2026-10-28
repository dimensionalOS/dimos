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

from dataclasses import replace
import math
import xml.etree.ElementTree as ET

import pytest

from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_MODEL,
    make_r1pro_model_config,
    make_r1pro_planar_model_config,
)
from dimos.robot.galaxea.r1pro.joints import TORSO_JOINTS, UPPER_BODY_JOINTS, coordinator_name


def test_installed_arm_limits_accept_feedback_and_preserve_vendor_margin(tmp_path):
    """The materialized planner URDF admits the real start, not extra hard-limit travel."""
    original = {name: (-1.7, 1.7) for name in UPPER_BODY_JOINTS}
    original["left_arm_joint3"] = (-2.3212894150, 2.3212894150)
    original["left_arm_joint4"] = (-2.0594934150, 0.3141934150)
    root = ET.Element("robot", name="limit_fixture")
    ET.SubElement(root, "link", name="base_link")
    for name, (lower, upper) in original.items():
        ET.SubElement(root, "link", name=f"{name}_link")
        joint = ET.SubElement(root, "joint", name=name, type="revolute")
        ET.SubElement(joint, "parent", link="base_link")
        ET.SubElement(joint, "child", link=f"{name}_link")
        ET.SubElement(joint, "axis", xyz="0 0 1")
        ET.SubElement(joint, "limit", lower=str(lower), upper=str(upper), effort="25", velocity="1")
    source = tmp_path / "robot.urdf"
    ET.ElementTree(root).write(source)
    # Only the external asset boundary is replaced; use production limit overrides
    # and renaming. No asset download, native planner, ROS, or robot connection.
    model = replace(R1PRO_MODEL, _source_path=source, _package_paths=(), _fixed_joints=())

    loaded = ET.fromstring(model.load().xml)
    limits = {
        joint.attrib["name"]: (
            float(joint.find("limit").attrib["lower"]),
            float(joint.find("limit").attrib["upper"]),
        )
        for joint in loaded.findall("joint")
    }

    expected = dict(original)
    expected["left_arm_joint3"] = (-2.3212894150, 2.3387427075)
    expected["left_arm_joint4"] = (-2.0594934150, 0.3316467075)
    assert limits == {coordinator_name(name): bounds for name, bounds in expected.items()}
    for name, measured, controller_upper in (
        ("left_arm_joint3", 2.33021, 2.356196),
        ("left_arm_joint4", 0.32809, 0.3491),
    ):
        lower, upper = limits[coordinator_name(name)]
        assert lower < measured < upper
        assert controller_upper - upper == pytest.approx(math.radians(1), abs=1e-8)


@pytest.mark.parametrize("make_config", [make_r1pro_model_config, make_r1pro_planar_model_config])
def test_torso_can_be_pose_targeted_without_selecting_arms(make_config):
    registry = PlanningGroupRegistry(make_config().planning_groups)

    selection = registry.select(["torso"])
    torso = selection.groups[0]

    assert torso.has_pose_target
    assert torso.base_link == "base_link"
    assert torso.tip_link == "torso_link4"
    assert selection.joint_names == tuple(coordinator_name(name) for name in TORSO_JOINTS)
