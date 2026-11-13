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
from __future__ import annotations

from typing import Any
import xml.etree.ElementTree as ET

from dimos.core.coordination.blueprints import Blueprint
from dimos.robot.manipulators.openarm.blueprints import mini_teleop
from dimos.robot.manipulators.openarm.config import (
    openarm_bimanual_model_config,
    openarm_urdf_joints,
)

_MODEL = openarm_bimanual_model_config(robot="arm")


def _atom(blueprint: Blueprint, module_type: type) -> Any:
    return next(atom for atom in blueprint.blueprints if atom.module is module_type)


def _urdf_limits() -> dict[str, tuple[float, float]]:
    limits = {}
    for joint in ET.fromstring(_MODEL.model.load().xml).iter("joint"):
        limit = joint.find("limit")
        if limit is not None:
            limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
    return limits


def test_mappings_drive_each_side_joint_for_joint_inside_the_urdf_limits() -> None:
    urdf_limits = _urdf_limits()
    for side in ("left", "right"):
        mapping = mini_teleop.MAPPINGS[side]
        assert mapping.joint_names() == tuple(f"arm/{joint}" for joint in openarm_urdf_joints(side))
        assert mapping.leader_joint_names() == (
            *[f"leader/{side}_joint_{i}" for i in range(1, 8)],
            f"leader/{side}_gripper",
        )
        for joint in mapping.joints:
            lower, upper = urdf_limits[joint.joint]
            assert lower <= joint.lower < joint.upper <= upper, joint.joint
            assert (joint.scale, joint.offset) == (1.0, 0.0)
        assert mapping.gripper is not None
        assert mapping.gripper.travel_radians == 1.0
    assert mini_teleop.MAPPINGS["left"].joints[3].lower == 0.0
