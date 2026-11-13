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

import xml.etree.ElementTree as ET

from dimos.robot.manipulators.openarm.config import (
    OPENARM_MINI_LEADERS,
    openarm_bimanual_model_config,
    openarm_urdf_joints,
)

_MODEL = openarm_bimanual_model_config(robot="arm")


def _urdf_limits() -> dict[str, tuple[float, float]]:
    limits = {}
    for joint in ET.fromstring(_MODEL.model.load().xml).iter("joint"):
        limit = joint.find("limit")
        if limit is not None:
            limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
    return limits


def test_each_leader_drives_its_side_joint_for_joint_inside_the_urdf_limits() -> None:
    urdf_limits = _urdf_limits()
    for bus in OPENARM_MINI_LEADERS.buses:
        assert [motor.joint for motor in bus.motors] == openarm_urdf_joints(bus.name)
        for motor in bus.motors:
            lower, upper = urdf_limits[f"arm/{motor.joint}"]
            assert lower <= motor.lower < motor.upper <= upper, motor.joint
        assert bus.trigger is not None
        assert bus.trigger.travel_radians == 1.0
