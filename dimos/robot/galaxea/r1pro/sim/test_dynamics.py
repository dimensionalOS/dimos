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

import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from dimos.robot.assets.model import LoadedRobotModel
from dimos.robot.galaxea.r1pro.sim.config import UPPER_JOINTS
from dimos.robot.galaxea.r1pro.sim.dynamics import UpperBodyServo, coupled_wrist_bounds


@pytest.fixture
def servo():
    # Independent 1 kg links with a 1 m COM lever provide an analytic gravity
    # oracle without downloading meshes or relying on a second dynamics engine.
    root = ET.Element("robot", name="gravity_fixture")
    ET.SubElement(root, "link", name="base")
    for i, name in enumerate(UPPER_JOINTS):
        link = ET.SubElement(root, "link", name=f"link{i}")
        inertia = ET.SubElement(link, "inertial")
        ET.SubElement(inertia, "origin", xyz="1 0 0")
        ET.SubElement(inertia, "mass", value="1")
        ET.SubElement(inertia, "inertia", ixx="1", iyy="1", izz="1", ixy="0", ixz="0", iyz="0")
        joint = ET.SubElement(root, "joint", name=name, type="revolute")
        ET.SubElement(joint, "parent", link="base")
        ET.SubElement(joint, "child", link=f"link{i}")
        ET.SubElement(joint, "axis", xyz="0 1 0")
        ET.SubElement(joint, "limit", lower="-1.3", upper="1.8", effort="100", velocity="5")
    return UpperBodyServo(LoadedRobotModel(ET.tostring(root, encoding="unicode"), None, {}))


def test_trajectory_reaches_target_with_bounded_speed_acceleration_and_smooth_retarget(servo):
    target = np.zeros(18)
    target[4] = 1.0
    measured = np.zeros(18)
    velocities = [0.0]
    positions = []
    for i in range(1000):
        if i == 150:
            target[4] = -0.4
        q, v = servo.step(target, measured)
        positions.append(q[4])
        velocities.append(v[4])
    assert max(abs(np.asarray(velocities))) <= 1.5 + 1e-9
    assert max(abs(np.diff(velocities) / 0.004)) <= 2.25 + 1e-8
    assert max(abs(np.diff(positions))) <= 1.5 * 0.004 + 1e-9
    assert positions[-1] == pytest.approx(-0.4, abs=1e-9)


def test_startup_uses_measured_pose_instead_of_commanding_a_jump_from_zero(servo):
    measured = np.full(18, 0.2)
    q, v = servo.step(measured.copy(), measured)
    np.testing.assert_allclose(q, measured, atol=1e-12)
    np.testing.assert_allclose(v, 0, atol=1e-12)


def test_gravity_uses_measured_torso_and_commanded_arms(servo):
    measured, command = np.full(18, 0.7), np.full(18, 0.2)
    tau = servo.gravity(command, measured)
    np.testing.assert_allclose(tau[:4], -9.81 * math.cos(0.7), atol=1e-10)
    np.testing.assert_allclose(tau[4:], -9.81 * math.cos(0.2), atol=1e-10)


def test_wrist_envelope_matches_vendor_endpoints_and_mirrors_sides():
    assert coupled_wrist_bounds(0, right=False) == pytest.approx((-3.4085984, 1.83624))
    left = coupled_wrist_bounds(-1, right=False)
    assert left == pytest.approx((-0.4843136, 0.59928), abs=1e-7)
    assert coupled_wrist_bounds(-1, right=True) == pytest.approx((-left[1], -left[0]))
    assert coupled_wrist_bounds(0.1, right=False) == (-np.inf, np.inf)


def test_coupled_wrist_clamps_j7_when_j6_folds(servo):
    target = np.zeros(18)
    target[9], target[10] = -1, -1
    target[16], target[17] = -1, 1
    bounded = servo.bound(target)
    assert bounded[9] == -1
    assert bounded[10] == pytest.approx(coupled_wrist_bounds(-1, right=False)[0])
    assert bounded[17] == pytest.approx(coupled_wrist_bounds(-1, right=True)[1])


def test_near_finished_retarget_does_not_stall_the_control_loop(servo):
    # State captured from the failed live sim's zero-duration synchronization.
    q = np.zeros(18)
    q[11:] = [
        2.245758362033603e-5,
        2.171844211248659e-6,
        -1.680237542019622e-7,
        0.1499931922692374,
        -1.081033619452205e-7,
        -5.700233652973792e-5,
        -1.033893079326835e-6,
    ]
    servo.initialized = True
    for group, _, inp, _ in servo.groups:
        inp.current_position = q[group].tolist()
        inp.current_velocity = [3.790800585945798e-16] * len(inp.current_position)
        inp.current_acceleration = [-1.316628013212084e-15] * len(inp.current_position)
    target = q + 1e-16
    result, velocity = servo.step(target, q)
    np.testing.assert_allclose(result, target, atol=1e-12)
    np.testing.assert_array_equal(velocity, 0)
    target[14] += 0.1
    moved, velocity = servo.step(target, q)
    assert moved[14] > result[14]
    assert velocity[14] > 0
