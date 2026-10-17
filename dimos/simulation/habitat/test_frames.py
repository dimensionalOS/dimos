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

import numpy as np

from dimos.simulation.habitat.frames import (
    OPTICAL_QUAT_XYZW,
    R_ROS_HAB,
    R_ROS_OPT,
    habitat_heading,
    habitat_quat_from_yaw,
    position_to_habitat,
    position_to_ros,
    quat_to_ros,
    yaw_from_habitat_quat,
)


def test_basis_vectors():
    # habitat forward (-z) -> ros forward (+x)
    np.testing.assert_allclose(position_to_ros([0, 0, -1]), [1, 0, 0], atol=1e-12)
    # habitat up (+y) -> ros up (+z)
    np.testing.assert_allclose(position_to_ros([0, 1, 0]), [0, 0, 1], atol=1e-12)
    # habitat right (+x) -> ros right (-y)
    np.testing.assert_allclose(position_to_ros([1, 0, 0]), [0, -1, 0], atol=1e-12)


def test_rotations_are_proper():
    for r in (R_ROS_HAB, R_ROS_OPT):
        np.testing.assert_allclose(r @ r.T, np.eye(3), atol=1e-12)
        assert np.linalg.det(r) > 0  # right-handed, not a mirror


def test_position_round_trip():
    rng = np.random.default_rng(0)
    for _ in range(32):
        p = rng.normal(size=3)
        np.testing.assert_allclose(position_to_habitat(position_to_ros(p)), p, atol=1e-12)


def test_optical_to_body():
    # optical forward (+z) -> ros forward (+x); optical down (+y) -> ros down (-z)
    np.testing.assert_allclose(R_ROS_OPT @ [0, 0, 1], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(R_ROS_OPT @ [0, 1, 0], [0, 0, -1], atol=1e-12)
    np.testing.assert_allclose(R_ROS_OPT @ [1, 0, 0], [0, -1, 0], atol=1e-12)


def test_yaw_round_trip():
    for yaw in np.linspace(-np.pi + 1e-6, np.pi - 1e-6, 25):
        np.testing.assert_allclose(
            yaw_from_habitat_quat(habitat_quat_from_yaw(yaw)), yaw, atol=1e-9
        )


def test_quat_to_ros_yaw_only_stays_yaw_only():
    """A habitat yaw (about +y) must land as a pure ROS yaw (about +z)."""
    q_ros = quat_to_ros(habitat_quat_from_yaw(0.7))
    np.testing.assert_allclose(q_ros[:2], [0.0, 0.0], atol=1e-12)
    assert q_ros[2] > 0  # positive yaw stays positive


def test_measured_turn_left_quaternion():
    """The quaternion habitat actually reported after one turn_left(30 deg).

    Recorded on the live sim: w 0.915825 -> 0.780683 with only the y component
    populated, i.e. 47.15 deg -> 77.26 deg about habitat +y.
    """
    before = yaw_from_habitat_quat([0.915825247764587, 0, 0.401577025651932, 0])
    after = yaw_from_habitat_quat([0.78068345785141, 0, 0.624926626682281, 0])
    assert np.isclose(np.degrees(after - before), 30.0, atol=0.2)


def test_heading_matches_quaternion():
    """habitat_heading must agree with rotating (0,0,-1) by the yaw quaternion."""
    for yaw in np.linspace(-3.0, 3.0, 21):
        forward, left = habitat_heading(yaw)
        # forward/left are unit and orthogonal
        np.testing.assert_allclose(np.linalg.norm(forward), 1.0, atol=1e-12)
        np.testing.assert_allclose(np.linalg.norm(left), 1.0, atol=1e-12)
        np.testing.assert_allclose(forward @ left, 0.0, atol=1e-12)
        # and they map to ROS +x / +y as the yaw demands
        np.testing.assert_allclose(
            position_to_ros(forward), [np.cos(yaw), np.sin(yaw), 0.0], atol=1e-12
        )
        np.testing.assert_allclose(
            position_to_ros(left), [-np.sin(yaw), np.cos(yaw), 0.0], atol=1e-12
        )


def test_zero_yaw_is_ros_x_forward():
    forward, left = habitat_heading(0.0)
    np.testing.assert_allclose(forward, [0, 0, -1], atol=1e-12)
    np.testing.assert_allclose(position_to_ros(forward), [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(position_to_ros(left), [0, 1, 0], atol=1e-12)


def test_optical_quat_matches_the_matrix():
    """OPTICAL_QUAT_XYZW must be exactly R_ROS_OPT, or the camera renders mis-aimed."""
    x, y, z, w = OPTICAL_QUAT_XYZW
    np.testing.assert_allclose(x * x + y * y + z * z + w * w, 1.0, atol=1e-12)
    r = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    np.testing.assert_allclose(r, R_ROS_OPT, atol=1e-12)


def test_optical_frame_points_camera_forward_not_up():
    """The regression this frame exists for: +z of the optical frame is FORWARD."""
    # A pinhole looks down its own +z. Expressed in the body frame that must be +x.
    np.testing.assert_allclose(R_ROS_OPT @ [0, 0, 1], [1, 0, 0], atol=1e-12)
    # and emphatically not +z (up), which is what an identity rotation would give
    assert not np.allclose(R_ROS_OPT @ [0, 0, 1], [0, 0, 1])
