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

import numpy as np
import pytest

from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.drone.px4.flight_control import (
    _build_mavsdk_odometry_values,
    _flu_to_frd_hamilton_quaternion,
    _flu_to_frd_vector,
    _InvalidExternalVisionSampleError,
    _seconds_to_microseconds,
    _transform_pose_covariance_to_frd,
    _transform_sensor_twist_to_base_frd,
)


def _odometry(*, ts: float = 12.3456789) -> Odometry:
    message = Odometry(ts=ts, frame_id="odom", child_frame_id="mid360_link")
    message.pose.position.x, message.pose.position.y, message.pose.position.z = (3.0, -2.0, 1.5)
    message.pose.orientation.x, message.pose.orientation.y = (0.0, 0.0)
    message.pose.orientation.z, message.pose.orientation.w = (0.0, 1.0)
    message.pose.covariance = np.eye(6).reshape(-1)
    message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = (1.0, 2.0, 3.0)
    message.twist.angular.x, message.twist.angular.y, message.twist.angular.z = (0.0, 0.0, 2.0)
    return message


def test_flu_values_convert_to_frd_axes_and_preserve_passive_yaw_sign() -> None:
    assert _flu_to_frd_vector((4.0, -5.0, 6.0)) == (4.0, 5.0, -6.0)
    assert _flu_to_frd_hamilton_quaternion((0.5, 0.5, 0.5, 0.5)) == (
        0.5,
        0.5,
        -0.5,
        -0.5,
    )
    assert _flu_to_frd_hamilton_quaternion((math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))) == (
        math.sqrt(0.5),
        0.0,
        0.0,
        -math.sqrt(0.5),
    )


def test_mounted_pose_applies_calibrated_inverse_to_nontrivial_sensor_pose() -> None:
    message = _odometry()
    message.pose.orientation.z = math.sqrt(0.5)
    message.pose.orientation.w = math.sqrt(0.5)

    values = _build_mavsdk_odometry_values(message)

    assert values.position_body == pytest.approx((3.02329, 2.02609, -1.44368), abs=1e-5)
    assert values.quaternion == pytest.approx((0.701057, -0.092296, -0.092296, -0.701057), abs=1e-6)


def test_odometry_builds_finite_mavsdk_values_with_packed_covariance_and_nan_velocity_sentinel() -> (
    None
):
    values = _build_mavsdk_odometry_values(_odometry())

    assert values.time_usec == 12_345_679
    assert len(values.pose_covariance) == 21
    assert len(values.velocity_covariance) == 1
    assert math.isnan(values.velocity_covariance[0])


def test_lever_arm_velocity_is_subtracted_before_flu_to_frd_conversion() -> None:
    linear_frd, angular_frd = _transform_sensor_twist_to_base_frd((1.0, 2.0, 3.0), (0.0, 0.0, 2.0))

    assert linear_frd == pytest.approx((0.234462, -1.947819, -3.168652), abs=1e-6)
    assert angular_frd == pytest.approx((-0.517638, 0.0, -1.931852), abs=1e-6)


def test_covariance_applies_jacobian_and_packs_upper_triangle() -> None:
    covariance = np.arange(36, dtype=float)

    packed = _transform_pose_covariance_to_frd(covariance, (1.0, 0.0, 0.0, 0.0))

    assert packed == pytest.approx(
        (
            -0.736629,
            -0.331503,
            -1.312502,
            2.247949,
            -3.214919,
            -4.18189,
            7.489287,
            8.344474,
            -9.321718,
            10.351947,
            11.382176,
            14.281328,
            -15.215354,
            16.218155,
            17.220955,
            21.0,
            -22.0,
            -23.0,
            28.0,
            29.0,
            35.0,
        ),
        abs=1e-6,
    )


def test_invalid_covariance_uses_single_nan_sentinel() -> None:
    covariance = _transform_pose_covariance_to_frd((1.0,), (1.0, 0.0, 0.0, 0.0))

    assert len(covariance) == 1
    assert math.isnan(covariance[0])


@pytest.mark.parametrize(
    ("frame_id", "child_frame_id"),
    (("map", "mid360_link"), ("odom", "camera_link")),
)
def test_odometry_rejects_frames_outside_the_pointlio_contract(
    frame_id: str, child_frame_id: str
) -> None:
    message = _odometry()
    message.frame_id = frame_id
    message.child_frame_id = child_frame_id

    with pytest.raises(_InvalidExternalVisionSampleError, match="odom -> mid360_link"):
        _build_mavsdk_odometry_values(message)


@pytest.mark.parametrize(
    "quaternion",
    ((0.0, 0.0, 0.0, 2.0), (math.nan, 0.0, 0.0, 1.0)),
)
def test_odometry_rejects_nonfinite_or_nonunit_quaternion_before_rotation(
    quaternion: tuple[float, float, float, float],
) -> None:
    message = _odometry()
    message.pose.orientation.x, message.pose.orientation.y = quaternion[:2]
    message.pose.orientation.z, message.pose.orientation.w = quaternion[2:]

    with pytest.raises(_InvalidExternalVisionSampleError, match="orientation quaternion"):
        _build_mavsdk_odometry_values(message)


@pytest.mark.parametrize("timestamp", (0.0, -0.1, math.nan, math.inf, -math.inf))
def test_timestamp_rejects_nonpositive_or_nonfinite_seconds(timestamp: float) -> None:
    with pytest.raises(_InvalidExternalVisionSampleError):
        _ = _seconds_to_microseconds(timestamp)


def test_timestamp_rounds_seconds_to_nearest_microsecond() -> None:
    assert _seconds_to_microseconds(12.3456789) == 12_345_679
