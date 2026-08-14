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
    _build_vision_position_estimate,
    _InvalidExternalVisionSampleError,
    _transform_pose_covariance_to_frd,
)


def _odometry() -> Odometry:
    message = Odometry(ts=12.3456789, frame_id="odom", child_frame_id="mid360_link")
    message.pose.position.x, message.pose.position.y, message.pose.position.z = (3.0, -2.0, 1.5)
    message.pose.orientation.w = 1.0
    message.pose.covariance = np.eye(6).reshape(-1)
    return message


def test_pointlio_odometry_builds_mavsdk_vision_estimate() -> None:
    estimate = _build_vision_position_estimate(_odometry())

    assert estimate.time_usec == 12_345_679
    assert (
        estimate.position_body.x_m,
        estimate.position_body.y_m,
        estimate.position_body.z_m,
    ) == pytest.approx((2.973909, 2.02329, -1.443681), abs=1e-6)
    assert len(estimate.pose_covariance.covariance_matrix) == 21


def test_invalid_covariance_uses_mavsdk_unknown_sentinel() -> None:
    covariance = _transform_pose_covariance_to_frd((1.0,), (1.0, 0.0, 0.0, 0.0))

    assert len(covariance) == 1
    assert math.isnan(covariance[0])


@pytest.mark.parametrize(
    ("frame_id", "child_frame_id"),
    (("map", "mid360_link"), ("odom", "camera_link")),
)
def test_external_vision_rejects_frames_outside_pointlio_contract(
    frame_id: str, child_frame_id: str
) -> None:
    message = _odometry()
    message.frame_id = frame_id
    message.child_frame_id = child_frame_id

    with pytest.raises(_InvalidExternalVisionSampleError, match="odom -> mid360_link"):
        _build_vision_position_estimate(message)


@pytest.mark.parametrize("timestamp", (0.0, -0.1, math.nan, math.inf))
def test_external_vision_rejects_invalid_timestamp(timestamp: float) -> None:
    message = _odometry()
    message.ts = timestamp

    with pytest.raises(_InvalidExternalVisionSampleError, match="timestamp"):
        _build_vision_position_estimate(message)


def test_external_vision_rejects_non_normalized_orientation() -> None:
    message = _odometry()
    message.pose.orientation.w = 2.0

    with pytest.raises(_InvalidExternalVisionSampleError, match="orientation quaternion"):
        _build_vision_position_estimate(message)


def test_external_vision_rejects_non_finite_orientation() -> None:
    message = _odometry()
    message.pose.orientation.x = math.nan

    with pytest.raises(_InvalidExternalVisionSampleError, match="must be finite"):
        _build_vision_position_estimate(message)


def test_external_vision_rejects_non_finite_position() -> None:
    message = _odometry()
    message.pose.position.x = math.inf

    with pytest.raises(_InvalidExternalVisionSampleError, match="position must be finite"):
        _build_vision_position_estimate(message)
