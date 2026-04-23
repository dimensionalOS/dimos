# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on the "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for ``dimos.navigation.trajectory_types`` (P1-1)."""

from dataclasses import FrozenInstanceError

import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample


def _sample_pose() -> Pose:
    return Pose(1.0, 2.0, 0.0)


def _sample_twist() -> Twist:
    return Twist(linear=Vector3(0.3, 0.4, 0.0), angular=Vector3(0.0, 0.0, 0.1))


def test_reference_sample_defensive_pose_twist_copy() -> None:
    pose = _sample_pose()
    twist = _sample_twist()
    row = TrajectoryReferenceSample(time_s=0.25, pose_plan=pose, twist_body=twist)
    pose.position.x = 99.0
    twist.linear.x = 99.0
    assert row.pose_plan.position.x == pytest.approx(1.0)
    assert row.twist_body.linear.x == pytest.approx(0.3)


def test_measured_sample_defensive_copy() -> None:
    pose = _sample_pose()
    twist = _sample_twist()
    row = TrajectoryMeasuredSample(time_s=1.0, pose_plan=pose, twist_body=twist)
    pose.position.y = -50.0
    assert row.pose_plan.position.y == pytest.approx(2.0)


def test_frozen_dataclasses_reject_mutation() -> None:
    row = TrajectoryReferenceSample(0.0, _sample_pose(), _sample_twist())
    with pytest.raises(FrozenInstanceError):
        row.time_s = 1.0  # type: ignore[misc]


def test_trajectory_types_module_docstring_frames() -> None:
    """Guardrail: frame notes stay aligned with trajectory_metrics vocabulary."""
    import dimos.navigation.trajectory_types as tt

    doc = tt.__doc__ or ""
    assert "pose_errors_vs_reference" in doc
    assert "cmd_vel" in doc
    assert "commanded_planar_speed" in doc
