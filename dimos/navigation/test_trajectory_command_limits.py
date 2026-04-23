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

"""Tests for ``trajectory_command_limits`` (P1-3) and ``trajectory_controller`` (P1-2)."""

import math

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_command_limits import HolonomicCommandLimits, clamp_holonomic_cmd_vel
from dimos.navigation.trajectory_controller import (
    ConfigurableTrajectoryController,
    TrajectoryController,
)
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample

_LIMITS = HolonomicCommandLimits(
    max_planar_speed_m_s=1.0,
    max_yaw_rate_rad_s=0.5,
    max_planar_linear_accel_m_s2=2.0,
    max_yaw_accel_rad_s2=1.0,
)


def test_limits_reject_non_finite_or_negative() -> None:
    with pytest.raises(ValueError, match="max_planar_speed_m_s"):
        HolonomicCommandLimits(
            max_planar_speed_m_s=-1.0,
            max_yaw_rate_rad_s=1.0,
            max_planar_linear_accel_m_s2=1.0,
            max_yaw_accel_rad_s2=1.0,
        )


def test_saturate_planar_speed() -> None:
    prev = Twist()
    raw = Twist(linear=Vector3(2.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
    out = clamp_holonomic_cmd_vel(
        prev,
        raw,
        _LIMITS,
        1.0,
    )
    assert out.linear.x == pytest.approx(1.0)
    assert out.linear.y == pytest.approx(0.0)


def test_saturate_planar_speed_holonomic_direction() -> None:
    """Speed cap scales (vx, vy) together, preserving direction."""
    prev = Twist()
    raw = Twist(
        linear=Vector3(3.0, 4.0, 0.0),
        angular=Vector3(0.0, 0.0, 0.0),
    )
    out = clamp_holonomic_cmd_vel(
        prev,
        raw,
        _LIMITS,
        1.0,
    )
    v_max = 1.0
    n = math.hypot(3.0, 4.0)
    assert out.linear.x == pytest.approx(3.0 / n * v_max)
    assert out.linear.y == pytest.approx(4.0 / n * v_max)


def test_acceleration_limits_one_axis_from_rest() -> None:
    prev = Twist()
    raw = Twist(linear=Vector3(1.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
    a_max = 2.0
    dt = 0.1
    limits = HolonomicCommandLimits(
        max_planar_speed_m_s=10.0,
        max_yaw_rate_rad_s=10.0,
        max_planar_linear_accel_m_s2=a_max,
        max_yaw_accel_rad_s2=10.0,
    )
    out = clamp_holonomic_cmd_vel(prev, raw, limits, dt)
    assert out.linear.x == pytest.approx(a_max * dt)


def test_slew_uses_previous_command() -> None:
    prev = Twist(
        linear=Vector3(0.8, 0.0, 0.0),
        angular=Vector3(0.0, 0.0, 0.0),
    )
    raw = Twist(
        linear=Vector3(1.0, 0.0, 0.0),
        angular=Vector3(0.0, 0.0, 0.0),
    )
    a_max = 1.0
    dt = 0.1
    limits = HolonomicCommandLimits(
        max_planar_speed_m_s=2.0,
        max_yaw_rate_rad_s=2.0,
        max_planar_linear_accel_m_s2=a_max,
        max_yaw_accel_rad_s2=10.0,
    )
    out = clamp_holonomic_cmd_vel(prev, raw, limits, dt)
    assert out.linear.x == pytest.approx(0.8 + a_max * dt)


def test_yaw_rate_and_acceleration() -> None:
    prev = Twist(angular=Vector3(0.0, 0.0, 0.0))
    raw = Twist(angular=Vector3(0.0, 0.0, 1.0))
    limits = HolonomicCommandLimits(
        max_planar_speed_m_s=1.0,
        max_yaw_rate_rad_s=0.5,
        max_planar_linear_accel_m_s2=1.0,
        max_yaw_accel_rad_s2=5.0,
    )
    out = clamp_holonomic_cmd_vel(prev, raw, limits, 1.0)
    assert out.angular.z == pytest.approx(0.5)


def test_yaw_acceleration_step() -> None:
    prev = Twist(angular=Vector3(0.0, 0.0, 0.0))
    raw = Twist(angular=Vector3(0.0, 0.0, 1.0))
    limits = HolonomicCommandLimits(
        max_planar_speed_m_s=1.0,
        max_yaw_rate_rad_s=10.0,
        max_planar_linear_accel_m_s2=1.0,
        max_yaw_accel_rad_s2=2.0,
    )
    out = clamp_holonomic_cmd_vel(prev, raw, limits, 0.1)
    assert out.angular.z == pytest.approx(0.2)


def test_rejects_non_positive_dt() -> None:
    with pytest.raises(ValueError, match="dt_s"):
        clamp_holonomic_cmd_vel(Twist(), Twist(), _LIMITS, 0.0)


def test_passes_through_unmodeled_vector_components() -> None:
    """linear.z, angular x/y come from raw_cmd, not from previous_cmd."""
    prev = Twist(
        linear=Vector3(0.0, 0.0, 1.0),
        angular=Vector3(0.0, 0.0, 0.0),
    )
    raw = Twist(
        linear=Vector3(0.0, 0.0, 2.0),
        angular=Vector3(0.1, -0.2, 0.0),
    )
    out = clamp_holonomic_cmd_vel(
        prev,
        raw,
        HolonomicCommandLimits(
            max_planar_speed_m_s=1.0,
            max_yaw_rate_rad_s=1.0,
            max_planar_linear_accel_m_s2=10.0,
            max_yaw_accel_rad_s2=10.0,
        ),
        0.05,
    )
    assert out.linear.z == pytest.approx(2.0)
    assert out.angular.x == pytest.approx(0.1)
    assert out.angular.y == pytest.approx(-0.2)


class _StubController:
    def __init__(self) -> None:
        self.reset_count = 0
        self.limits: HolonomicCommandLimits | None = None

    def control(
        self,
        reference: TrajectoryReferenceSample,
        measurement: TrajectoryMeasuredSample,
    ) -> Twist:
        return reference.twist_body

    def reset(self) -> None:
        self.reset_count += 1

    def configure(self, limits: HolonomicCommandLimits) -> None:
        self.limits = limits


def test_protocol_runtime_check() -> None:
    stub: TrajectoryController = _StubController()
    assert isinstance(stub, TrajectoryController)
    assert isinstance(stub, ConfigurableTrajectoryController)
    c = _StubController()
    c.configure(
        HolonomicCommandLimits(1.0, 1.0, 1.0, 1.0),
    )
    assert c.limits is not None


class _MinimalController:
    def control(
        self,
        reference: TrajectoryReferenceSample,
        measurement: TrajectoryMeasuredSample,
    ) -> Twist:
        return Twist()

    def reset(self) -> None:
        pass


def test_configurable_is_optional_on_runtime_check() -> None:
    m = _MinimalController()
    assert isinstance(m, TrajectoryController)
    assert not isinstance(m, ConfigurableTrajectoryController)
