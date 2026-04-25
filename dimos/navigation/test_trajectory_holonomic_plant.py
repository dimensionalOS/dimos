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

"""P5-2 actuated holonomic plant: step response and determinism (T-14)."""

from __future__ import annotations

import math
import random

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_holonomic_plant import ActuatedHolonomicPlant, IntegratedHolonomicPlant


def _cmd_vx(v: float) -> Twist:
    return Twist(linear=Vector3(v, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))


def test_integrated_plant_matches_actuated_with_instant_dynamics() -> None:
    """Large accel and zero lag on actuated plant tracks ideal integration."""
    dt = 0.02
    ideal = IntegratedHolonomicPlant()
    act = ActuatedHolonomicPlant(
        linear_lag_time_constant_s=0.0,
        yaw_lag_time_constant_s=0.0,
        max_linear_accel_m_s2=1.0e6,
        max_yaw_accel_rad_s2=1.0e6,
    )
    u = _cmd_vx(0.3)
    for _ in range(50):
        ideal.step(u, dt)
        act.step(u, dt)
    assert act.x == pytest.approx(ideal.x)
    assert act.y == pytest.approx(ideal.y)
    assert act.yaw_rad == pytest.approx(ideal.yaw_rad)


def test_step_response_vx_settles_to_command() -> None:
    """First-order lag with generous accel: velocity approaches step command."""
    dt = 0.01
    plant = ActuatedHolonomicPlant(
        linear_lag_time_constant_s=0.15,
        yaw_lag_time_constant_s=0.15,
        max_linear_accel_m_s2=20.0,
        max_yaw_accel_rad_s2=20.0,
    )
    u = _cmd_vx(0.8)
    for _ in range(800):
        plant.step(u, dt)
    v = plant.twist_body_velocity
    assert v.linear.x == pytest.approx(0.8, abs=0.02)
    assert v.linear.y == pytest.approx(0.0, abs=1e-6)
    assert v.angular.z == pytest.approx(0.0, abs=1e-6)


def test_step_response_vx_monotonic_during_rise() -> None:
    """No overshoot: forward velocity never decreases while tracking a positive step."""
    dt = 0.01
    plant = ActuatedHolonomicPlant(
        linear_lag_time_constant_s=0.12,
        yaw_lag_time_constant_s=0.1,
        max_linear_accel_m_s2=15.0,
        max_yaw_accel_rad_s2=15.0,
    )
    u = _cmd_vx(0.5)
    prev_vx = plant.twist_body_velocity.linear.x
    for _ in range(400):
        plant.step(u, dt)
        vx = plant.twist_body_velocity.linear.x
        assert vx >= prev_vx - 1e-9
        prev_vx = vx


def test_step_response_rate_limited_ramp() -> None:
    """Zero lag: velocity ramps at max linear accel until command is reached."""
    dt = 0.05
    a_max = 0.4
    plant = ActuatedHolonomicPlant(
        linear_lag_time_constant_s=0.0,
        yaw_lag_time_constant_s=0.0,
        max_linear_accel_m_s2=a_max,
        max_yaw_accel_rad_s2=1.0,
    )
    u = _cmd_vx(1.0)
    samples: list[float] = []
    for _ in range(8):
        plant.step(u, dt)
        samples.append(plant.twist_body_velocity.linear.x)
    # Each step can add at most a_max * dt to vx when below command
    for i in range(1, len(samples)):
        assert samples[i] - samples[i - 1] == pytest.approx(a_max * dt, rel=0, abs=1e-9)


def test_determinism_two_runs_identical() -> None:
    dt = 0.01
    n = 300
    u = _cmd_vx(0.25)

    def run() -> tuple[float, float, float, float, float, float]:
        p = ActuatedHolonomicPlant(
            x=0.1,
            y=-0.2,
            yaw_rad=0.05,
            linear_lag_time_constant_s=0.08,
            yaw_lag_time_constant_s=0.1,
            max_linear_accel_m_s2=3.0,
            max_yaw_accel_rad_s2=2.5,
        )
        for _ in range(n):
            p.step(u, dt)
        t = p.twist_body_velocity
        return (p.x, p.y, p.yaw_rad, t.linear.x, t.linear.y, t.angular.z)

    a = run()
    b = run()
    assert a == b


def test_bounded_noise_reproducible_with_seed() -> None:
    dt = 0.02
    n = 200
    u = _cmd_vx(0.4)

    def run(rng: random.Random) -> tuple[float, float, float]:
        p = ActuatedHolonomicPlant(
            linear_lag_time_constant_s=0.05,
            yaw_lag_time_constant_s=0.05,
            max_linear_accel_m_s2=10.0,
            max_yaw_accel_rad_s2=10.0,
            noise_rng=rng,
            noise_linear_max_m_s=0.02,
            noise_yaw_max_rad_s=0.01,
        )
        for _ in range(n):
            p.step(u, dt)
        t = p.twist_body_velocity
        return (p.x, t.linear.x, t.angular.z)

    r1 = random.Random(12345)
    r2 = random.Random(12345)
    assert run(r1) == run(r2)


def test_actuated_rejects_negative_lag() -> None:
    with pytest.raises(ValueError, match="linear_lag_time_constant_s"):
        ActuatedHolonomicPlant(linear_lag_time_constant_s=-0.1)


def test_actuated_rejects_noise_without_rng() -> None:
    with pytest.raises(ValueError, match="noise_rng"):
        ActuatedHolonomicPlant(noise_linear_max_m_s=0.01)


def test_measured_sample_reports_realized_twist() -> None:
    plant = ActuatedHolonomicPlant(
        linear_lag_time_constant_s=0.2,
        yaw_lag_time_constant_s=0.2,
        max_linear_accel_m_s2=50.0,
        max_yaw_accel_rad_s2=50.0,
    )
    cmd = _cmd_vx(1.0)
    plant.step(cmd, 0.05)
    m = plant.measured_sample(0.05, cmd)
    assert m.twist_body.linear.x == pytest.approx(plant.twist_body_velocity.linear.x)
    assert m.pose_plan.position.x == pytest.approx(plant.x)
    assert math.isfinite(m.pose_plan.position.y)
