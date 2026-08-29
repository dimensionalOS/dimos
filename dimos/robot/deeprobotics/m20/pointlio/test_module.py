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

"""Configuration contract tests for the native M20 Point-LIO adapter."""

from pydantic import ValidationError
import pytest

from dimos.robot.deeprobotics.m20.pointlio.module import M20PointLioConfig


def test_pointlio_matches_verified_m20_sensor_contract() -> None:
    config = M20PointLioConfig()

    assert config.base_frame == "base_link"
    assert config.world_frame == "odom"
    assert config.scan_line == 192
    assert config.scan_rate == 10
    assert config.msr_freq == 200.0
    assert config.imu_time_inte == 0.005
    assert config.lidar_timeout_s == 0.5
    assert config.imu_timeout_s == 0.5
    assert config.estimate_timeout_s == 0.5
    assert config.max_cloud_points == 20_000
    assert config.extrinsic_t == [0.0, 0.0, 0.0]
    assert config.extrinsic_r == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def test_pointlio_rejects_cloud_limit_above_native_static_capacity() -> None:
    with pytest.raises(ValidationError):
        M20PointLioConfig(max_cloud_points=100_001)
