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

"""Configuration contract tests for the native M20 ROS bridge."""

from pydantic import ValidationError
import pytest

from dimos.robot.deeprobotics.m20.bridge.module import M20ROSBridgeConfig


def test_bridge_requires_five_missed_nominal_clouds_before_stale() -> None:
    config = M20ROSBridgeConfig()

    assert config.lidar_timeout_s == 0.5


def test_bridge_normalizes_the_vendor_mislabeled_cloud_frame() -> None:
    config = M20ROSBridgeConfig()

    assert config.cloud_frame == "base_link"


def test_bridge_rejects_nonpositive_lidar_timeout() -> None:
    with pytest.raises(ValidationError):
        M20ROSBridgeConfig(lidar_timeout_s=0.0)
