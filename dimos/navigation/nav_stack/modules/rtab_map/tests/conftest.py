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

"""Shared fixtures for the RtabMap validation tests."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.navigation.nav_stack.modules.rtab_map._rtab_runner import (
    LiteRtabRunner,
    LiteRtabRunnerConfig,
)


@pytest.fixture()
def lite_runner() -> LiteRtabRunner:
    """Lite runner with spec defaults applied."""
    return LiteRtabRunner(
        LiteRtabRunnerConfig(
            cell_size=0.1,
            ray_tracing=True,
            max_ground_angle_deg=45.0,
            ground_is_obstacle=False,
        )
    )


def identity_pose() -> np.ndarray:
    return np.eye(4)


def translated_pose(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> np.ndarray:
    """SE(3) pose with the given translation and identity rotation."""
    pose = np.eye(4)
    pose[:3, 3] = [x, y, z]
    return pose


def yawed_pose(yaw_rad: float, x: float = 0.0, y: float = 0.0) -> np.ndarray:
    """SE(3) pose with the given yaw and (x, y) translation."""
    pose = np.eye(4)
    pose[:3, :3] = np.array(
        [
            [np.cos(yaw_rad), -np.sin(yaw_rad), 0.0],
            [np.sin(yaw_rad), np.cos(yaw_rad), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    pose[0, 3] = x
    pose[1, 3] = y
    return pose
