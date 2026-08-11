# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
import sys

import numpy as np
import pytest

RUNTIME_ROOT = Path(__file__).parent / "container/runtime"
if str(RUNTIME_ROOT) not in sys.path:
    sys.path.insert(0, str(RUNTIME_ROOT))

from vlnce_runtime.motion import (
    PlanarMotionError,
    integrate_planar,
    record_accepted_motion,
)


def test_fixed_period_planar_motion_uses_habitat_axes_and_yaw() -> None:
    requested, rotation = integrate_planar(
        [1.0, 2.0, 3.0],
        [0.0, 0.0, 0.0, 1.0],
        linear_x=0.4,
        linear_y=0.2,
        angular_z=np.pi,
        period_seconds=0.5,
    )

    assert requested == pytest.approx([0.9, 2.0, 2.8])
    assert rotation == pytest.approx([0.0, np.sqrt(0.5), 0.0, np.sqrt(0.5)])


def test_motion_rotates_local_translation_into_world_coordinates() -> None:
    requested, _ = integrate_planar(
        [0.0, 0.0, 0.0],
        [0.0, np.sqrt(0.5), 0.0, np.sqrt(0.5)],
        linear_x=1.0,
        linear_y=0.0,
        angular_z=0.0,
        period_seconds=0.1,
    )

    assert requested == pytest.approx([-0.1, 0.0, 0.0])


def test_each_accepted_motion_is_recorded_and_collision_clipping_is_visible() -> None:
    trajectory = [[0.0, 0.0, 0.0]]

    first_collision = record_accepted_motion(
        trajectory, requested=[0.0, 0.0, -0.1], accepted=[0.0, 0.0, -0.1]
    )
    second_collision = record_accepted_motion(
        trajectory, requested=[0.0, 0.0, -0.2], accepted=[0.0, 0.0, -0.15]
    )

    assert first_collision is False
    assert second_collision is True
    assert trajectory == [[0.0, 0.0, 0.0], [0.0, 0.0, -0.1], [0.0, 0.0, -0.15]]


def test_invalid_period_or_accepted_pose_cannot_silently_lose_trajectory() -> None:
    with pytest.raises(PlanarMotionError, match="positive period"):
        integrate_planar([0, 0, 0], [0, 0, 0, 1], 1.0, 0.0, 0.0, 0.0)

    trajectory = [[0.0, 0.0, 0.0]]
    with pytest.raises(PlanarMotionError, match="invalid accepted position"):
        record_accepted_motion(trajectory, [1, 0, 0], [np.nan, 0, 0])
    assert trajectory == [[0.0, 0.0, 0.0]]
