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

"""Viewpoints around an object for a wrist camera. World frame, Z up, metres."""

from __future__ import annotations

import math

import numpy as np
import pytest

from dimos.manipulation import look_around

# A camera 17 cm behind the tool tip, looking along its Z axis.
CAMERA_FROM_TIP = np.array(
    [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, -0.17], [0.0, 0.0, 0.0, 1.0]]
)
CENTER = (0.5, 0.1, 0.8)


@pytest.mark.parametrize("bearing", [0.0, 1.0, math.pi, -2.0])
def test_the_camera_looks_at_the_centre_from_the_ring(bearing: float) -> None:
    tip = look_around.viewpoint(CENTER, bearing, 0.35, math.radians(50.0), CAMERA_FROM_TIP)

    camera = tip @ CAMERA_FROM_TIP
    eye, forward, right = camera[:3, 3], camera[:3, 2], camera[:3, 0]
    to_center = np.asarray(CENTER) - eye
    assert np.linalg.norm(to_center) == pytest.approx(0.35)
    assert forward == pytest.approx(to_center / 0.35)
    # 50 degrees above the centre, on the given bearing, image rows level.
    assert math.degrees(math.asin(-forward[2])) == pytest.approx(50.0)
    assert look_around.bearing_of(CENTER, eye) == pytest.approx(
        math.remainder(bearing, 2 * math.pi)
    )
    assert right[2] == pytest.approx(0.0)
    assert np.linalg.det(camera[:3, :3]) == pytest.approx(1.0)


def test_each_next_view_is_the_farthest_from_those_seen() -> None:
    bearings = look_around.ring(0.0, 8)[1:]
    seen = [0.0]

    first = look_around.farthest(bearings, seen)
    second = look_around.farthest([b for b in bearings if b != first], [*seen, first])

    assert first == pytest.approx(math.pi)
    assert abs(math.remainder(second, 2 * math.pi)) == pytest.approx(math.pi / 2)
