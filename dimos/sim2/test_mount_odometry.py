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

from __future__ import annotations

import math

import numpy as np

from dimos.sim2.mount_odometry import MountOdometry


def test_odometry_starts_at_the_body_s_place_on_the_mount_not_at_zero() -> None:
    body_on_mount = np.eye(4)
    body_on_mount[2, 3] = 0.4
    odometry = MountOdometry(body_on_mount)
    # The mount is at (3, 1, 0) turned a quarter about z; the body stands 0.4 above it.
    turn = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))

    start = odometry((3.0, 1.0, 0.4, *turn))
    # One metre along the mount's x is world +y.
    later = odometry((3.0, 2.0, 0.4, *turn))

    assert np.allclose([start.translation.x, start.translation.y, start.translation.z], [0, 0, 0.4])
    assert np.allclose([later.translation.x, later.translation.y, later.translation.z], [1, 0, 0.4])
    assert np.isclose(later.rotation.w, 1.0)
