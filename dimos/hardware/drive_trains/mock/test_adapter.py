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

import math

from dimos.hardware.drive_trains.mock.adapter import MockTwistBaseAdapter


def _drive(adapter, velocities, seconds, dt=0.01):
    for _ in range(round(seconds / dt)):
        adapter.write_velocities(velocities)


def test_integrated_odometry_follows_body_velocity_in_the_world_frame():
    """Facing +y, driving forward moves the base along world +y, and yaw wraps past pi."""
    adapter = MockTwistBaseAdapter(dof=3, integrate_odometry=True, step_dt=0.01)
    adapter.set_odometry([0.0, 0.0, math.pi / 2])

    _drive(adapter, [1.0, 0.0, 0.0], seconds=2.0)
    _drive(adapter, [0.0, 0.0, 1.0], seconds=2.0)

    x, y, yaw = adapter.read_odometry()
    assert abs(x) < 1e-6
    assert abs(y - 2.0) < 1e-6
    assert abs(yaw - (math.pi / 2 + 2.0 - 2 * math.pi)) < 1e-9
