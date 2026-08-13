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

"""The published mount tree composes back to the measured rig geometry.

mount_transforms() inverts two of the four FRAMES edges to root the tree at
mid360_link, so the geometry a consumer reads back is not the geometry written
in FRAMES. These pin the composed result, which is what nav actually uses.
"""

import math

from dimos.protocol.tf.tf import MultiTBuffer
from dimos.robot.unitree.go2.go2_mid360_static_transforms import (
    MID360_PITCH_DOWN,
    mount_transforms,
)

# base_link -> mid360_link, summed down the FRAMES chain.
MOUNT_X = 0.32715 - 0.032
MOUNT_Z = 0.04297 + 0.12


def _buffer() -> MultiTBuffer:
    buffer = MultiTBuffer()
    buffer.receive_transform(*mount_transforms())
    return buffer


def test_mount_height_survives_the_inversion() -> None:
    """The lidar sits MOUNT_Z above base_link, the offset every ground projection uses."""
    leg = _buffer().get("mid360_link", "base_link")
    assert leg is not None
    base_to_sensor = -leg
    assert abs(base_to_sensor.translation.z - MOUNT_Z) < 1e-6
    assert abs(base_to_sensor.translation.x - MOUNT_X) < 1e-6


def test_mount_pitch_survives_the_inversion() -> None:
    """A sign flip here steers the follower off-heading rather than failing loudly."""
    leg = _buffer().get("mid360_link", "base_link")
    assert leg is not None
    pitch = (-leg).rotation.euler.y
    assert abs(pitch - MID360_PITCH_DOWN) < 1e-6
    assert abs(math.degrees(pitch) - 60.0) < 1e-6


def test_camera_optical_hangs_off_base_link() -> None:
    """The tree is rooted at mid360_link, so the camera edge is reachable by composition."""
    optical = _buffer().get("base_link", "camera_optical")
    assert optical is not None
    assert abs(optical.translation.x - 0.32715) < 1e-6
    assert abs(optical.translation.z - 0.04297) < 1e-6
