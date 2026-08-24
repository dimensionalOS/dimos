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

"""The published mount tree composes back to the g1.urdf geometry."""

import math

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.robot.unitree.g1.g1_tf_publisher import (
    D435_PITCH,
    MID360_PITCH,
    base_to_torso,
    mount_transforms,
    torso_to_mid360,
)

# g1.urdf pelvis -> torso_link rest offsets.
PELVIS_TORSO_X = -0.0039635
PELVIS_TORSO_Z = 0.044
# base_link -> mid360_link, summed down the rest-pose chain.
MOUNT_X = PELVIS_TORSO_X + 0.0002835
MOUNT_Z = PELVIS_TORSO_Z + 0.41618
LIDAR_HEIGHT = 1.2


def _buffer(
    waist_yaw: float = 0.0,
    waist_roll: float = 0.0,
    waist_pitch: float = 0.0,
    live: Transform | None = None,
) -> MultiTBuffer:
    buffer = MultiTBuffer()
    buffer.receive_transform(*mount_transforms(waist_yaw, waist_roll, waist_pitch))
    if live is not None:
        buffer.receive_transform(live)
    return buffer


def test_rest_pose_offsets_match_urdf() -> None:
    """The lidar sits MOUNT_Z above base_link, the offset every ground projection uses."""
    leg = _buffer().get("mid360_link", "base_link")
    assert leg is not None
    base_to_sensor = -leg
    assert abs(base_to_sensor.translation.z - MOUNT_Z) < 1e-6
    assert abs(base_to_sensor.translation.x - MOUNT_X) < 1e-6


def test_mid360_frame_is_upside_down() -> None:
    """The inverted mount points the sensor's z axis at the floor in the base frame."""
    leg = _buffer().get("base_link", "mid360_link")
    assert leg is not None
    z_axis = leg.rotation.rotate_vector(Vector3(0.0, 0.0, 1.0))
    assert z_axis.z < -0.99


def test_flipped_level_sensor_yields_level_base() -> None:
    """A standing robot reports a flipped sensor pose. base_link must come out level, below it."""
    live = Transform(
        translation=Vector3(0.0, 0.0, LIDAR_HEIGHT),
        rotation=torso_to_mid360().rotation,
        frame_id="world",
        child_frame_id="mid360_link",
    )
    base = _buffer(live=live).get("world", "base_link")
    assert base is not None
    euler = base.rotation.euler
    assert abs(euler.x) < 1e-6
    assert abs(euler.y) < 1e-6
    assert abs(euler.z) < 1e-6
    assert math.isclose(base.translation.z, LIDAR_HEIGHT - MOUNT_Z, abs_tol=1e-6)


def test_waist_yaw_rotates_base_link_against_the_torso() -> None:
    """A twisted waist must show up as opposite yaw on base_link, not be baked away."""
    yaw = math.pi / 4
    leg = _buffer(waist_yaw=yaw).get("torso_link", "base_link")
    assert leg is not None
    assert abs(leg.rotation.euler.z - (-yaw)) < 1e-6


def test_waist_pitch_rotates_base_link_against_the_torso() -> None:
    pitch = 0.3
    leg = _buffer(waist_pitch=pitch).get("torso_link", "base_link")
    assert leg is not None
    assert abs(leg.rotation.euler.y - (-pitch)) < 1e-6


def test_rest_pose_base_to_torso_matches_urdf_offsets() -> None:
    rest = base_to_torso(0.0, 0.0, 0.0)
    assert abs(rest.translation.x - PELVIS_TORSO_X) < 1e-6
    assert abs(rest.translation.z - PELVIS_TORSO_Z) < 1e-6
    assert abs(rest.rotation.euler.x) < 1e-6
    assert abs(rest.rotation.euler.y) < 1e-6
    assert abs(rest.rotation.euler.z) < 1e-6


def test_d435_hangs_off_base_link() -> None:
    """The tree is rooted at mid360_link, so the camera edge is reachable by composition."""
    camera = _buffer().get("base_link", "d435_link")
    assert camera is not None
    assert abs(camera.translation.x - (PELVIS_TORSO_X + 0.0576235)) < 1e-6
    assert abs(camera.translation.z - (PELVIS_TORSO_Z + 0.42987)) < 1e-6
    assert abs(camera.rotation.euler.y - D435_PITCH) < 1e-6


def test_pelvis_height_matches_config_note() -> None:
    """mid360 1.2m above ground implies the 0.74m nominal standing pelvis height."""
    assert math.isclose(LIDAR_HEIGHT - MOUNT_Z, 0.74, abs_tol=0.005)


def test_mid360_pitch_constant_matches_urdf() -> None:
    assert math.isclose(MID360_PITCH, 0.04014257279586953)
