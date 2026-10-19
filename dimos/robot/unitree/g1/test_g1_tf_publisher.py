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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header

from dimos.msgs.geometry import inverse_transform, quaternion_euler, transform_matrix
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
    live: TransformStamped | None = None,
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
    base_to_sensor = inverse_transform(leg)
    assert abs(base_to_sensor.transform.translation.z - MOUNT_Z) < 1e-6
    assert abs(base_to_sensor.transform.translation.x - MOUNT_X) < 1e-6


def test_mid360_frame_is_upside_down() -> None:
    """The inverted mount points the sensor's z axis at the floor in the base frame."""
    leg = _buffer().get("base_link", "mid360_link")
    assert leg is not None
    z_axis = transform_matrix(leg.transform)[:3, 2]
    assert z_axis[2] < -0.99


def test_flipped_level_sensor_yields_level_base() -> None:
    """A standing robot reports a flipped sensor pose. base_link must come out level, below it."""
    live = TransformStamped(
        header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
        child_frame_id="mid360_link",
        transform=Transform(
            translation=Vector3(z=LIDAR_HEIGHT, x=0.0, y=0.0),
            rotation=torso_to_mid360().transform.rotation,
        ),
    )
    base = _buffer(live=live).get("world", "base_link")
    assert base is not None
    euler = quaternion_euler(base.transform.rotation)
    assert abs(euler[0]) < 1e-6
    assert abs(euler[1]) < 1e-6
    assert abs(euler[2]) < 1e-6
    assert math.isclose(base.transform.translation.z, LIDAR_HEIGHT - MOUNT_Z, abs_tol=1e-6)


def test_waist_yaw_rotates_base_link_against_the_torso() -> None:
    """A twisted waist must show up as opposite yaw on base_link, not be baked away."""
    yaw = math.pi / 4
    leg = _buffer(waist_yaw=yaw).get("torso_link", "base_link")
    assert leg is not None
    assert abs(quaternion_euler(leg.transform.rotation)[2] - (-yaw)) < 1e-6


def test_waist_pitch_rotates_base_link_against_the_torso() -> None:
    pitch = 0.3
    leg = _buffer(waist_pitch=pitch).get("torso_link", "base_link")
    assert leg is not None
    assert abs(quaternion_euler(leg.transform.rotation)[1] - (-pitch)) < 1e-6


def test_rest_pose_base_to_torso_matches_urdf_offsets() -> None:
    rest = base_to_torso(0.0, 0.0, 0.0)
    assert abs(rest.transform.translation.x - PELVIS_TORSO_X) < 1e-6
    assert abs(rest.transform.translation.z - PELVIS_TORSO_Z) < 1e-6
    assert abs(quaternion_euler(rest.transform.rotation)[0]) < 1e-6
    assert abs(quaternion_euler(rest.transform.rotation)[1]) < 1e-6
    assert abs(quaternion_euler(rest.transform.rotation)[2]) < 1e-6


def test_d435_hangs_off_base_link() -> None:
    """The tree is rooted at mid360_link, so the camera edge is reachable by composition."""
    camera = _buffer().get("base_link", "d435_link")
    assert camera is not None
    assert abs(camera.transform.translation.x - (PELVIS_TORSO_X + 0.0576235)) < 1e-6
    assert abs(camera.transform.translation.z - (PELVIS_TORSO_Z + 0.42987)) < 1e-6
    assert abs(quaternion_euler(camera.transform.rotation)[1] - D435_PITCH) < 1e-6


def test_pelvis_height_matches_config_note() -> None:
    """mid360 1.2m above ground implies the 0.74m nominal standing pelvis height."""
    assert math.isclose(LIDAR_HEIGHT - MOUNT_Z, 0.74, abs_tol=0.005)


def test_mid360_pitch_constant_matches_urdf() -> None:
    assert math.isclose(MID360_PITCH, 0.04014257279586953)
