# Copyright 2025 Dimensional Inc.
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

from typing import Optional, TypedDict

import numpy as np

from dimos.msgs.foxglove_msgs.Arrow import Arrow
from dimos.msgs.geometry_msgs import Pose, PoseStamped, Quaternion, Transform, Vector3


class ArrowConfigDict(TypedDict, total=False):
    shaft_diameter: float
    head_diameter: float
    head_length_ratio: float
    head_length: Optional[float]
    color: tuple[float, float, float, float]


def test_arrow_from_transform_basic():
    """Test basic arrow creation from pose and transform."""
    # Create a pose at origin
    pose = Pose(1.0, 2.0, 3.0)

    # Create a transform that moves 2 units in x direction
    transform = Transform(translation=Vector3(2.0, 0.0, 0.0))

    # Create arrow
    arrow = Arrow.from_transform(transform, pose)

    # Check that arrow pose matches input pose
    assert arrow.pose.position.x == 1.0
    assert arrow.pose.position.y == 2.0
    assert arrow.pose.position.z == 3.0

    # Check that shaft length matches the transform magnitude
    expected_length = 2.0  # magnitude of Vector3(2.0, 0.0, 0.0)
    assert np.isclose(arrow.shaft_length, expected_length, atol=1e-10)

    # Check default configuration values
    assert arrow.shaft_diameter == 0.02
    assert arrow.head_diameter == 2.0
    assert arrow.color.r == 1.0
    assert arrow.color.g == 0.0
    assert arrow.color.b == 0.0
    assert arrow.color.a == 1.0


def test_arrow_from_transform_with_config():
    """Test arrow creation with custom configuration."""
    pose = Pose(0.0, 0.0, 0.0)
    transform = Transform(translation=Vector3(1.0, 1.0, 0.0))

    # Custom configuration
    config = {
        "shaft_diameter": 0.05,
        "head_diameter": 1.5,
        "color": (0.0, 1.0, 0.0, 0.8),  # Green with transparency
    }

    arrow = Arrow.from_transform(transform, pose, config)

    # Check custom values were applied
    assert arrow.shaft_diameter == 0.05
    assert arrow.head_diameter == 1.5
    assert arrow.color.r == 0.0
    assert arrow.color.g == 1.0
    assert arrow.color.b == 0.0
    assert arrow.color.a == 0.8

    # Check shaft length matches transform magnitude
    expected_length = np.sqrt(2.0)  # magnitude of Vector3(1.0, 1.0, 0.0)
    assert np.isclose(arrow.shaft_length, expected_length, atol=1e-10)


def test_arrow_from_transform_zero_length():
    """Test arrow creation with zero-length transform."""
    pose = Pose(5.0, 5.0, 5.0)

    # Zero transform (no movement) - identity transform
    transform = Transform()

    arrow = Arrow.from_transform(
        transform,
        pose,
    )

    # Arrow should have zero length
    assert arrow.shaft_length == 0.0

    # Pose should be preserved
    assert arrow.pose.position.x == 5.0
    assert arrow.pose.position.y == 5.0
    assert arrow.pose.position.z == 5.0


def test_arrow_head_length_calculation():
    """Test head length calculation with and without explicit setting."""
    pose = Pose()
    transform = Transform(translation=Vector3(1.0, 0.0, 0.0))

    # Test with default head length (should be head_diameter * head_length_ratio)
    arrow1 = Arrow.from_transform(
        transform,
        pose,
    )
    expected_head_length = 2.0 * 1.0  # head_diameter * head_length_ratio
    assert arrow1.head_length == expected_head_length

    # Test with explicit head length
    config = {"head_length": 0.5}
    arrow2 = Arrow.from_transform(transform, pose, config)
    assert arrow2.head_length == 0.5

    # Test with custom ratio
    config = {"head_length_ratio": 2.0}
    arrow3 = Arrow.from_transform(transform, pose, config)
    expected_head_length = 2.0 * 2.0  # head_diameter * custom_ratio
    assert arrow3.head_length == expected_head_length


def test_arrow_3d_transform():
    """Test arrow with 3D translation vector."""
    pose = Pose(1.0, 1.0, 1.0)
    transform = Transform(translation=Vector3(2.0, 3.0, 6.0))  # magnitude = 7.0

    arrow = Arrow.from_transform(transform, pose)

    expected_length = 7.0  # sqrt(2^2 + 3^2 + 6^2)
    assert np.isclose(arrow.shaft_length, expected_length, atol=1e-10)

    # Verify the arrow starts at the original pose
    assert arrow.pose.position.x == 1.0
    assert arrow.pose.position.y == 1.0
    assert arrow.pose.position.z == 1.0


def test_arrow_lcm_encode_decode():
    """Test LCM encoding and decoding of Arrow."""
    # Create an arrow using from_transform
    pose = Pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.707107, 0.707107)  # 90 deg around Z
    transform = Transform(translation=Vector3(2.0, 1.0, 0.5))
    config = {
        "shaft_diameter": 0.03,
        "head_diameter": 1.8,
        "head_length": 0.4,
        "color": (0.2, 0.8, 0.3, 0.9),
    }

    arrow_source = Arrow.from_transform(transform, pose, config)

    # Encode to binary
    binary_msg = arrow_source.lcm_encode()

    # Decode from binary
    arrow_dest = Arrow.lcm_decode(binary_msg)

    # Verify it's a new instance of Arrow (not ArrowPrimitive)
    assert isinstance(arrow_dest, Arrow)
    assert arrow_dest is not arrow_source

    # Verify all fields match
    assert np.isclose(arrow_dest.pose.position.x, arrow_source.pose.position.x, atol=1e-10)
    assert np.isclose(arrow_dest.pose.position.y, arrow_source.pose.position.y, atol=1e-10)
    assert np.isclose(arrow_dest.pose.position.z, arrow_source.pose.position.z, atol=1e-10)
    assert np.isclose(arrow_dest.pose.orientation.x, arrow_source.pose.orientation.x, atol=1e-10)
    assert np.isclose(arrow_dest.pose.orientation.y, arrow_source.pose.orientation.y, atol=1e-10)
    assert np.isclose(arrow_dest.pose.orientation.z, arrow_source.pose.orientation.z, atol=1e-10)
    assert np.isclose(arrow_dest.pose.orientation.w, arrow_source.pose.orientation.w, atol=1e-10)

    assert np.isclose(arrow_dest.shaft_length, arrow_source.shaft_length, atol=1e-10)
    assert np.isclose(arrow_dest.shaft_diameter, arrow_source.shaft_diameter, atol=1e-10)
    assert np.isclose(arrow_dest.head_length, arrow_source.head_length, atol=1e-10)
    assert np.isclose(arrow_dest.head_diameter, arrow_source.head_diameter, atol=1e-10)

    assert np.isclose(arrow_dest.color.r, arrow_source.color.r, atol=1e-10)
    assert np.isclose(arrow_dest.color.g, arrow_source.color.g, atol=1e-10)
    assert np.isclose(arrow_dest.color.b, arrow_source.color.b, atol=1e-10)
    assert np.isclose(arrow_dest.color.a, arrow_source.color.a, atol=1e-10)


def test_arrow_from_transform_with_posestamped():
    """Test arrow creation from PoseStamped and transform."""
    # Create a PoseStamped
    pose_stamped = PoseStamped(
        ts=1234567890.123,
        frame_id="base_link",
        position=Vector3(2.0, 3.0, 4.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )

    # Create a transform
    transform = Transform(
        translation=Vector3(3.0, 0.0, 0.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
    )

    # Create arrow
    arrow = Arrow.from_transform(transform, pose_stamped)

    # Check that arrow pose matches input pose_stamped
    assert arrow.pose.position.x == 2.0
    assert arrow.pose.position.y == 3.0
    assert arrow.pose.position.z == 4.0

    # Check that shaft length matches the transform magnitude
    expected_length = 3.0  # magnitude of Vector3(3.0, 0.0, 0.0)
    assert np.isclose(arrow.shaft_length, expected_length, atol=1e-10)

    # Verify the arrow properties are set correctly
    assert arrow.shaft_diameter == 0.02
    assert arrow.head_diameter == 2.0
    assert arrow.color.r == 1.0
    assert arrow.color.g == 0.0
    assert arrow.color.b == 0.0
    assert arrow.color.a == 1.0


def test_arrow_complex_transform_with_rotation():
    """Test arrow with transform that includes both translation and rotation."""
    # Create a pose at origin facing forward
    pose = Pose(0.0, 0.0, 0.0)

    # Create a transform with translation and 45 degree rotation around Z
    angle = np.pi / 4  # 45 degrees
    transform = Transform(
        translation=Vector3(2.0, 2.0, 0.0),
        rotation=Quaternion(0.0, 0.0, np.sin(angle / 2), np.cos(angle / 2)),
    )

    # Create arrow
    arrow = Arrow.from_transform(transform, pose)

    # The arrow vector should be from pose position to transformed position
    # Since pose is at origin, the transformed position is just the translation
    expected_length = np.sqrt(8.0)  # magnitude of Vector3(2.0, 2.0, 0.0)
    assert np.isclose(arrow.shaft_length, expected_length, atol=1e-10)

    # Arrow should start at the original pose
    assert arrow.pose.position.x == 0.0
    assert arrow.pose.position.y == 0.0
    assert arrow.pose.position.z == 0.0
