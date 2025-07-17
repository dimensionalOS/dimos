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

import numpy as np
from typing import Optional, TypedDict

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
    arrow = Arrow.from_transform(pose, transform)

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

    arrow = Arrow.from_transform(pose, transform, config)

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

    arrow = Arrow.from_transform(pose, transform)

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
    arrow1 = Arrow.from_transform(pose, transform)
    expected_head_length = 2.0 * 1.0  # head_diameter * head_length_ratio
    assert arrow1.head_length == expected_head_length

    # Test with explicit head length
    config = {"head_length": 0.5}
    arrow2 = Arrow.from_transform(pose, transform, config)
    assert arrow2.head_length == 0.5

    # Test with custom ratio
    config = {"head_length_ratio": 2.0}
    arrow3 = Arrow.from_transform(pose, transform, config)
    expected_head_length = 2.0 * 2.0  # head_diameter * custom_ratio
    assert arrow3.head_length == expected_head_length


def test_arrow_3d_transform():
    """Test arrow with 3D translation vector."""
    pose = Pose(1.0, 1.0, 1.0)
    transform = Transform(translation=Vector3(2.0, 3.0, 6.0))  # magnitude = 7.0

    arrow = Arrow.from_transform(pose, transform)

    expected_length = 7.0  # sqrt(2^2 + 3^2 + 6^2)
    assert np.isclose(arrow.shaft_length, expected_length, atol=1e-10)

    # Verify the arrow starts at the original pose
    assert arrow.pose.position.x == 1.0
    assert arrow.pose.position.y == 1.0
    assert arrow.pose.position.z == 1.0
