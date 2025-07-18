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

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import TYPE_CHECKING, Optional, TypedDict

import numpy as np
from dimos_lcm.foxglove_msgs import ArrowPrimitive, Color, LinePrimitive
from dimos_lcm.geometry_msgs import Point, Vector3

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Twist import Twist


class ArrowConfig(TypedDict, total=False):
    shaft_diameter: float
    head_diameter: float
    head_length_ratio: float
    head_length: Optional[float]
    color: tuple[float, float, float, float]


default_config: ArrowConfig = {
    "shaft_diameter": 0.02,
    "head_diameter": 2.0,
    "head_length_ratio": 1.0,
    "head_length": None,
    "color": (1.0, 0.0, 0.0, 1.0),
}


class Arrow(ArrowPrimitive):
    @classmethod
    def from_transform(
        cls,
        transform: "Transform",
        pose: "Pose | PoseStamped",
        arrow_config: ArrowConfig = {},
    ):
        config = {**default_config, **arrow_config}

        # Apply transform to pose to get end position
        transformed_pose = pose @ transform

        # Calculate arrow vector using Vector3 operations
        arrow_vec = transformed_pose.position - pose.position
        length = arrow_vec.length()

        # Calculate head length
        head_length = config["head_length"]
        if head_length is None:
            head_length = config["head_diameter"] * config["head_length_ratio"]

        # Create arrow geometry
        arrow = cls()

        # Set the pose (start position and orientation)
        arrow.pose = pose

        # Set arrow properties using the actual ArrowPrimitive fields
        arrow.shaft_length = length
        arrow.shaft_diameter = config["shaft_diameter"]
        arrow.head_length = head_length
        arrow.head_diameter = config["head_diameter"]

        # Set color
        arrow.color = Color(
            r=config["color"][0], g=config["color"][1], b=config["color"][2], a=config["color"][3]
        )

        return arrow

    def lcm_encode(self) -> bytes:
        """Encode Arrow to LCM binary format."""
        return self.encode()

    @classmethod
    def lcm_decode(cls, data: bytes):
        """Decode Arrow from LCM binary format.

        Note: This returns an Arrow instance, not ArrowPrimitive.
        """
        # First decode as ArrowPrimitive
        arrow_primitive = ArrowPrimitive.decode(data)

        # Create a new Arrow instance and copy all fields
        arrow = cls()
        arrow.pose = arrow_primitive.pose
        arrow.shaft_length = arrow_primitive.shaft_length
        arrow.shaft_diameter = arrow_primitive.shaft_diameter
        arrow.head_length = arrow_primitive.head_length
        arrow.head_diameter = arrow_primitive.head_diameter
        arrow.color = arrow_primitive.color

        return arrow
