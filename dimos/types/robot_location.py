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

"""
RobotLocation type definition for storing and managing robot location data.
"""

from dataclasses import dataclass, field
import time
from typing import Any
import uuid


@dataclass
class RobotLocation:
    """
    Represents a named location in the robot's spatial memory.

    This class stores the position, rotation, and descriptive metadata for
    locations that the robot can remember and navigate to.

    Attributes:
        name: Human-readable name of the location (e.g., "kitchen", "office")
        position: 3D position coordinates (x, y, z)
        rotation: 3D rotation angles in radians (roll, pitch, yaw)
        frame_id: ID of the associated video frame if available
        timestamp: Time when the location was recorded
        location_id: Unique identifier for this location
        metadata: Additional metadata for the location
    """

    name: str
    position: tuple[float, float, float]
    rotation: tuple[float, float, float]
    frame_id: str | None = None
    timestamp: float = field(default_factory=time.time)
    location_id: str = field(default_factory=lambda: f"loc_{uuid.uuid4().hex[:8]}")
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate and normalize the position and rotation tuples."""
        # Ensure position is a tuple of 3 floats
        if len(self.position) == 2:
            self.position = (self.position[0], self.position[1], 0.0)
        else:
            self.position = tuple(float(x) for x in self.position)  # type: ignore[assignment]

        # Ensure rotation is a tuple of 3 floats
        if len(self.rotation) == 1:
            self.rotation = (0.0, 0.0, self.rotation[0])
        else:
            self.rotation = tuple(float(x) for x in self.rotation)  # type: ignore[assignment]

    def to_vector_metadata(self) -> dict[str, Any]:
        """
        Convert the location to metadata format for storing in a vector database.

        Returns:
            Dictionary with metadata fields compatible with vector DB storage
        """
        metadata = {
            "pos_x": float(self.position[0]),
            "pos_y": float(self.position[1]),
            "pos_z": float(self.position[2]),
            "rot_x": float(self.rotation[0]),
            "rot_y": float(self.rotation[1]),
            "rot_z": float(self.rotation[2]),
            "timestamp": self.timestamp,
            "location_id": self.location_id,
            "location_name": self.name,
            "description": self.name,  # Makes it searchable by text
        }

        # Only add frame_id if it's not None
        if self.frame_id is not None:
            metadata["frame_id"] = self.frame_id

        return metadata

    @classmethod
    def from_vector_metadata(cls, metadata: dict[str, Any]) -> "RobotLocation":
        """
        Create a RobotLocation object from vector database metadata.

        Args:
            metadata: Dictionary with metadata from vector database

        Returns:
            RobotLocation object
        """
        return cls(
            name=metadata.get("location_name", "unknown"),
            position=(
                metadata.get("pos_x", 0.0),
                metadata.get("pos_y", 0.0),
                metadata.get("pos_z", 0.0),
            ),
            rotation=(
                metadata.get("rot_x", 0.0),
                metadata.get("rot_y", 0.0),
                metadata.get("rot_z", 0.0),
            ),
            frame_id=metadata.get("frame_id"),
            timestamp=metadata.get("timestamp", time.time()),
            location_id=metadata.get("location_id", f"loc_{uuid.uuid4().hex[:8]}"),
            metadata={
                k: v
                for k, v in metadata.items()
                if k
                not in [
                    "pos_x",
                    "pos_y",
                    "pos_z",
                    "rot_x",
                    "rot_y",
                    "rot_z",
                    "timestamp",
                    "location_id",
                    "frame_id",
                    "location_name",
                    "description",
                ]
            },
        )

    def __str__(self) -> str:
        return f"[RobotPosition name:{self.name} pos:{self.position} rot:{self.rotation})]"
