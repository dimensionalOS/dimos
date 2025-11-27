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

from abc import ABC
from dataclasses import dataclass, field
from enum import Enum
import time
from typing import TYPE_CHECKING, Any, Literal, TypedDict
import uuid

import numpy as np

from dimos.types.vector import Vector

if TYPE_CHECKING:
    import open3d as o3d  # type: ignore[import-untyped]


class ConstraintType(Enum):
    """Types of manipulation constraints."""

    TRANSLATION = "translation"
    ROTATION = "rotation"
    FORCE = "force"


@dataclass
class AbstractConstraint(ABC):
    """Base class for all manipulation constraints."""

    description: str = ""
    id: str = field(default_factory=lambda: str(uuid.uuid4())[:8])


@dataclass
class TranslationConstraint(AbstractConstraint):
    """Constraint parameters for translational movement along a single axis."""

    translation_axis: Literal["x", "y", "z"] = None  # type: ignore[assignment]  # Axis to translate along
    reference_point: Vector | None = None
    bounds_min: Vector | None = None  # For bounded translation
    bounds_max: Vector | None = None  # For bounded translation
    target_point: Vector | None = None  # For relative positioning


@dataclass
class RotationConstraint(AbstractConstraint):
    """Constraint parameters for rotational movement around a single axis."""

    rotation_axis: Literal["roll", "pitch", "yaw"] = None  # type: ignore[assignment]  # Axis to rotate around
    start_angle: Vector | None = None  # Angle values applied to the specified rotation axis
    end_angle: Vector | None = None  # Angle values applied to the specified rotation axis
    pivot_point: Vector | None = None  # Point of rotation
    secondary_pivot_point: Vector | None = None  # For double point rotations


@dataclass
class ForceConstraint(AbstractConstraint):
    """Constraint parameters for force application."""

    max_force: float = 0.0  # Maximum force in newtons
    min_force: float = 0.0  # Minimum force in newtons
    force_direction: Vector | None = None  # Direction of force application


class ObjectData(TypedDict, total=False):
    """Data about an object in the manipulation scene."""

    # Basic detection information
    object_id: int  # Unique ID for the object
    bbox: list[float]  # Bounding box [x1, y1, x2, y2]
    depth: float  # Depth in meters from Metric3d
    confidence: float  # Detection confidence
    class_id: int  # Class ID from the detector
    label: str  # Semantic label (e.g., 'cup', 'table')
    movement_tolerance: float  # (0.0 = immovable, 1.0 = freely movable)
    segmentation_mask: np.ndarray  # type: ignore[type-arg]  # Binary mask of the object's pixels

    # 3D pose and dimensions
    position: dict[str, float] | Vector  # 3D position {x, y, z} or Vector
    rotation: dict[str, float] | Vector  # 3D rotation {roll, pitch, yaw} or Vector
    size: dict[str, float]  # Object dimensions {width, height, depth}

    # Point cloud data
    point_cloud: "o3d.geometry.PointCloud"  # Open3D point cloud object
    point_cloud_numpy: np.ndarray  # type: ignore[type-arg]  # Nx6 array of XYZRGB points
    color: np.ndarray  # type: ignore[type-arg]  # RGB color for visualization [R, G, B]


class ManipulationMetadata(TypedDict, total=False):
    """Typed metadata for manipulation constraints."""

    timestamp: float
    objects: dict[str, ObjectData]


@dataclass
class ManipulationTaskConstraint:
    """Set of constraints for a specific manipulation action."""

    constraints: list[AbstractConstraint] = field(default_factory=list)

    def add_constraint(self, constraint: AbstractConstraint) -> None:
        """Add a constraint to this set."""
        if constraint not in self.constraints:
            self.constraints.append(constraint)

    def get_constraints(self) -> list[AbstractConstraint]:
        """Get all constraints in this set."""
        return self.constraints


@dataclass
class ManipulationTask:
    """Complete definition of a manipulation task."""

    description: str
    target_object: str  # Semantic label of target object
    target_point: tuple[float, float] | None = (
        None  # (X,Y) point in pixel-space of the point to manipulate on target object
    )
    metadata: ManipulationMetadata = field(default_factory=dict)  # type: ignore[assignment]
    timestamp: float = field(default_factory=time.time)
    task_id: str = ""
    result: dict[str, Any] | None = None  # Any result data from the task execution
    constraints: list[AbstractConstraint] | ManipulationTaskConstraint | AbstractConstraint = field(
        default_factory=list
    )

    def add_constraint(self, constraint: AbstractConstraint) -> None:
        """Add a constraint to this manipulation task."""
        # If constraints is a ManipulationTaskConstraint object
        if isinstance(self.constraints, ManipulationTaskConstraint):
            self.constraints.add_constraint(constraint)
            return

        # If constraints is a single AbstractConstraint, convert to list
        if isinstance(self.constraints, AbstractConstraint):
            self.constraints = [self.constraints, constraint]
            return

        # If constraints is a list, append to it
        # This will also handle empty lists (the default case)
        self.constraints.append(constraint)

    def get_constraints(self) -> list[AbstractConstraint]:
        """Get all constraints in this manipulation task."""
        # If constraints is a ManipulationTaskConstraint object
        if isinstance(self.constraints, ManipulationTaskConstraint):
            return self.constraints.get_constraints()

        # If constraints is a single AbstractConstraint, return as list
        if isinstance(self.constraints, AbstractConstraint):
            return [self.constraints]

        # If constraints is a list (including empty list), return it
        return self.constraints
