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

from enum import Enum
from typing import Dict, List, Optional, Any, Union, TypedDict, Tuple
from dataclasses import dataclass, field, fields
from abc import ABC, abstractmethod
import uuid
import numpy as np
import time
from dimos.types.vector import Vector


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
    """Constraint parameters for translational movement."""

    lock_x: bool = False
    lock_y: bool = False
    lock_z: bool = False
    reference_point: Optional[Vector] = None
    bounds_min: Optional[Vector] = None  # For bounded translation
    bounds_max: Optional[Vector] = None  # For bounded translation
    target_point: Optional[Vector] = None  # For relative positioning
    description: str = ""


@dataclass
class RotationConstraint(AbstractConstraint):
    """Constraint parameters for rotational movement."""

    lock_roll: bool = False
    lock_pitch: bool = False
    lock_yaw: bool = False
    start_angle: Optional[Vector] = None  # Roll, pitch, yaw start angles
    end_angle: Optional[Vector] = None  # Roll, pitch, yaw end angles
    pivot_point: Optional[Vector] = None  # Point of rotation
    secondary_pivot_point: Optional[Vector] = None  # For double point locked rotation
    description: str = ""


@dataclass
class ForceConstraint(AbstractConstraint):
    """Constraint parameters for force application."""

    max_force: float = 0.0  # Maximum force in newtons
    min_force: float = 0.0  # Minimum force in newtons
    force_direction: Optional[Vector] = None  # Direction of force application
    description: str = ""


class ObjectData(TypedDict, total=False):
    """Data about an object in the manipulation scene."""

    object_id: int  # Unique ID for the object
    bbox: List[float]  # Bounding box [x1, y1, x2, y2]
    depth: float  # Depth in meters from Metric3d
    confidence: float  # Detection confidence
    class_id: int  # Class ID from the detector
    label: str  # Semantic label (e.g., 'cup', 'table')
    movement_tolerance: float  # 0-1 value indicating how movable the object is
    segmentation_mask: np.ndarray  # Binary mask of the object's pixels
    position: Dict[str, float]  # 3D position {x, y, z}
    rotation: Dict[str, float]  # 3D rotation {roll, pitch, yaw}
    size: Dict[str, float]  # Object dimensions {width, height}


class ManipulationMetadata(TypedDict, total=False):
    """Typed metadata for manipulation constraints."""

    timestamp: float
    objects: Dict[str, ObjectData]


@dataclass
class ManipulationTaskConstraint:
    """Set of constraints for a specific manipulation action."""

    constraints: List[AbstractConstraint] = field(default_factory=list)

    def add_constraint(self, constraint: AbstractConstraint):
        """Add a constraint to this set."""
        if constraint not in self.constraints:
            self.constraints.append(constraint)

    def get_constraints(self) -> List[AbstractConstraint]:
        """Get all constraints in this set."""
        return self.constraints


@dataclass
class ManipulationTask:
    """Complete definition of a manipulation task."""

    description: str
    target_object: str  # Semantic label of target object
    target_point: Optional[Tuple[float, float]] = (
        None  # (X,Y) point in pixel-space of the point to manipulate on target object
    )
    metadata: ManipulationMetadata = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)
    task_id: str = ""
    result: Optional[Dict[str, Any]] = None  # Any result data from the task execution
    constraints: Union[List[AbstractConstraint], ManipulationTaskConstraint, AbstractConstraint] = (
        field(default_factory=list)
    )

    def add_constraint(self, constraint: AbstractConstraint):
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

    def get_constraints(self) -> List[AbstractConstraint]:
        """Get all constraints in this manipulation task."""
        # If constraints is a ManipulationTaskConstraint object
        if isinstance(self.constraints, ManipulationTaskConstraint):
            return self.constraints.get_constraints()

        # If constraints is a single AbstractConstraint, return as list
        if isinstance(self.constraints, AbstractConstraint):
            return [self.constraints]

        # If constraints is a list (including empty list), return it
        return self.constraints
