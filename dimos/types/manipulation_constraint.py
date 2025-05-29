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
from typing import Optional, List, Tuple, Dict, Any, Union
from dataclasses import dataclass
from abc import ABC, abstractmethod
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


@dataclass
class ManipulationConstraint:
    """Complete set of constraints for a manipulation task."""

    name: str
    description: str
    constraints: List[AbstractConstraint] = None

    target_object: str = ""  # Semantic label of target object
    metadata: Dict[str, Any] = None

    def __post_init__(self):
        # Initialize metadata dictionary if None
        if self.metadata is None:
            self.metadata = {}

        # Initialize constraints list if None
        if self.constraints is None:
            self.constraints = []

    def add_constraint(self, constraint: AbstractConstraint):
        """Add a constraint to this manipulation task."""
        if self.constraints is None:
            self.constraints = []

        # Add the constraint to the list
        if constraint not in self.constraints:
            self.constraints.append(constraint)

    def get_constraints_by_type(self, constraint_type: ConstraintType) -> List[AbstractConstraint]:
        """Get all constraints of a specific type."""
        if self.constraints is None:
            return []

        return [c for c in self.constraints if c.get_constraint_type() == constraint_type]
