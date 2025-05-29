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

from typing import Optional, List, Tuple
from pydantic import Field

from dimos.skills.skills import AbstractRobotSkill
from dimos.types.manipulation_constraint import RotationConstraint
from dimos.utils.logging_config import setup_logger
from dimos.types.vector import Vector

# Initialize logger
logger = setup_logger("dimos.skills.rotation_constraint_skill")


class RotationConstraintSkill(AbstractRobotSkill):
    """
    Skill for generating rotation constraints for robot manipulation.
    """

    # Constraint parameters
    lock_roll: bool = Field(False, description="Whether to lock rotation around the x-axis")
    lock_pitch: bool = Field(False, description="Whether to lock rotation around the y-axis")
    lock_yaw: bool = Field(False, description="Whether to lock rotation around the z-axis")

    # Simple angle values for rotation (in radians)
    start_angle: Optional[float] = Field(None, description="Starting angle in radians")
    end_angle: Optional[float] = Field(None, description="Ending angle in radians")

    # Pivot points as (x,y) tuples
    pivot_point: Optional[Tuple[float, float]] = Field(
        None, description="Pivot point (x,y) for rotation"
    )

    # TODO: Secondary pivot point if needed (for double-point locked rotation)
    secondary_pivot_point: Optional[Tuple[float, float]] = Field(
        None, description="Secondary pivot point (x,y) for double-pivot rotation"
    )

    # Description
    description: str = Field("", description="Description of the rotation constraint")

    def __call__(self) -> RotationConstraint:
        """
        Generate a rotation constraint based on the parameters.

        Returns:
            RotationConstraint: The generated constraint
        """
        # Figure out which axis we're rotating around based on what's not locked
        rotation_axis = None
        if not self.lock_roll:
            rotation_axis = "roll"
        elif not self.lock_pitch:
            rotation_axis = "pitch"
        elif not self.lock_yaw:
            rotation_axis = "yaw"

        # start angle vector with angle applied about correct locked axis
        start_angle_vector = None
        if self.start_angle is not None:
            start_angle_vector = Vector(
                self.start_angle if rotation_axis == "roll" else 0.0,
                self.start_angle if rotation_axis == "pitch" else 0.0,
                self.start_angle if rotation_axis == "yaw" else 0.0,
            )

        # end angle vector with angle applied about correct locked axis
        end_angle_vector = None
        if self.end_angle is not None:
            end_angle_vector = Vector(
                self.end_angle if rotation_axis == "roll" else 0.0,
                self.end_angle if rotation_axis == "pitch" else 0.0,
                self.end_angle if rotation_axis == "yaw" else 0.0,
            )

        # Create pivot point vector if provided (convert 2D point to 3D vector with z=0)
        pivot_point_vector = None
        if self.pivot_point:
            pivot_point_vector = Vector(self.pivot_point[0], self.pivot_point[1], 0.0)

        # Create secondary pivot point vector if provided
        secondary_pivot_vector = None
        if self.secondary_pivot_point:
            secondary_pivot_vector = Vector(
                self.secondary_pivot_point[0], self.secondary_pivot_point[1], 0.0
            )

        # Create and return the constraint
        constraint = RotationConstraint(
            lock_roll=self.lock_roll,
            lock_pitch=self.lock_pitch,
            lock_yaw=self.lock_yaw,
            start_angle=start_angle_vector,
            end_angle=end_angle_vector,
            pivot_point=pivot_point_vector,
            secondary_pivot_point=secondary_pivot_vector,
            description=self.description,
        )

        # Log the constraint creation
        logger.info(f"Generated rotation constraint: {self.description}")

        return constraint
