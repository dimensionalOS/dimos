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

from typing import Literal

from pydantic import Field

from dimos.skills.manipulation.abstract_manipulation_skill import AbstractManipulationSkill
from dimos.types.manipulation import RotationConstraint
from dimos.types.vector import Vector
from dimos.utils.logging_config import setup_logger

# Initialize logger
logger = setup_logger()


class RotationConstraintSkill(AbstractManipulationSkill):
    """
    Skill for generating rotation constraints for robot manipulation.

    This skill generates rotation constraints and adds them to the ManipulationInterface's
    agent_constraints list for tracking constraints created by the Agent.
    """

    # Rotation axis parameter
    rotation_axis: Literal["roll", "pitch", "yaw"] = Field(
        "roll",
        description="Axis to rotate around: 'roll' (x-axis), 'pitch' (y-axis), or 'yaw' (z-axis)",
    )

    # Simple angle values for rotation (in degrees)
    start_angle: float | None = Field(None, description="Starting angle in degrees")
    end_angle: float | None = Field(None, description="Ending angle in degrees")

    # Pivot points as (x,y) tuples
    pivot_point: tuple[float, float] | None = Field(
        None, description="Pivot point (x,y) for rotation"
    )

    # TODO: Secondary pivot point for more complex rotations
    secondary_pivot_point: tuple[float, float] | None = Field(
        None, description="Secondary pivot point (x,y) for double-pivot rotation"
    )

    def __call__(self) -> RotationConstraint:
        """
        Generate a rotation constraint based on the parameters.

        This implementation supports rotation around a single axis (roll, pitch, or yaw).

        Returns:
            RotationConstraint: The generated constraint
        """
        # rotation_axis is guaranteed to be one of "roll", "pitch", or "yaw" due to Literal type constraint

        # Create angle vectors more efficiently
        start_angle_vector = None
        if self.start_angle is not None:
            # Build rotation vector on correct axis
            values = [0.0, 0.0, 0.0]
            axis_index = {"roll": 0, "pitch": 1, "yaw": 2}[self.rotation_axis]
            values[axis_index] = self.start_angle
            start_angle_vector = Vector(*values)  # type: ignore[arg-type]

        end_angle_vector = None
        if self.end_angle is not None:
            values = [0.0, 0.0, 0.0]
            axis_index = {"roll": 0, "pitch": 1, "yaw": 2}[self.rotation_axis]
            values[axis_index] = self.end_angle
            end_angle_vector = Vector(*values)  # type: ignore[arg-type]

        # Create pivot point vector if provided (convert 2D point to 3D vector with z=0)
        pivot_point_vector = None
        if self.pivot_point:
            pivot_point_vector = Vector(self.pivot_point[0], self.pivot_point[1], 0.0)  # type: ignore[arg-type]

        # Create secondary pivot point vector if provided
        secondary_pivot_vector = None
        if self.secondary_pivot_point:
            secondary_pivot_vector = Vector(
                self.secondary_pivot_point[0],  # type: ignore[arg-type]
                self.secondary_pivot_point[1],  # type: ignore[arg-type]
                0.0,  # type: ignore[arg-type]
            )

        constraint = RotationConstraint(
            rotation_axis=self.rotation_axis,
            start_angle=start_angle_vector,
            end_angle=end_angle_vector,
            pivot_point=pivot_point_vector,
            secondary_pivot_point=secondary_pivot_vector,
        )

        # Add constraint to manipulation interface
        self.manipulation_interface.add_constraint(constraint)  # type: ignore[union-attr]

        # Log the constraint creation
        logger.info(f"Generated rotation constraint around {self.rotation_axis} axis")

        return constraint
