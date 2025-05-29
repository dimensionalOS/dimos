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
from dimos.types.manipulation_constraint import TranslationConstraint, Vector
from dimos.utils.logging_config import setup_logger

# Initialize logger
logger = setup_logger("dimos.skills.translation_constraint_skill")


class TranslationConstraintSkill(AbstractRobotSkill):
    """
    Skill for generating translation constraints for robot manipulation.
    """

    # Constraint parameters
    lock_x: bool = Field(False, description="Whether to lock translation along the x-axis")
    lock_y: bool = Field(False, description="Whether to lock translation along the y-axis")
    lock_z: bool = Field(False, description="Whether to lock translation along the z-axis")

    reference_point: Optional[Tuple[float, float]] = Field(
        None, description="Reference point (x,y) on the target object for translation constraining"
    )

    bounds_min: Optional[Tuple[float, float]] = Field(
        None, description="Minimum bounds (x,y) for bounded translation"
    )

    bounds_max: Optional[Tuple[float, float]] = Field(
        None, description="Maximum bounds (x,y) for bounded translation"
    )

    target_point: Optional[Tuple[float, float]] = Field(
        None, description="Final target position (x,y) for translation constraining"
    )

    # Description
    description: str = Field("", description="Description of the translation constraint")

    def __call__(self) -> TranslationConstraint:
        """
        Generate a translation constraint based on the parameters.

        Returns:
            TranslationConstraint: The generated constraint
        """
        # Create reference point vector if provided (convert 2D point to 3D vector with z=0)
        reference_point = None
        if self.reference_point:
            reference_point = Vector(self.reference_point[0], self.reference_point[1], 0.0)

        # Create bounds minimum vector if provided
        bounds_min = None
        if self.bounds_min:
            bounds_min = Vector(self.bounds_min[0], self.bounds_min[1], 0.0)

        # Create bounds maximum vector if provided
        bounds_max = None
        if self.bounds_max:
            bounds_max = Vector(self.bounds_max[0], self.bounds_max[1], 0.0)

        # Create relative target vector if provided
        target_point = None
        if self.target_point:
            target_point = Vector(self.target_point[0], self.target_point[1], 0.0)

        # Create and return the constraint
        constraint = TranslationConstraint(
            lock_x=self.lock_x,
            lock_y=self.lock_y,
            lock_z=self.lock_z,
            reference_point=reference_point,
            bounds_min=bounds_min,
            bounds_max=bounds_max,
            target_point=target_point,
            description=self.description,
        )

        # Log the constraint creation
        logger.info(f"Generated translation constraint: {self.description}")

        return constraint
