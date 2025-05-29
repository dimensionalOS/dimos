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
from dimos.types.manipulation_constraint import ForceConstraint, Vector
from dimos.utils.logging_config import setup_logger

# Initialize logger
logger = setup_logger("dimos.skills.force_constraint_skill")


class ForceConstraintSkill(AbstractRobotSkill):
    """
    Skill for generating force constraints for robot manipulation.
    """

    # Constraint parameters
    max_force: float = Field(0.0, description="Maximum force in newtons to apply")
    min_force: float = Field(0.0, description="Minimum force in newtons to apply")

    # Force direction as (x,y) tuple
    force_direction: Optional[Tuple[float, float]] = Field(
        None, description="Force direction vector (x,y)"
    )

    # Description
    description: str = Field("", description="Description of the force constraint")

    def __call__(self) -> ForceConstraint:
        """
        Generate a force constraint based on the parameters.

        Returns:
            ForceConstraint: The generated constraint
        """
        # Create force direction vector if provided (convert 2D point to 3D vector with z=0)
        force_direction_vector = None
        if self.force_direction:
            force_direction_vector = Vector(self.force_direction[0], self.force_direction[1], 0.0)

        # Create and return the constraint
        constraint = ForceConstraint(
            max_force=self.max_force,
            min_force=self.min_force,
            force_direction=force_direction_vector,
            description=self.description,
        )

        # Log the constraint creation
        logger.info(f"Generated force constraint: {self.description}")

        return constraint
