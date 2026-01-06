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
from dimos.types.manipulation import TranslationConstraint, Vector  # type: ignore[attr-defined]
from dimos.utils.logging_config import setup_logger

# Initialize logger
logger = setup_logger()


class TranslationConstraintSkill(AbstractManipulationSkill):
    """
    Skill for generating translation constraints for robot manipulation.

    This skill generates translation constraints and adds them to the ManipulationInterface's
    agent_constraints list for tracking constraints created by the Agent.
    """

    # Constraint parameters
    translation_axis: Literal["x", "y", "z"] = Field(
        "x", description="Axis to translate along: 'x', 'y', or 'z'"
    )

    reference_point: tuple[float, float] | None = Field(
        None, description="Reference point (x,y) on the target object for translation constraining"
    )

    bounds_min: tuple[float, float] | None = Field(
        None, description="Minimum bounds (x,y) for bounded translation"
    )

    bounds_max: tuple[float, float] | None = Field(
        None, description="Maximum bounds (x,y) for bounded translation"
    )

    target_point: tuple[float, float] | None = Field(
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
            reference_point = Vector(self.reference_point[0], self.reference_point[1], 0.0)  # type: ignore[arg-type]

        # Create bounds minimum vector if provided
        bounds_min = None
        if self.bounds_min:
            bounds_min = Vector(self.bounds_min[0], self.bounds_min[1], 0.0)  # type: ignore[arg-type]

        # Create bounds maximum vector if provided
        bounds_max = None
        if self.bounds_max:
            bounds_max = Vector(self.bounds_max[0], self.bounds_max[1], 0.0)  # type: ignore[arg-type]

        # Create relative target vector if provided
        target_point = None
        if self.target_point:
            target_point = Vector(self.target_point[0], self.target_point[1], 0.0)  # type: ignore[arg-type]

        constraint = TranslationConstraint(
            translation_axis=self.translation_axis,
            reference_point=reference_point,
            bounds_min=bounds_min,
            bounds_max=bounds_max,
            target_point=target_point,
        )

        # Add constraint to manipulation interface
        self.manipulation_interface.add_constraint(constraint)  # type: ignore[union-attr]

        # Log the constraint creation
        logger.info(f"Generated translation constraint along {self.translation_axis} axis")

        return {"success": True}  # type: ignore[return-value]
