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

from typing import Optional, List, Tuple, Dict, Any, Union
from pydantic import Field, validator

from dimos.skills.skills import AbstractRobotSkill
from dimos.types.manipulation_constraint import (
    AbstractConstraint,
    ManipulationConstraint,
    TranslationConstraint,
    RotationConstraint,
    ForceConstraint,
    ConstraintType,
)
from dimos.utils.logging_config import setup_logger

# Initialize logger
logger = setup_logger("dimos.skills.manipulate_skill")


class Manipulate(AbstractRobotSkill):
    """
    Skill for executing manipulation tasks with constraints.
    Can be called by an LLM with a list of manipulation constraints.
    """

    # Core parameters
    name: str = Field("Manipulation Task", description="Name of the manipulation task")

    description: str = Field("", description="Description of the manipulation task")

    # Target object information
    target_object: str = Field(
        "", description="Semantic label of the target object (e.g., 'cup', 'box')"
    )

    # Constraints - can be set directly
    constraints: List[Union[AbstractConstraint, str]] = Field(
        [],
        description="List of AbstractConstraint objects or constraint IDs from AgentMemory to apply to the manipulation task",
    )

    # Additional metadata TODO: Maybe put the movement tolerances and other LLM generated parameters here
    # metadata: Dict[str, Any] = Field(
    #     None,
    #     description="Additional metadata for the manipulation task"
    # )

    def __call__(self) -> Dict[str, Any]:
        """
        Execute a manipulation task with the given constraints.

        Returns:
            Dict[str, Any]: Result of the manipulation operation
        """
        # Create the manipulation constraint object
        manipulation_constraint = self._build_manipulation_constraint()

        if not manipulation_constraint or not manipulation_constraint.constraints:
            logger.error("No valid constraints provided for manipulation")
            return {"success": False, "error": "No valid constraints provided"}

        # Execute the manipulation with the constraint
        result = self._execute_manipulation(manipulation_constraint)

        # Log the execution
        constraint_types = [
            c.get_constraint_type().value for c in manipulation_constraint.constraints
        ]
        logger.info(f"Executed manipulation '{self.name}' with constraints: {constraint_types}")

        return result

    def _build_manipulation_constraint(self) -> ManipulationConstraint:
        """
        Build a ManipulationConstraint object from the provided parameters.
        """
        # Initialize the task manipulation constraint
        constraint = ManipulationConstraint(
            name=self.name,
            description=self.description,
            target_object=self.target_object,
            # metadata=self.metadata or {}
        )

        # Add constraints directly or resolve from IDs
        for c in self.constraints:
            if isinstance(c, AbstractConstraint):
                constraint.add_constraint(c)
            elif isinstance(c, str) and self._robot and hasattr(self._robot, "get_constraint"):
                # Try to load constraint from ID
                saved_constraint = self._robot.get_constraint(
                    c
                )  # TODO: implement constraint ID retrieval library
                if saved_constraint:
                    constraint.add_constraint(saved_constraint)

        return constraint

    # TODO: Implement
    def _execute_manipulation(self, constraint: ManipulationConstraint) -> Dict[str, Any]:
        """
        Execute the manipulation with the given constraint.

        Args:
            constraint: The manipulation constraint to use

        Returns:
            Dict[str, Any]: Result of the manipulation operation
        """
        pass
