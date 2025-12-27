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

import time
from typing import Any
import uuid

from pydantic import Field

from dimos.skills.manipulation.abstract_manipulation_skill import AbstractManipulationSkill
from dimos.types.manipulation import (
    AbstractConstraint,
    ManipulationMetadata,
    ManipulationTask,
    ManipulationTaskConstraint,
)
from dimos.utils.logging_config import setup_logger

# Initialize logger
logger = setup_logger()


class Manipulate(AbstractManipulationSkill):
    """
    Skill for executing manipulation tasks with constraints.
    Can be called by an LLM with a list of manipulation constraints.
    """

    description: str = Field("", description="Description of the manipulation task")

    # Target object information
    target_object: str = Field(
        "", description="Semantic label of the target object (e.g., 'cup', 'box')"
    )

    target_point: str = Field(
        "", description="(X,Y) point in pixel-space of the point to manipulate on target object"
    )

    # Constraints - can be set directly
    constraints: list[str] = Field(
        [],
        description="List of AbstractConstraint constraint IDs from AgentMemory to apply to the manipulation task",
    )

    # Object movement tolerances
    object_tolerances: dict[str, float] = Field(
        {},  # Empty dict as default
        description="Dictionary mapping object IDs to movement tolerances (0.0 = immovable, 1.0 = freely movable)",
    )

    def __call__(self) -> dict[str, Any]:
        """
        Execute a manipulation task with the given constraints.

        Returns:
            Dict[str, Any]: Result of the manipulation operation
        """
        # Get the manipulation constraint
        constraint = self._build_manipulation_constraint()

        # Create task with unique ID
        task_id = f"{str(uuid.uuid4())[:4]}"
        timestamp = time.time()

        # Build metadata with environment state
        metadata = self._build_manipulation_metadata()

        task = ManipulationTask(
            description=self.description,
            target_object=self.target_object,
            target_point=tuple(map(int, self.target_point.strip("()").split(","))),  # type: ignore[arg-type]
            constraints=constraint,
            metadata=metadata,
            timestamp=timestamp,
            task_id=task_id,
            result=None,
        )

        # Add task to manipulation interface
        self.manipulation_interface.add_manipulation_task(task)  # type: ignore[union-attr]

        # Execute the manipulation
        result = self._execute_manipulation(task)

        # Log the execution
        logger.info(
            f"Executed manipulation '{self.description}' with constraints: {self.constraints}"
        )

        return result

    def _build_manipulation_metadata(self) -> ManipulationMetadata:
        """
        Build metadata for the current environment state, including object data and movement tolerances.
        """
        # Get detected objects from the manipulation interface
        detected_objects = []  # type: ignore[var-annotated]
        try:
            detected_objects = self.manipulation_interface.get_latest_objects() or []  # type: ignore[union-attr]
        except Exception as e:
            logger.warning(f"Failed to get detected objects: {e}")

        # Create dictionary of objects keyed by ID for easier lookup
        objects_by_id = {}
        for obj in detected_objects:
            obj_id = str(obj.get("object_id", -1))
            objects_by_id[obj_id] = dict(obj)  # Make a copy to avoid modifying original

        # Create objects_data dictionary with tolerances applied
        objects_data: dict[str, Any] = {}

        # First, apply all specified tolerances
        for object_id, tolerance in self.object_tolerances.items():
            if object_id in objects_by_id:
                # Object exists in detected objects, update its tolerance
                obj_data = objects_by_id[object_id]
                obj_data["movement_tolerance"] = tolerance
                objects_data[object_id] = obj_data

        # Add any detected objects not explicitly given tolerances
        for obj_id, obj in objects_by_id.items():
            if obj_id not in self.object_tolerances:
                obj["movement_tolerance"] = 0.0  # Default to immovable
                objects_data[obj_id] = obj

        # Create properly typed ManipulationMetadata
        metadata: ManipulationMetadata = {"timestamp": time.time(), "objects": objects_data}

        return metadata

    def _build_manipulation_constraint(self) -> ManipulationTaskConstraint:
        """
        Build a ManipulationTaskConstraint object from the provided parameters.
        """

        constraint = ManipulationTaskConstraint()

        # Add constraints directly or resolve from IDs
        for c in self.constraints:
            if isinstance(c, AbstractConstraint):
                constraint.add_constraint(c)
            elif isinstance(c, str) and self.manipulation_interface:
                # Try to load constraint from ID
                saved_constraint = self.manipulation_interface.get_constraint(c)
                if saved_constraint:
                    constraint.add_constraint(saved_constraint)

        return constraint

    # TODO: Implement
    def _execute_manipulation(self, task: ManipulationTask) -> dict[str, Any]:
        """
        Execute the manipulation with the given constraint.

        Args:
            task: The manipulation task to execute

        Returns:
            Dict[str, Any]: Result of the manipulation operation
        """
        return {"success": True}
