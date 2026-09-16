# Copyright 2026 Dimensional Inc.
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

"""Agent tools for explicitly controlled BEHAVIOR episodes."""

import json
from typing import Any, Protocol

from dimos.agents.annotation import skill
from dimos.core.module import Module
from dimos.simulation.behavior.types import BehaviorStatus, ControlMode, Operation, TaskSelection
from dimos.spec.utils import Spec


class BehaviorSpec(Spec, Protocol):
    def describe(self) -> dict[str, Any]: ...
    def get_status(self) -> BehaviorStatus: ...
    def get_ground_truth(self) -> dict[str, Any]: ...
    def list_tasks(self) -> list[TaskSelection]: ...
    def load_task(self, task: TaskSelection) -> str: ...
    def reset_task(self) -> str: ...
    def take_control(self, mode: ControlMode) -> str: ...
    def start_primitive(self, kind: str, primitive: str, target: str = "") -> str: ...
    def get_operation(self, operation_id: str) -> Operation: ...
    def cancel_operation(self, operation_id: str) -> None: ...


class BehaviorSkills(Module):
    _behavior: BehaviorSpec

    @skill
    def behavior_inspect(self, subject: str = "status") -> str:
        """Inspect status, capabilities, ground_truth, or installed tasks.

        Ground truth comes from the simulator, not camera perception.
        """
        match subject:
            case "status":
                return self._behavior.get_status().model_dump_json()
            case "capabilities":
                return json.dumps(self._behavior.describe())
            case "ground_truth":
                return json.dumps(self._behavior.get_ground_truth())
            case "tasks":
                return json.dumps([task.model_dump() for task in self._behavior.list_tasks()])
            case _:
                raise ValueError("Choose status, capabilities, ground_truth, or tasks")

    @skill
    def behavior_load(
        self, scene: str, activity: str, definition: int = 0, instance: int = 0
    ) -> str:
        """Load an installed task instance; returns an operation ID to poll."""
        return self._behavior.load_task(
            TaskSelection(scene=scene, activity=activity, definition=definition, instance=instance)
        )

    @skill
    def behavior_reset(self) -> str:
        """Reset the current task and return an operation ID to poll."""
        return self._behavior.reset_task()

    @skill
    def behavior_take_control(self, mode: str = "primitive") -> str:
        """Explicitly take robot control: primitive, dimos, or native. Cancels active motion."""
        return self._behavior.take_control(ControlMode(mode))

    @skill
    def behavior_act(self, primitive: str, target: str = "", kind: str = "physical") -> str:
        """Start a supported primitive after taking primitive control.

        kind is physical or symbolic. Symbolic actions directly change world state.
        target is an object ID from ground_truth; RELEASE needs no target.
        """
        return self._behavior.start_primitive(kind, primitive, target)

    @skill
    def behavior_operation(self, operation_id: str) -> str:
        """Read an operation's running, succeeded, failed, or cancelled result."""
        return self._behavior.get_operation(operation_id).model_dump_json()

    @skill
    def behavior_cancel(self, operation_id: str) -> str:
        """Request operation cancellation; poll until cancellation completes."""
        self._behavior.cancel_operation(operation_id)
        return "Cancellation requested; poll the operation for acknowledgement."


SYSTEM_PROMPT = """You control R1 in an OmniGibson BEHAVIOR environment.
Inspect capabilities, task status, and ground_truth before acting. Object identifiers
are scoped to the current episode. Take primitive control explicitly and wait for
that operation to succeed before starting an action. Poll operation handles; never
assume that accepting an action means it succeeded. Execute actions sequentially.
Use physical actions by default. Use symbolic actions only when the user requests
symbolic execution; they modify world state and do not demonstrate physical skill.
Some physical actions are unsupported. Do not substitute symbolic actions for them.
Task success comes from the episode evaluator, not your assessment or a single
primitive's result. Reset only when requested; resetting changes the episode.
"""
