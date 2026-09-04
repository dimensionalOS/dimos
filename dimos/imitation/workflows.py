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

"""Built-in bindings for complete imitation-learning workflows."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
from typing import Any


@dataclass(frozen=True)
class ImitationWorkflow:
    """Bind collection, data preparation, and rollout for one robot setup."""

    name: str
    collection_method: str
    required_hardware: tuple[str, ...]
    collection_builder: str
    dataprep_profile: str
    rollout_builder: str

    def load_collection_builder(self) -> Any:
        return _load_reference(self.collection_builder)

    def load_dataprep_profile(self) -> Any:
        return _load_reference(self.dataprep_profile)

    def load_rollout_builder(self) -> Any:
        return _load_reference(self.rollout_builder)


_OPENYAM_BLUEPRINTS = "dimos.robot.manipulators.openyam.blueprints"
_OPENYAM_PROFILE = "dimos.robot.manipulators.openyam.learning"

WORKFLOWS = {
    workflow.name: workflow
    for workflow in (
        ImitationWorkflow(
            name="openyam-teach",
            collection_method="gravity-compensated hand guidance",
            required_hardware=("OpenYAM arm", "wrist RGB camera"),
            collection_builder=f"{_OPENYAM_BLUEPRINTS}.learning_collection:build_teach_collection",
            dataprep_profile=f"{_OPENYAM_PROFILE}:OPENYAM_TEACH_LEARNING_PROFILE",
            rollout_builder=f"{_OPENYAM_BLUEPRINTS}.learning_rollout:build_openyam_rollout",
        ),
        ImitationWorkflow(
            name="openyam-quest",
            collection_method="Quest teleoperation",
            required_hardware=("OpenYAM arm", "wrist RGB camera", "Quest headset"),
            collection_builder=f"{_OPENYAM_BLUEPRINTS}.learning_collection:build_quest_collection",
            dataprep_profile=f"{_OPENYAM_PROFILE}:OPENYAM_LEARNING_PROFILE",
            rollout_builder=f"{_OPENYAM_BLUEPRINTS}.learning_rollout:build_openyam_rollout",
        ),
    )
}


def get_workflow(name: str) -> ImitationWorkflow:
    """Return a built-in workflow by its CLI name."""
    try:
        return WORKFLOWS[name]
    except KeyError as exc:
        choices = ", ".join(sorted(WORKFLOWS))
        raise ValueError(f"unknown imitation workflow {name!r}; choose one of: {choices}") from exc


def _load_reference(reference: str) -> Any:
    module_name, attribute = reference.split(":", 1)
    module = importlib.import_module(module_name)
    return getattr(module, attribute)
