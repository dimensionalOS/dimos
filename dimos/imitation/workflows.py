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

"""Lazy catalogs for independently selectable collection and rollout setups."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
from typing import Any, TypeVar

T = TypeVar("T")


@dataclass(frozen=True)
class CollectionWorkflow:
    """Collection stack and its dataset contract."""

    name: str
    method: str
    required_hardware: tuple[str, ...]
    builder: str
    profile: str
    dual_can: bool = False

    def load_builder(self) -> Any:
        return _load_reference(self.builder)

    def load_profile(self) -> Any:
        return _load_reference(self.profile)


@dataclass(frozen=True)
class RolloutWorkflow:
    """Policy backend and robot stack for one rollout profile."""

    name: str
    backend: str
    required_hardware: tuple[str, ...]
    builder: str
    profile: str
    dual_can: bool = False

    def load_builder(self) -> Any:
        return _load_reference(self.builder)

    def load_profile(self) -> Any:
        return _load_reference(self.profile)


_OPENYAM_BLUEPRINTS = "dimos.robot.manipulators.openyam.blueprints"
_OPENYAM_PROFILE = "dimos.robot.manipulators.openyam.learning"
_DUAL_BLUEPRINTS = "dimos.robot.manipulators.dual_openyam.blueprints"
_DUAL_PROFILE = "dimos.robot.manipulators.dual_openyam.learning"

COLLECTION_WORKFLOWS = {
    workflow.name: workflow
    for workflow in (
        CollectionWorkflow(
            name="openyam-teach",
            method="gravity-compensated hand guidance",
            required_hardware=("OpenYAM arm", "wrist RGB camera"),
            builder=f"{_OPENYAM_BLUEPRINTS}.learning_collection:build_teach_collection",
            profile=f"{_OPENYAM_PROFILE}:OPENYAM_TEACH_IO",
        ),
        CollectionWorkflow(
            name="openyam-quest",
            method="Quest teleoperation",
            required_hardware=("OpenYAM arm", "wrist RGB camera", "Quest headset"),
            builder=f"{_OPENYAM_BLUEPRINTS}.learning_collection:build_quest_collection",
            profile=f"{_OPENYAM_PROFILE}:OPENYAM_QUEST_IO",
        ),
        CollectionWorkflow(
            name="dual-openyam-quest",
            method="bimanual Quest teleoperation",
            required_hardware=(
                "Dual OpenYAM arms",
                "left wrist RGB camera",
                "right wrist RGB camera",
                "Quest headset",
            ),
            builder=(f"{_DUAL_BLUEPRINTS}.learning_collection:build_dual_openyam_quest_collection"),
            profile=f"{_DUAL_PROFILE}:DUAL_OPENYAM_TWO_WRIST_IO",
            dual_can=True,
        ),
    )
}

ROLLOUT_WORKFLOWS = {
    workflow.name: workflow
    for workflow in (
        RolloutWorkflow(
            name="openyam-lerobot",
            backend="LeRobot",
            required_hardware=("OpenYAM arm", "wrist RGB camera"),
            builder=f"{_OPENYAM_BLUEPRINTS}.learning_rollout:build_openyam_rollout",
            profile=f"{_OPENYAM_PROFILE}:OPENYAM_QUEST_IO",
        ),
        RolloutWorkflow(
            name="dual-openyam-abc",
            backend="Amazon ABC-DiT",
            required_hardware=(
                "Dual OpenYAM arms",
                "top RGB camera",
                "left wrist RGB camera",
                "right wrist RGB camera",
            ),
            builder=(f"{_DUAL_BLUEPRINTS}.learning_rollout:build_dual_openyam_abc_rollout"),
            profile=f"{_DUAL_PROFILE}:DUAL_OPENYAM_ABC_IO",
            dual_can=True,
        ),
    )
}


def get_collection_workflow(name: str) -> CollectionWorkflow:
    """Return a collection workflow by its CLI name."""
    return _get(name, COLLECTION_WORKFLOWS, "collection")


def get_rollout_workflow(name: str) -> RolloutWorkflow:
    """Return a rollout workflow by its CLI name."""
    return _get(name, ROLLOUT_WORKFLOWS, "rollout")


def _get(name: str, catalog: dict[str, T], kind: str) -> T:
    try:
        return catalog[name]
    except KeyError as exc:
        choices = ", ".join(sorted(catalog))
        raise ValueError(f"unknown imitation {kind} {name!r}; choose one of: {choices}") from exc


def _load_reference(reference: str) -> Any:
    module_name, attribute = reference.split(":", 1)
    module = importlib.import_module(module_name)
    return getattr(module, attribute)
