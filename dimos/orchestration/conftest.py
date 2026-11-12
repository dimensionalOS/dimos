# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Callable, Iterator, Mapping

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.orchestration.definition import (
    Activity,
    Event,
    Node,
    OrchestratorDefinition,
    Signal,
    Transition,
)


@pytest.fixture
def patrol() -> OrchestratorDefinition:
    return OrchestratorDefinition(
        root="Demo",
        resources=["base"],
        constants={"route": ["lobby", "lab", "kitchen"], "SUCCEEDED": 3},
        vars={"patrol.index": 0},
        signals=[
            Signal(name="estop_button", stream="FakeEstop.pressed", field="data"),
            Signal(name="nav_status", stream="FakeBase.status"),
        ],
        events=[
            Event(name="estop", when="estop_button"),
            Event(name="estop_cleared", when="!estop_button"),
            Event(
                name="arrived",
                when="nav_status.status == SUCCEEDED && nav_status.goal_id.id == nav.goal",
            ),
        ],
        activities=[
            Activity(name="go_to", start="FakeBase.go_to", cancel="FakeBase.stop", uses=["base"]),
        ],
        nodes=[
            Node(name="Demo", children=["Patrolling", "Estopped"], initial="Patrolling"),
            Node(
                name="Patrolling",
                children=["NextWaypoint", "Navigating"],
                initial="NextWaypoint",
                transitions=[Transition(trigger="estop", mode="immediate", goto="Estopped")],
            ),
            Node(
                name="NextWaypoint",
                transitions=[
                    Transition(if_="patrol.index < size(route)", goto="Navigating"),
                    Transition(goto="Navigating", set={"patrol.index": 0}),
                ],
            ),
            Node(
                name="Navigating",
                activity="go_to",
                args={"name": "route[patrol.index]"},
                result="nav.goal",
                transitions=[
                    Transition(
                        trigger="arrived",
                        goto="NextWaypoint",
                        set={"patrol.index": "patrol.index + 1"},
                    )
                ],
            ),
            Node(
                name="Estopped",
                transitions=[Transition(trigger="estop_cleared", goto="Patrolling")],
            ),
        ],
    )


@pytest.fixture
def build() -> Iterator[Callable[..., ModuleCoordinator]]:
    """Builds a blueprint with no viewer, and stops it after the test."""
    built: list[ModuleCoordinator] = []

    def _build(
        blueprint: Blueprint, overrides: Mapping[str, object] | None = None
    ) -> ModuleCoordinator:
        settings = {"g": {"viewer": "none"}, **(overrides or {})}
        parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides=settings)
        built.append(ModuleCoordinator.build(blueprint, parsed))
        return built[-1]

    yield _build
    for coordinator in built:
        coordinator.stop()
