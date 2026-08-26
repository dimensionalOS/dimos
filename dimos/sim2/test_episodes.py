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

"""Focused checks for typed public episode task views."""

import pytest

from dimos.sim2.episodes import (
    MultiObjectRelationKind,
    PublicEpisodeContext,
    PublicEpisodeRegion,
    PublicEpisodeRole,
    PublicEpisodeTarget,
    PublicEpisodeTargetKind,
)


def _role(role_id: str) -> PublicEpisodeRole:
    return PublicEpisodeRole(
        role_id=role_id,
        entity_id=f"entity-{role_id}",
        name=role_id.replace("_", " "),
    )


def _region(role_id: str, entity_role_id: str) -> PublicEpisodeRegion:
    return PublicEpisodeRegion(
        role_id=role_id,
        entity_role_id=entity_role_id,
        entity_id=f"entity-{entity_role_id}",
        region_id=f"region-{role_id}",
        kind="support",
    )


def test_navigation_view_exposes_the_goal_without_role_strings() -> None:
    destination = _role("destination")
    context = PublicEpisodeContext(
        case_id="navigate-case",
        task_family_id="navigate-to-region",
        instruction="Navigate to the loading zone.",
        roles={"destination": destination},
        targets={
            "goal": PublicEpisodeTarget(
                target_id="goal",
                kind=PublicEpisodeTargetKind.NAVIGATION,
                entity_role_id="destination",
                entity_id=destination.entity_id,
                region_role_id="goal",
                region_id="loading-zone",
                position=(2.0, 3.0, 0.0),
            )
        },
    )

    assert context.navigation().goal.position == (2.0, 3.0, 0.0)


def test_containment_view_exposes_object_source_and_interior() -> None:
    object_role = _role("object")
    surface = _role("surface")
    target = _role("target")
    context = PublicEpisodeContext(
        case_id="containment-case",
        task_family_id="object-in-receptacle",
        instruction="Put the cup in the basket.",
        roles={"object": object_role, "surface": surface, "target": target},
        regions={
            "source": _region("source", "surface"),
            "destination": _region("destination", "target"),
        },
    )

    task = context.containment()

    assert task.object is object_role
    assert task.source.entity_role_id == "surface"
    assert task.interior.entity_role_id == "target"
    with pytest.raises(ValueError, match="does not expose a lift view"):
        context.lift()


def test_multi_object_view_names_each_goal_relation() -> None:
    roles = {
        role_id: _role(role_id)
        for role_id in ("first_object", "second_object", "target", "surface")
    }
    context = PublicEpisodeContext(
        case_id="collect-case",
        task_family_id="collect-objects-in-receptacle",
        instruction="Put both objects in the basket.",
        roles=roles,
        regions={
            "source": _region("source", "surface"),
            "destination": _region("destination", "target"),
        },
    )

    task = context.multi_object()

    assert tuple(role.role_id for role in task.objects) == (
        "first_object",
        "second_object",
    )
    assert tuple(relation.kind for relation in task.relations) == (
        MultiObjectRelationKind.CONTAINED_IN,
        MultiObjectRelationKind.CONTAINED_IN,
    )
