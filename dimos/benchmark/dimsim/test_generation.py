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

from pydantic import ValidationError
import pytest

from dimos.benchmark.dimsim.config import (
    CATEGORY_ORDER,
    DESTINATION_ANGULAR_SPEED_TOLERANCE_RAD_S,
    DESTINATION_LINEAR_SPEED_TOLERANCE_M_S,
    DESTINATION_STATIONARY_DWELL_S,
    PUBLIC_TEMPLATES,
)
from dimos.benchmark.dimsim.fixture import apartment_oracle_fixture
from dimos.benchmark.dimsim.generation import (
    GenerationError,
    compile_smoke_tasks,
    generate_comparison,
    generate_count,
    generate_destination,
    generate_targeted_state,
)
from dimos.benchmark.dimsim.models import (
    Entity,
    EntityChoiceOutcome,
    EnumOutcome,
    IntegerOutcome,
    ProvenanceGroup,
    SceneOracleView,
    TerminalOutcome,
)


def _replace_entity(view: SceneOracleView, entity: Entity) -> SceneOracleView:
    entities = tuple(
        entity if item.entity_id == entity.entity_id else item for item in view.entities
    )
    return view.model_copy(update={"entities": entities})


def test_destination_question_smoke() -> None:
    task = generate_destination(apartment_oracle_fixture())

    assert task.public.text == PUBLIC_TEMPLATES["destination"]
    assert task.contract.contract.kind == "navigate-within-outer-footprint"
    assert (
        task.contract.contract.linear_speed_tolerance_m_s == DESTINATION_LINEAR_SPEED_TOLERANCE_M_S
    )
    assert (
        task.contract.contract.angular_speed_tolerance_rad_s
        == DESTINATION_ANGULAR_SPEED_TOLERANCE_RAD_S
    )
    assert task.contract.contract.stationary_dwell_s == DESTINATION_STATIONARY_DWELL_S
    assert task.outcome.expected == TerminalOutcome()


def test_generated_source_carries_explicit_profile_revision() -> None:
    task = generate_destination(apartment_oracle_fixture())

    assert task.contract.source.profile_revision == "dimsim-apartment-profile-v1"


def test_television_question_smoke() -> None:
    task = generate_targeted_state(apartment_oracle_fixture())

    assert task.public.text == PUBLIC_TEMPLATES["targeted-qa"]
    assert task.outcome.expected == EnumOutcome(value="OFF")


def test_dining_chair_question_smoke_excludes_work_chair() -> None:
    task = generate_count(apartment_oracle_fixture())

    assert task.public.text == PUBLIC_TEMPLATES["broad-exploration-qa"]
    assert task.outcome.expected == IntegerOutcome(value=4)


def test_sofa_comparison_question_smoke_uses_surface_distance() -> None:
    task = generate_comparison(apartment_oracle_fixture())

    assert task.public.text == PUBLIC_TEMPLATES["multi-hop-qa"]
    assert task.outcome.expected == EntityChoiceOutcome(entity_id="apt-bathtub-01")


def test_smoke_compiler_has_fixed_four_category_order() -> None:
    tasks = compile_smoke_tasks(apartment_oracle_fixture())

    assert tuple(task.public.category for task in tasks) == CATEGORY_ORDER
    assert len({task.public.task_id for task in tasks}) == 4


def test_duplicate_semantic_entity_is_rejected() -> None:
    view = apartment_oracle_fixture()
    duplicate = view.entities[0].model_copy(update={"entity_id": "apt-bathtub-02"})
    changed = view.model_copy(update={"entities": (*view.entities, duplicate)})

    with pytest.raises(GenerationError, match="exactly 1"):
        generate_destination(changed)


def test_missing_semantic_class_is_not_inferred_from_display_title() -> None:
    view = apartment_oracle_fixture()
    bathtub = view.entities[0].model_copy(update={"semantic_class": "unknown"})

    with pytest.raises(GenerationError, match="found 0"):
        generate_destination(_replace_entity(view, bathtub))


def test_missing_truth_field_provenance_is_rejected() -> None:
    view = apartment_oracle_fixture().model_copy(
        update={
            "provenance": (
                ProvenanceGroup(
                    source_kind="authored",
                    source_revision="incomplete",
                    field_paths=("entities.*.aliases",),
                ),
            )
        }
    )

    with pytest.raises(GenerationError, match="missing semantic provenance"):
        generate_destination(view)


def test_visual_only_television_state_is_rejected() -> None:
    view = apartment_oracle_fixture()
    television = view.entities[1].model_copy(update={"properties": ()})

    with pytest.raises(GenerationError, match="authoritative power"):
        generate_targeted_state(_replace_entity(view, television))


def test_invalid_footprint_is_rejected_by_geometry_gate() -> None:
    view = apartment_oracle_fixture()
    bathtub = view.entities[0].model_copy(
        update={"footprint": ((0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0))}
    )

    with pytest.raises(ValueError, match="valid non-empty polygon"):
        generate_destination(_replace_entity(view, bathtub))


def test_near_tie_comparison_is_rejected() -> None:
    view = apartment_oracle_fixture()
    television = view.entities[1].model_copy(
        update={
            "position": (5.7, 2.0),
            "footprint": ((5.1, 1.6), (6.3, 1.6), (6.3, 2.4), (5.1, 2.4)),
        }
    )

    with pytest.raises(GenerationError, match="stability margin"):
        generate_comparison(_replace_entity(view, television))


def test_comparison_uses_surfaces_when_center_distance_disagrees() -> None:
    view = apartment_oracle_fixture()
    bathtub = view.entities[0].model_copy(
        update={
            "position": (0.0, 2.0),
            "footprint": ((-2.5, 1.6), (2.5, 1.6), (2.5, 2.4), (-2.5, 2.4)),
        }
    )
    television = view.entities[1].model_copy(
        update={
            "position": (6.2, 2.0),
            "footprint": ((6.0, 1.8), (6.4, 1.8), (6.4, 2.2), (6.0, 2.2)),
        }
    )
    changed = _replace_entity(_replace_entity(view, bathtub), television)

    task = generate_comparison(changed)

    assert task.outcome.expected == EntityChoiceOutcome(entity_id="apt-bathtub-01")


def test_answer_change_does_not_change_semantic_task_identity() -> None:
    off = generate_targeted_state(apartment_oracle_fixture(television_power="OFF"))
    on = generate_targeted_state(apartment_oracle_fixture(television_power="ON"))

    assert off.public.task_id == on.public.task_id
    assert off.outcome.outcome_id != on.outcome.outcome_id
    assert off.outcome.oracle_view_digest != on.outcome.oracle_view_digest


def test_predicate_policy_change_updates_semantic_id_but_not_source_digest(
    monkeypatch,
) -> None:
    view = apartment_oracle_fixture()
    before = compile_smoke_tasks(view)
    monkeypatch.setattr(
        "dimos.benchmark.dimsim.generation.PREDICATE_POLICY_VERSION",
        "dimsim-smoke-predicates-v3",
    )

    after = compile_smoke_tasks(view)

    assert {task.public.task_id for task in before}.isdisjoint(
        task.public.task_id for task in after
    )
    assert {task.contract.source.oracle_view_digest for task in before} == {
        task.contract.source.oracle_view_digest for task in after
    }
    assert {task.contract.source.predicate_policy_version for task in after} == {
        "dimsim-smoke-predicates-v3"
    }


def test_strict_model_does_not_coerce_state_answer() -> None:
    with pytest.raises(ValidationError):
        EnumOutcome(value=1)
