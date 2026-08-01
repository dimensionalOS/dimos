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

"""Pure deterministic generators for the four apartment smoke questions."""

from __future__ import annotations

from collections.abc import Callable
from typing import Literal, cast

from dimos.benchmark.dimsim.config import (
    CATEGORY_ORDER,
    CLEARANCE_POLICY_VERSION,
    COMPARISON_STABILITY_MARGIN_M,
    DESTINATION_ANGULAR_SPEED_TOLERANCE_RAD_S,
    DESTINATION_LINEAR_SPEED_TOLERANCE_M_S,
    DESTINATION_STATIONARY_DWELL_S,
    DESTINATION_THRESHOLD_M,
    EMBODIMENT_CLEARANCE_M,
    GENERATOR_REVISION,
    POWER_VOCABULARY,
    PREDICATE_POLICY_VERSION,
    PUBLIC_TEMPLATES,
    TEMPLATE_VERSION,
)
from dimos.benchmark.dimsim.geometry import feasible_stopping_region, surface_distance
from dimos.benchmark.dimsim.models import (
    ArgminDistanceContract,
    Category,
    CompiledTask,
    CountClassContract,
    Entity,
    EntityChoiceOutcome,
    EntityStateContract,
    EnumOutcome,
    ExpectedOutcome,
    ExpectedValue,
    IntegerOutcome,
    NavigateContract,
    PublicTask,
    ResponseType,
    SceneOracleView,
    SourceProvenance,
    TaskContract,
    TaskContractPayload,
    TerminalOutcome,
)
from dimos.benchmark.dimsim.utilities import oracle_view_digest, outcome_id, task_id
from dimos.benchmark.spatial.utilities import JsonValue


class GenerationError(ValueError):
    """Raised when a mandatory semantic or objectivity gate rejects a task."""


def require_provenance(view: SceneOracleView, required_path: str) -> None:
    """Require authored or policy-derived support for a truth-bearing field."""

    declared = {path for group in view.provenance for path in group.field_paths}
    wildcard = ".".join("*" if part.isdigit() else part for part in required_path.split("."))
    if required_path not in declared and wildcard not in declared:
        raise GenerationError(f"missing semantic provenance for {required_path}")


def resolve_entities(
    view: SceneOracleView,
    semantic_class: str,
    *,
    cardinality: int | None = None,
) -> tuple[Entity, ...]:
    entities = tuple(
        sorted(
            (
                entity
                for entity in view.entities
                if entity.task_eligible and entity.semantic_class == semantic_class
            ),
            key=lambda entity: entity.entity_id,
        )
    )
    if cardinality is not None and len(entities) != cardinality:
        raise GenerationError(
            f"semantic class {semantic_class!r} requires exactly {cardinality} "
            f"eligible entities; found {len(entities)}"
        )
    return entities


def _source(view: SceneOracleView) -> SourceProvenance:
    return SourceProvenance(
        scene_id=view.scene_id,
        scene_revision=view.scene_revision,
        reset_revision=view.reset_revision,
        semantic_schema_version=view.schema_version,
        profile_revision=view.profile_revision,
        upstream_revision=view.upstream_revision,
        oracle_view_digest=oracle_view_digest(view),
        generator_revision=GENERATOR_REVISION,
        predicate_policy_version=PREDICATE_POLICY_VERSION,
        template_version=TEMPLATE_VERSION,
    )


def _compiled(
    view: SceneOracleView,
    category: str,
    response_type: str,
    identity: dict[str, str | float | list[str]],
    contract: TaskContractPayload,
    expected: ExpectedValue,
    enum_values: tuple[str, ...] | None = None,
) -> CompiledTask:
    identity_payload: dict[str, JsonValue] = cast("dict[str, JsonValue]", identity)
    identifier = task_id(identity_payload)
    digest = oracle_view_digest(view)
    expected_payload = cast("JsonValue", expected.model_dump(mode="json"))
    return CompiledTask(
        public=PublicTask(
            task_id=identifier,
            category=cast("Category", category),
            text=PUBLIC_TEMPLATES[category],
            template_version=TEMPLATE_VERSION,
            response_type=cast("ResponseType", response_type),
            enum_values=enum_values,
        ),
        contract=TaskContract(
            task_id=identifier,
            identity_payload=identity,
            contract=contract,
            source=_source(view),
        ),
        outcome=ExpectedOutcome(
            outcome_id=outcome_id(identifier, digest, expected_payload),
            task_id=identifier,
            oracle_view_digest=digest,
            expected=expected,
        ),
    )


def generate_destination(view: SceneOracleView) -> CompiledTask:
    bathtub = resolve_entities(view, "bathtub", cardinality=1)[0]
    require_provenance(view, "entities.*.semantic_class")
    require_provenance(view, "entities.*.footprint")
    require_provenance(view, "navigation.*")
    require_provenance(view, "embodiment.canonical_spawn")
    feasible_stopping_region(
        bathtub.footprint,
        view.navigation,
        (view.embodiment.canonical_spawn.x_m, view.embodiment.canonical_spawn.z_m),
        DESTINATION_THRESHOLD_M,
        view.embodiment.footprint_radius_m,
        EMBODIMENT_CLEARANCE_M,
    )
    identity: dict[str, str | float | list[str]] = {
        "scene_id": view.scene_id,
        "category": "destination",
        "kind": "navigate-within-outer-footprint",
        "target_entity_id": bathtub.entity_id,
        "threshold_m": DESTINATION_THRESHOLD_M,
        "linear_speed_tolerance_m_s": DESTINATION_LINEAR_SPEED_TOLERANCE_M_S,
        "angular_speed_tolerance_rad_s": DESTINATION_ANGULAR_SPEED_TOLERANCE_RAD_S,
        "stationary_dwell_s": DESTINATION_STATIONARY_DWELL_S,
        "predicate_policy_version": PREDICATE_POLICY_VERSION,
        "template_version": TEMPLATE_VERSION,
    }
    return _compiled(
        view,
        "destination",
        "terminal",
        identity,
        NavigateContract(
            target_entity_id=bathtub.entity_id,
            threshold_m=DESTINATION_THRESHOLD_M,
            clearance_policy_version=CLEARANCE_POLICY_VERSION,
            linear_speed_tolerance_m_s=DESTINATION_LINEAR_SPEED_TOLERANCE_M_S,
            angular_speed_tolerance_rad_s=DESTINATION_ANGULAR_SPEED_TOLERANCE_RAD_S,
            stationary_dwell_s=DESTINATION_STATIONARY_DWELL_S,
        ),
        TerminalOutcome(),
    )


def generate_targeted_state(view: SceneOracleView) -> CompiledTask:
    television = resolve_entities(view, "television", cardinality=1)[0]
    require_provenance(view, "entities.*.semantic_class")
    require_provenance(view, "entities.*.properties")
    power = television.property("power")
    if power is None or not power.authoritative:
        raise GenerationError("television requires an authoritative power property")
    if power.value not in POWER_VOCABULARY:
        raise GenerationError(f"unsupported television power value: {power.value!r}")
    identity: dict[str, str | float | list[str]] = {
        "scene_id": view.scene_id,
        "category": "targeted-qa",
        "kind": "entity-state",
        "entity_id": television.entity_id,
        "property_name": "power",
        "predicate_policy_version": PREDICATE_POLICY_VERSION,
        "template_version": TEMPLATE_VERSION,
    }
    return _compiled(
        view,
        "targeted-qa",
        "enum",
        identity,
        EntityStateContract(
            entity_id=television.entity_id,
            vocabulary=("ON", "OFF"),
        ),
        EnumOutcome(value=cast("Literal['ON', 'OFF']", power.value)),
        POWER_VOCABULARY,
    )


def generate_count(view: SceneOracleView) -> CompiledTask:
    require_provenance(view, "entities.*.semantic_class")
    chairs = resolve_entities(view, "dining-chair")
    if not chairs:
        raise GenerationError("semantic class 'dining-chair' has no eligible entities")
    identity: dict[str, str | float | list[str]] = {
        "scene_id": view.scene_id,
        "category": "broad-exploration-qa",
        "kind": "count-semantic-class",
        "semantic_class": "dining-chair",
        "scope": "scene",
        "predicate_policy_version": PREDICATE_POLICY_VERSION,
        "template_version": TEMPLATE_VERSION,
    }
    return _compiled(
        view,
        "broad-exploration-qa",
        "integer",
        identity,
        CountClassContract(semantic_class="dining-chair"),
        IntegerOutcome(value=len(chairs)),
    )


def generate_comparison(view: SceneOracleView) -> CompiledTask:
    require_provenance(view, "entities.*.semantic_class")
    require_provenance(view, "entities.*.footprint")
    sofa = resolve_entities(view, "sofa", cardinality=1)[0]
    bathtub = resolve_entities(view, "bathtub", cardinality=1)[0]
    television = resolve_entities(view, "television", cardinality=1)[0]
    bathtub_distance = surface_distance(sofa.footprint, bathtub.footprint)
    television_distance = surface_distance(sofa.footprint, television.footprint)
    difference = abs(bathtub_distance - television_distance)
    if difference <= COMPARISON_STABILITY_MARGIN_M:
        raise GenerationError(
            f"comparison distance difference {difference:.6f}m does not exceed "
            f"{COMPARISON_STABILITY_MARGIN_M:.6f}m stability margin"
        )
    winner = bathtub if bathtub_distance < television_distance else television
    candidates = [bathtub.entity_id, television.entity_id]
    identity: dict[str, str | float | list[str]] = {
        "scene_id": view.scene_id,
        "category": "multi-hop-qa",
        "kind": "argmin-surface-distance",
        "anchor_entity_id": sofa.entity_id,
        "candidate_entity_ids": candidates,
        "stability_margin_m": COMPARISON_STABILITY_MARGIN_M,
        "predicate_policy_version": PREDICATE_POLICY_VERSION,
        "template_version": TEMPLATE_VERSION,
    }
    return _compiled(
        view,
        "multi-hop-qa",
        "entity-choice",
        identity,
        ArgminDistanceContract(
            anchor_entity_id=sofa.entity_id,
            candidate_entity_ids=(bathtub.entity_id, television.entity_id),
            stability_margin_m=COMPARISON_STABILITY_MARGIN_M,
        ),
        EntityChoiceOutcome(entity_id=winner.entity_id),
        (bathtub.aliases[0], television.aliases[0]),
    )


_GENERATORS: dict[str, Callable[[SceneOracleView], CompiledTask]] = {
    "destination": generate_destination,
    "targeted-qa": generate_targeted_state,
    "broad-exploration-qa": generate_count,
    "multi-hop-qa": generate_comparison,
}


def compile_smoke_tasks(view: SceneOracleView) -> tuple[CompiledTask, ...]:
    return tuple(_GENERATORS[category](view) for category in CATEGORY_ORDER)
