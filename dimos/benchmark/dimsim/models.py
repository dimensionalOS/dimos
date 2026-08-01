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

"""Strict private oracle and persisted DimSim corpus records."""

from __future__ import annotations

from itertools import pairwise
from typing import Annotated, Literal, TypeAlias

from pydantic import BaseModel, ConfigDict, Field, model_validator

from dimos.benchmark.dimsim.config import SCHEMA_VERSION, SEMANTIC_SCHEMA_VERSION

OpaqueId = Annotated[str, Field(pattern=r"^[a-z][a-z0-9_-]*_[0-9a-f]{64}$")]
Sha256 = Annotated[str, Field(pattern=r"^[0-9a-f]{64}$")]
NonEmpty = Annotated[str, Field(min_length=1)]
Point2 = tuple[float, float]
Polygon2 = Annotated[tuple[Point2, ...], Field(min_length=3)]


class StrictModel(BaseModel):
    """Reject coercion and unknown fields; keep records immutable."""

    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)


class FrameContract(StrictModel):
    frame_id: NonEmpty
    handedness: Literal["right"]
    length_unit: Literal["metre"]
    gravity_axis: Literal["+Y", "-Y"]
    horizontal_axes: Literal["XZ"]
    transform_convention: Literal["world-from-local"]
    policy_version: NonEmpty


class Pose2(StrictModel):
    x_m: float
    z_m: float
    yaw_rad: float


class Embodiment(StrictModel):
    embodiment_id: NonEmpty
    footprint_radius_m: Annotated[float, Field(gt=0)]
    canonical_spawn: Pose2


class NavigationGeometry(StrictModel):
    navigable: tuple[Polygon2, ...] = Field(min_length=1)
    blocked: tuple[Polygon2, ...] = ()
    clearance_radius_m: Annotated[float, Field(ge=0)] = 0.0
    collision_source_count: Annotated[int, Field(ge=0)] = 0

    @model_validator(mode="after")
    def preclearance_has_collision_sources(self) -> NavigationGeometry:
        if self.clearance_radius_m > 0 and self.collision_source_count == 0:
            raise ValueError("precleared navigation requires collision sources")
        return self


class SemanticProperty(StrictModel):
    name: NonEmpty
    value: str | int | float | bool
    authoritative: bool


class Entity(StrictModel):
    entity_id: NonEmpty
    semantic_class: NonEmpty
    aliases: tuple[NonEmpty, ...] = Field(min_length=1)
    display_title: NonEmpty
    position: Point2
    yaw_rad: float
    footprint: Polygon2
    properties: tuple[SemanticProperty, ...] = ()
    region_ids: tuple[NonEmpty, ...] = ()
    task_eligible: bool = True

    @model_validator(mode="after")
    def unique_properties(self) -> Entity:
        names = [item.name for item in self.properties]
        if len(names) != len(set(names)):
            raise ValueError("entity property names must be unique")
        return self

    def property(self, name: str) -> SemanticProperty | None:
        return next((item for item in self.properties if item.name == name), None)


class Region(StrictModel):
    region_id: NonEmpty
    semantic_class: NonEmpty
    footprint: Polygon2


class ProvenanceGroup(StrictModel):
    source_kind: Literal["authored", "policy-derived"]
    source_revision: NonEmpty
    policy_version: str | None = None
    field_paths: tuple[NonEmpty, ...] = Field(min_length=1)


class RuntimePoint(StrictModel):
    x: float
    z: float


class RuntimeBounds(StrictModel):
    min_x: float
    max_x: float
    min_z: float
    max_z: float

    @model_validator(mode="after")
    def has_area(self) -> RuntimeBounds:
        if self.min_x >= self.max_x or self.min_z >= self.max_z:
            raise ValueError("runtime world bounds must have positive XZ area")
        return self


class RuntimeEntitySnapshot(StrictModel):
    entity_id: NonEmpty
    display_title: NonEmpty
    current_state_id: NonEmpty
    state_ids: tuple[NonEmpty, ...] = Field(min_length=1)
    position: RuntimePoint
    yaw_rad: float
    bounds: RuntimeBounds


class RuntimeGridRun(StrictModel):
    row: Annotated[int, Field(ge=0)]
    start_col: Annotated[int, Field(ge=0)]
    end_col: Annotated[int, Field(ge=0)]

    @model_validator(mode="after")
    def ordered_columns(self) -> RuntimeGridRun:
        if self.start_col > self.end_col:
            raise ValueError("navigation grid run columns must be ordered")
        return self


class RuntimeNavigationGrid(StrictModel):
    min_x: float
    min_z: float
    cell_size_m: Annotated[float, Field(gt=0)]
    width: Annotated[int, Field(gt=0)]
    height: Annotated[int, Field(gt=0)]
    clearance_radius_m: Annotated[float, Field(gt=0)]
    collision_source_count: Annotated[int, Field(gt=0)]
    reachable_runs: tuple[RuntimeGridRun, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def runs_are_in_bounds(self) -> RuntimeNavigationGrid:
        for run in self.reachable_runs:
            if run.row >= self.height or run.end_col >= self.width:
                raise ValueError("navigation grid run is outside declared dimensions")
        ordered = tuple(sorted(self.reachable_runs, key=lambda run: (run.row, run.start_col)))
        if self.reachable_runs != ordered:
            raise ValueError("navigation grid runs must use canonical row-major order")
        for previous, current in pairwise(ordered):
            if previous.row == current.row and current.start_col <= previous.end_col + 1:
                raise ValueError("navigation grid runs must be disjoint and maximally merged")
        return self


class RuntimeSceneSnapshot(StrictModel):
    snapshot_schema_version: Literal["1.0"]
    asset_count: Annotated[int, Field(gt=0)]
    missing_entity_ids: tuple[NonEmpty, ...]
    agent_position: RuntimePoint
    agent_radius_m: Annotated[float, Field(gt=0)]
    agent_half_height_m: Annotated[float, Field(gt=0)]
    navigation_grid: RuntimeNavigationGrid
    entities: tuple[RuntimeEntitySnapshot, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def has_unique_entities(self) -> RuntimeSceneSnapshot:
        entity_ids = [entity.entity_id for entity in self.entities]
        if len(entity_ids) != len(set(entity_ids)):
            raise ValueError("runtime snapshot entity IDs must be unique")
        return self


class SceneOracleView(StrictModel):
    schema_version: Literal["1.0"] = SEMANTIC_SCHEMA_VERSION
    scene_id: NonEmpty
    scene_revision: NonEmpty
    reset_revision: NonEmpty
    upstream_revision: NonEmpty
    profile_revision: NonEmpty
    frame: FrameContract
    embodiment: Embodiment
    navigation: NavigationGeometry
    entities: tuple[Entity, ...] = Field(min_length=1)
    regions: tuple[Region, ...] = ()
    provenance: tuple[ProvenanceGroup, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def coherent_references(self) -> SceneOracleView:
        entity_ids = [entity.entity_id for entity in self.entities]
        if len(entity_ids) != len(set(entity_ids)):
            raise ValueError("entity IDs must be unique")
        region_ids = [region.region_id for region in self.regions]
        if len(region_ids) != len(set(region_ids)):
            raise ValueError("region IDs must be unique")
        unknown = {
            region_id
            for entity in self.entities
            for region_id in entity.region_ids
            if region_id not in region_ids
        }
        if unknown:
            raise ValueError(f"unknown region references: {sorted(unknown)}")
        return self


Category: TypeAlias = Literal["destination", "targeted-qa", "broad-exploration-qa", "multi-hop-qa"]
ResponseType: TypeAlias = Literal["terminal", "enum", "integer", "entity-choice"]


class PublicTask(StrictModel):
    record_type: Literal["public-task"] = "public-task"
    schema_version: Literal["1.0"] = SCHEMA_VERSION
    task_id: OpaqueId
    category: Category
    text: NonEmpty
    template_version: NonEmpty
    response_type: ResponseType
    enum_values: tuple[NonEmpty, ...] | None = None


class NavigateContract(StrictModel):
    kind: Literal["navigate-within-outer-footprint"] = "navigate-within-outer-footprint"
    target_entity_id: NonEmpty
    threshold_m: Annotated[float, Field(gt=0)]
    metric: Literal["outer-footprint"] = "outer-footprint"
    clearance_policy_version: NonEmpty
    linear_speed_tolerance_m_s: Annotated[float, Field(gt=0)]
    angular_speed_tolerance_rad_s: Annotated[float, Field(gt=0)]
    stationary_dwell_s: Annotated[float, Field(gt=0)]


class EntityStateContract(StrictModel):
    kind: Literal["entity-state"] = "entity-state"
    entity_id: NonEmpty
    property_name: Literal["power"] = "power"
    vocabulary: tuple[Literal["ON", "OFF"], Literal["ON", "OFF"]]


class CountClassContract(StrictModel):
    kind: Literal["count-semantic-class"] = "count-semantic-class"
    semantic_class: NonEmpty
    scope: Literal["scene"] = "scene"


class ArgminDistanceContract(StrictModel):
    kind: Literal["argmin-surface-distance"] = "argmin-surface-distance"
    anchor_entity_id: NonEmpty
    candidate_entity_ids: tuple[NonEmpty, NonEmpty]
    metric: Literal["polygon-surface-distance"] = "polygon-surface-distance"
    stability_margin_m: Annotated[float, Field(gt=0)]


TaskContractPayload: TypeAlias = Annotated[
    NavigateContract | EntityStateContract | CountClassContract | ArgminDistanceContract,
    Field(discriminator="kind"),
]


class SourceProvenance(StrictModel):
    scene_id: NonEmpty
    scene_revision: NonEmpty
    reset_revision: NonEmpty
    semantic_schema_version: NonEmpty
    profile_revision: NonEmpty
    upstream_revision: NonEmpty
    oracle_view_digest: Sha256
    generator_revision: NonEmpty
    predicate_policy_version: NonEmpty
    template_version: NonEmpty


class TaskContract(StrictModel):
    record_type: Literal["task-contract"] = "task-contract"
    schema_version: Literal["1.0"] = SCHEMA_VERSION
    task_id: OpaqueId
    identity_payload: dict[str, str | float | list[str]]
    contract: TaskContractPayload
    source: SourceProvenance


class TerminalOutcome(StrictModel):
    kind: Literal["terminal-predicate"] = "terminal-predicate"
    predicate: Literal["navigate-within-outer-footprint"] = "navigate-within-outer-footprint"


class EnumOutcome(StrictModel):
    kind: Literal["enum"] = "enum"
    value: Literal["ON", "OFF"]


class IntegerOutcome(StrictModel):
    kind: Literal["integer"] = "integer"
    value: int


class EntityChoiceOutcome(StrictModel):
    kind: Literal["entity-choice"] = "entity-choice"
    entity_id: NonEmpty


ExpectedValue: TypeAlias = Annotated[
    TerminalOutcome | EnumOutcome | IntegerOutcome | EntityChoiceOutcome,
    Field(discriminator="kind"),
]


class ExpectedOutcome(StrictModel):
    record_type: Literal["expected-outcome"] = "expected-outcome"
    schema_version: Literal["1.0"] = SCHEMA_VERSION
    outcome_id: OpaqueId
    task_id: OpaqueId
    oracle_view_digest: Sha256
    expected: ExpectedValue


class Diagnostic(StrictModel):
    code: NonEmpty
    category: Category | None = None
    message: NonEmpty
    entity_ids: tuple[NonEmpty, ...] = ()


class GenerationCheck(StrictModel):
    name: NonEmpty
    passed: bool
    detail: NonEmpty


class GenerationReport(StrictModel):
    record_type: Literal["generation-report"] = "generation-report"
    schema_version: Literal["1.0"] = SCHEMA_VERSION
    complete: bool
    retained_task_count: int
    checks: tuple[GenerationCheck, ...]
    diagnostics: tuple[Diagnostic, ...] = ()


class Manifest(StrictModel):
    record_type: Literal["manifest"] = "manifest"
    schema_version: Literal["1.0"] = SCHEMA_VERSION
    release_id: OpaqueId
    release_version: Annotated[str, Field(pattern=r"^v[0-9]+\.[0-9]+\.[0-9]+$")]
    generator_revision: NonEmpty
    complete: bool
    task_count: int
    public_tasks_path: Literal["public/tasks.jsonl"] = "public/tasks.jsonl"


class CompiledTask(StrictModel):
    public: PublicTask
    contract: TaskContract
    outcome: ExpectedOutcome


PERSISTED_MODELS: tuple[type[BaseModel], ...] = (
    Manifest,
    PublicTask,
    TaskContract,
    ExpectedOutcome,
    GenerationReport,
)
