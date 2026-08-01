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

"""Strict, immutable v1 records for public and private spatial corpus bundles."""

from __future__ import annotations

from enum import Enum
import json
from pathlib import Path
from typing import Annotated, Literal, TypeAlias

from pydantic import AfterValidator, BaseModel, ConfigDict, Field, PositiveFloat, model_validator

from dimos.benchmark.spatial.utilities import validate_relative_path

SchemaVersion: TypeAlias = Literal["1.0"]
OpaqueId = Annotated[str, Field(pattern=r"^[A-Za-z0-9_-]+_[0-9a-f]{64}$")]
Sha256 = Annotated[str, Field(pattern=r"^[0-9a-f]{64}$")]
RelativePath = Annotated[str, AfterValidator(validate_relative_path)]
NonEmptyText = Annotated[str, Field(min_length=1)]


class SpatialModel(BaseModel):
    """Common strict and frozen configuration for every corpus record."""

    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)


class Split(str, Enum):
    DEVELOPMENT = "development"
    HELD_OUT = "held-out"


class MapVariant(str, Enum):
    CLEAN = "clean"
    NOISY_01 = "noisy-01"
    NOISY_02 = "noisy-02"


class Predicate(str, Enum):
    POSE_OCCUPANCY = "pose-occupancy"
    STRAIGHT_TRANSLATION = "straight-translation"
    IN_PLACE_ROTATION = "in-place-rotation"
    ELIGIBLE_ROOM_COUNT = "eligible-room-count"
    SAME_ROOM = "same-room"
    DIRECT_ROOM_CONNECTION = "direct-room-connection"
    DIRECT_NEIGHBOR_COUNT = "direct-neighbor-count"


class AnswerType(str, Enum):
    BOOLEAN = "boolean"
    INTEGER = "integer"


class Pose2D(SpatialModel):
    x_m: float
    y_m: float
    yaw_rad: float


class Point2D(SpatialModel):
    x_m: float
    y_m: float


class Polygon2D(SpatialModel):
    vertices: tuple[Point2D, ...] = Field(min_length=3)
    holes: tuple[tuple[Point2D, ...], ...] = ()


class Marker(SpatialModel):
    """Neutral public marker; it deliberately has no room or answer metadata."""

    marker_id: OpaqueId
    position: Point2D


class PoseOccupancyContract(SpatialModel):
    kind: Literal["pose-occupancy"] = "pose-occupancy"
    footprint_policy_version: NonEmptyText


class TranslationContract(SpatialModel):
    kind: Literal["straight-translation"] = "straight-translation"
    distance_m: PositiveFloat
    footprint_policy_version: NonEmptyText


class RotationContract(SpatialModel):
    kind: Literal["in-place-rotation"] = "in-place-rotation"
    yaw_delta_rad: float
    footprint_policy_version: NonEmptyText


class EligibleRoomCountContract(SpatialModel):
    kind: Literal["eligible-room-count"] = "eligible-room-count"


class SameRoomContract(SpatialModel):
    kind: Literal["same-room"] = "same-room"
    first_marker_id: OpaqueId
    second_marker_id: OpaqueId


class DirectRoomConnectionContract(SpatialModel):
    kind: Literal["direct-room-connection"] = "direct-room-connection"
    first_marker_id: OpaqueId
    second_marker_id: OpaqueId


class DirectNeighborCountContract(SpatialModel):
    kind: Literal["direct-neighbor-count"] = "direct-neighbor-count"
    marker_id: OpaqueId


QuestionContract: TypeAlias = Annotated[
    PoseOccupancyContract
    | TranslationContract
    | RotationContract
    | EligibleRoomCountContract
    | SameRoomContract
    | DirectRoomConnectionContract
    | DirectNeighborCountContract,
    Field(discriminator="kind"),
]


class ManifestScene(SpatialModel):
    scene_id: OpaqueId
    split: Split
    scene_path: RelativePath


class Manifest(SpatialModel):
    record_type: Literal["manifest"] = "manifest"
    schema_version: SchemaVersion = "1.0"
    release_id: OpaqueId
    release_version: Annotated[str, Field(pattern=r"^v[0-9]+\.[0-9]+\.[0-9]+$")]
    generator_revision: NonEmptyText
    mapper_configuration_digest: Sha256
    source_dataset_revision: NonEmptyText
    scenes: tuple[ManifestScene, ...] = Field(min_length=1)


class Scene(SpatialModel):
    record_type: Literal["scene"] = "scene"
    schema_version: SchemaVersion = "1.0"
    scene_id: OpaqueId
    split: Split
    trajectory_ids: tuple[OpaqueId, ...] = Field(min_length=1)


class Trajectory(SpatialModel):
    record_type: Literal["trajectory"] = "trajectory"
    schema_version: SchemaVersion = "1.0"
    trajectory_id: OpaqueId
    scene_id: OpaqueId
    policy_version: NonEmptyText
    frame_id: NonEmptyText
    waypoints: tuple[Pose2D, ...] = Field(min_length=1)


class Question(SpatialModel):
    record_type: Literal["question"] = "question"
    schema_version: SchemaVersion = "1.0"
    question_id: OpaqueId
    scene_id: OpaqueId
    trajectory_id: OpaqueId
    predicate: Predicate
    template_version: NonEmptyText
    text: NonEmptyText
    answer_type: AnswerType
    contract: QuestionContract

    @model_validator(mode="after")
    def validate_contract(self) -> Question:
        if self.predicate.value != self.contract.kind:
            raise ValueError("predicate must match contract kind")
        expected_answer_type = (
            AnswerType.INTEGER
            if self.predicate in {Predicate.ELIGIBLE_ROOM_COUNT, Predicate.DIRECT_NEIGHBOR_COUNT}
            else AnswerType.BOOLEAN
        )
        if self.answer_type != expected_answer_type:
            raise ValueError("answer_type does not match predicate")
        return self


class FrameContract(SpatialModel):
    """Complete coordinate contract for a stored world-frame map artifact."""

    frame_id: NonEmptyText
    units: Literal["m"] = "m"
    handedness: Literal["right-handed"] = "right-handed"
    gravity_axis: Literal["+z"] = "+z"
    yaw_direction: Literal["counterclockwise"] = "counterclockwise"
    yaw_units: Literal["rad"] = "rad"


class MapperConfigurationRecord(SpatialModel):
    """Self-contained, serializable settings used to produce a map artifact."""

    voxel_size_m: PositiveFloat
    block_count: int = Field(gt=0)
    device: Literal["CPU:0"] = "CPU:0"
    carve_columns: Literal[True] = True
    frame_id: NonEmptyText
    emit_every: int = Field(gt=0)


class FrameConventionRecord(FrameContract):
    """Complete absolute world convention, including planar axis semantics."""

    space: Literal["metric"] = "metric"
    x_axis: Literal["forward"] = "forward"
    y_axis: Literal["left"] = "left"


class Snapshot(SpatialModel):
    record_type: Literal["snapshot"] = "snapshot"
    schema_version: SchemaVersion = "1.0"
    snapshot_id: OpaqueId
    scene_id: OpaqueId
    trajectory_id: OpaqueId
    variant: MapVariant
    terminal_pose: Pose2D
    map_artifact_path: RelativePath
    map_artifact_sha256: Sha256
    mapper_revision: NonEmptyText
    mapper_configuration_digest: Sha256
    mapper_configuration: MapperConfigurationRecord
    noise_profile_version: NonEmptyText
    seed: int
    frame_id: NonEmptyText
    frame_contract: FrameConventionRecord

    @model_validator(mode="after")
    def validate_frame_contract(self) -> Snapshot:
        """Keep the artifact's declared frame and coordinate contract inseparable."""

        if self.frame_id != self.frame_contract.frame_id:
            raise ValueError("snapshot frame_id must match frame_contract.frame_id")
        if self.frame_id != self.mapper_configuration.frame_id:
            raise ValueError("snapshot frame_id must match mapper_configuration.frame_id")
        return self


class OccupancyGeometry(SpatialModel):
    kind: Literal["pose-occupancy"] = "pose-occupancy"
    pose: Pose2D


class TranslationGeometry(SpatialModel):
    kind: Literal["straight-translation"] = "straight-translation"
    start_pose: Pose2D


class RotationGeometry(SpatialModel):
    kind: Literal["in-place-rotation"] = "in-place-rotation"
    pose: Pose2D


class MarkerGeometry(SpatialModel):
    kind: Literal["markers"] = "markers"
    markers: tuple[Marker, ...] = Field(min_length=1, max_length=2)


class EmptyGeometry(SpatialModel):
    kind: Literal["none"] = "none"


PublicQueryGeometry: TypeAlias = Annotated[
    OccupancyGeometry | TranslationGeometry | RotationGeometry | MarkerGeometry | EmptyGeometry,
    Field(discriminator="kind"),
]


class Instance(SpatialModel):
    record_type: Literal["instance"] = "instance"
    schema_version: SchemaVersion = "1.0"
    instance_id: OpaqueId
    question_id: OpaqueId
    snapshot_id: OpaqueId
    scene_id: OpaqueId
    trajectory_id: OpaqueId
    variant: MapVariant
    query_geometry: PublicQueryGeometry


class SourceProvenance(SpatialModel):
    record_type: Literal["source-provenance"] = "source-provenance"
    schema_version: SchemaVersion = "1.0"
    scene_id: OpaqueId
    source_dataset: NonEmptyText
    source_scene_key: NonEmptyText
    source_revision: NonEmptyText
    source_artifact_sha256: Sha256
    coordinate_frame_description: NonEmptyText


class Geometry(SpatialModel):
    record_type: Literal["geometry"] = "geometry"
    schema_version: SchemaVersion = "1.0"
    scene_id: OpaqueId
    floor_regions: tuple[Polygon2D, ...] = Field(min_length=1)
    blocked_regions: tuple[Polygon2D, ...]
    openings: tuple[Polygon2D, ...]
    barrier_segments: tuple[BarrierSegment, ...] = ()
    units: Literal["m"] = "m"
    handedness: Literal["right-handed"] = "right-handed"
    gravity_axis: Literal["+z"] = "+z"


class Room(SpatialModel):
    room_id: OpaqueId
    boundary: Polygon2D


class OpeningEdge(SpatialModel):
    opening_id: OpaqueId
    first_room_id: OpaqueId
    second_room_id: OpaqueId

    @model_validator(mode="after")
    def validate_distinct_rooms(self) -> OpeningEdge:
        if self.first_room_id == self.second_room_id:
            raise ValueError("an opening must connect two distinct rooms")
        return self


class BarrierSegment(SpatialModel):
    """An authoritative impermeable 2-D wall segment.

    Doorway spans are removed before this record is emitted, so both topology and
    collision consume exactly the same physical boundary representation.
    """

    start: Point2D
    end: Point2D

    @model_validator(mode="after")
    def validate_nonzero_length(self) -> BarrierSegment:
        if self.start == self.end:
            raise ValueError("barrier segments must have positive length")
        return self


class CoverageConnector(SpatialModel):
    """Private routing-only portal across a validated opening.

    The connector is not public geometry and does not widen walkable space; it
    only records two collision-verified approach points used by offline coverage
    trajectory routing when footprint erosion disconnects rooms at thin portals.
    """

    start: Point2D
    end: Point2D


class FreeSpaceModel(SpatialModel):
    """Private authoritative walkable union and its remaining barrier segments."""

    floor_regions: tuple[Polygon2D, ...] = Field(min_length=1)
    barriers: tuple[BarrierSegment, ...]
    coverage_connectors: tuple[CoverageConnector, ...] = ()


class Topology(SpatialModel):
    record_type: Literal["topology"] = "topology"
    schema_version: SchemaVersion = "1.0"
    scene_id: OpaqueId
    rooms: tuple[Room, ...] = Field(min_length=1)
    direct_openings: tuple[OpeningEdge, ...]


class BooleanAnswerValue(SpatialModel):
    kind: Literal["boolean"] = "boolean"
    value: bool


class IntegerAnswerValue(SpatialModel):
    kind: Literal["integer"] = "integer"
    value: int = Field(ge=0)


OracleAnswerValue: TypeAlias = Annotated[
    BooleanAnswerValue | IntegerAnswerValue,
    Field(discriminator="kind"),
]


class Answer(SpatialModel):
    record_type: Literal["answer"] = "answer"
    schema_version: SchemaVersion = "1.0"
    question_id: OpaqueId
    predicate: Predicate
    value: OracleAnswerValue
    oracle_policy_version: NonEmptyText


class OracleQuestionGeometry(SpatialModel):
    """Private physical query geometry in authoritative oracle/world frame."""

    record_type: Literal["oracle-question-geometry"] = "oracle-question-geometry"
    schema_version: SchemaVersion = "1.0"
    question_id: OpaqueId
    pose: Pose2D | None = None
    markers: tuple[Marker, ...] = ()


class ReviewAction(str, Enum):
    EXCLUDE = "exclude"
    CORRECT = "correct"


class ReviewOverride(SpatialModel):
    record_type: Literal["review-override"] = "review-override"
    schema_version: SchemaVersion = "1.0"
    override_id: OpaqueId
    question_id: OpaqueId
    action: ReviewAction
    reason: NonEmptyText
    corrected_value: OracleAnswerValue | None = None

    @model_validator(mode="after")
    def validate_correction(self) -> ReviewOverride:
        if self.action is ReviewAction.CORRECT and self.corrected_value is None:
            raise ValueError("correct overrides require corrected_value")
        if self.action is ReviewAction.EXCLUDE and self.corrected_value is not None:
            raise ValueError("exclude overrides must not contain corrected_value")
        return self


RECORD_MODELS: tuple[type[SpatialModel], ...] = (
    Manifest,
    Scene,
    Trajectory,
    Question,
    Snapshot,
    Instance,
    SourceProvenance,
    Geometry,
    Topology,
    Answer,
    OracleQuestionGeometry,
    ReviewOverride,
)


def write_record_schemas(target_directory: Path) -> tuple[Path, ...]:
    """Write deterministic JSON Schemas for every top-level corpus record model."""

    target_directory.mkdir(parents=True, exist_ok=True)
    written_paths: list[Path] = []
    for model in RECORD_MODELS:
        filename = f"{model.__name__.lower()}.schema.json"
        path = target_directory / filename
        schema_json = json.dumps(
            model.model_json_schema(),
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        path.write_text(f"{schema_json}\n", encoding="utf-8")
        written_paths.append(path)
    return tuple(written_paths)
