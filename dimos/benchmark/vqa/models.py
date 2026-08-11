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

"""Contracts shared by the single-frame perception VQA pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Protocol

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


@dataclass(frozen=True)
class CalibratedFrame:
    """One self-contained image and point-cloud pair for VQA generation."""

    id: str
    image: Image
    pointcloud: PointCloud2
    camera_info: CameraInfo
    pointcloud_to_camera: Transform
    image_is_rectified: bool
    original_image: Image | None = None


@dataclass(frozen=True)
class ProjectionConfig:
    """Controls static pinhole projection of a frame's point cloud."""

    min_depth_m: float = 0.01


@dataclass(frozen=True)
class GroundingConfig:
    """Quality thresholds for accepting an image mask as a grounded object."""

    min_mask_area_px: int = 128
    min_foreground_points: int = 3


@dataclass(frozen=True)
class ProjectedPoints:
    """Visible point-cloud samples represented in the camera image."""

    camera_points: list[tuple[float, float, float]]
    pixels: list[tuple[int, int]]
    source_indices: list[int]


@dataclass(frozen=True)
class GroundedObject:
    """One semantic object with foreground point-cloud support."""

    id: str
    label: str
    point_count: int
    range_m: float
    horizontal_direction: str


@dataclass(frozen=True)
class VqaExample:
    """A closed-answer question generated from grounded objects."""

    id: str
    question: str
    expected_answer: str
    answer_type: str
    object_ids: tuple[str, ...]
    allowed_answers: tuple[str, ...] = ()


QuestionKind = Literal[
    "presence",
    "horizontal_direction",
    "within_distance",
    "visible_count",
    "camera_range",
    "compare_nearest_by_side",
    "compare_left_right",
    "compare_height",
    "object_on_support",
    "opening_width",
    "door_state",
    "closest_object",
    "forward_path",
]


@dataclass(frozen=True)
class QuestionIntent:
    """A constrained question proposed from an image."""

    kind: QuestionKind
    object_query: str
    threshold_m: float | None = None
    candidate_queries: tuple[str, ...] = ()
    comparison_query: str | None = None


@dataclass(frozen=True)
class ToolTrace:
    """One perception operation used to establish a ground-truth answer."""

    tool: str
    detail: str


@dataclass(frozen=True)
class GroundTruthResult:
    """An answered or rejected question with private perception evidence."""

    intent: QuestionIntent
    question: VqaExample
    status: Literal["answered", "rejected"]
    answer: str | None
    reason: str | None
    evidence: tuple[GroundedObject, ...]
    trace: tuple[ToolTrace, ...]


@dataclass(frozen=True)
class BooleanAnswerContract:
    """A yes/no answer required from a private oracle."""

    kind: Literal["boolean"] = "boolean"


@dataclass(frozen=True)
class ChoiceAnswerContract:
    """An answer selected exactly from the supplied choices."""

    choices: tuple[str, ...]
    kind: Literal["choice"] = "choice"


@dataclass(frozen=True)
class DeferredHeightChoiceContract:
    """A height question whose public choices follow a private measurement."""

    strategy: Literal["height-window-v1"] = "height-window-v1"
    kind: Literal["deferred_height_choice"] = "deferred_height_choice"


AnswerContract = BooleanAnswerContract | ChoiceAnswerContract | DeferredHeightChoiceContract
ResolvedAnswerContract = BooleanAnswerContract | ChoiceAnswerContract


@dataclass(frozen=True)
class QuestionProposal:
    """Public image-only question frozen before private oracle execution."""

    id: str
    question: str
    answer_contract: AnswerContract
    object_queries: tuple[str, ...] = ()
    tool_hints: tuple[str, ...] = ()


@dataclass(frozen=True)
class OracleEvidence:
    """A private evidence item emitted by a registered local tool."""

    id: str
    version: str
    object_id: str
    label: str
    range_m: float
    side: str
    point_count: int
    measurement: OracleMeasurement | None = None


@dataclass(frozen=True)
class OracleMeasurement:
    """A private scalar measurement with its uncertainty and provenance."""

    value: float
    unit: str
    tolerance: float
    quality_flags: tuple[str, ...]
    provenance_ids: tuple[str, ...]


@dataclass(frozen=True)
class GroundPlaneEstimate:
    """A robust ground-plane fit in frozen camera coordinates."""

    normal: tuple[float, float, float]
    offset_m: float
    sample_count: int
    inlier_count: int
    residual_m: float


@dataclass(frozen=True)
class OracleToolResult:
    """Structured result and evidence IDs from one local tool invocation."""

    tool: str
    query: str
    evidence: tuple[OracleEvidence, ...]
    version: str = "v1"
    measurement: OracleMeasurement | None = None
    choice: str | None = None
    choices: tuple[str, ...] = ()
    plane: GroundPlaneEstimate | None = None
    quality_flags: tuple[str, ...] = ()
    rejection_reason: str | None = None
    metrics: tuple[tuple[str, float], ...] = ()


@dataclass(frozen=True)
class OracleTrace:
    """Audit record for a private oracle model or tool operation."""

    operation: str
    detail: str


@dataclass(frozen=True)
class AcceptedOracleResult:
    """Validated private answer for a frozen public proposal."""

    proposal: QuestionProposal
    answer: str
    answer_contract: ResolvedAnswerContract
    evidence_ids: tuple[str, ...]
    tool_results: tuple[OracleToolResult, ...]
    trace: tuple[OracleTrace, ...]


@dataclass(frozen=True)
class RejectedOracleResult:
    """Private oracle attempt that cannot be safely exported as a case."""

    proposal: QuestionProposal
    reason: str
    tool_results: tuple[OracleToolResult, ...]
    trace: tuple[OracleTrace, ...]


class ObjectDetector(Protocol):
    """Produces object-query detections from one image."""

    def detect(self, image: Image, query: str) -> ImageDetections2D: ...


class ObjectSegmenter(Protocol):
    """Refines object detections into foreground masks in one image."""

    def segment(self, detections: ImageDetections2D) -> ImageDetections2D: ...


class ObjectPointLocalizer(Protocol):
    """Locates a queried object with positive image points."""

    def locate(self, image: Image, query: str) -> ImageDetections2D: ...


class PointObjectSegmenter(Protocol):
    """Creates foreground masks from positive image-point prompts."""

    def segment_points(self, points: ImageDetections2D) -> ImageDetections2D: ...
