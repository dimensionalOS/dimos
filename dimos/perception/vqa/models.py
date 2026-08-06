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


@dataclass(frozen=True)
class VqaEvaluation:
    """The result of asking an image-only agent one generated question."""

    example_id: str
    expected_answer: str
    raw_response: str
    normalized_response: str | None
    passed: bool


QuestionKind = Literal[
    "presence", "horizontal_direction", "within_distance", "compare_nearest_by_side"
]


@dataclass(frozen=True)
class QuestionIntent:
    """A constrained question proposed from an image."""

    kind: QuestionKind
    object_query: str
    threshold_m: float | None = None


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


class VisualQuestionAnswerer(Protocol):
    """Answers a question from an image without access to ground truth."""

    def answer(self, image: Image, question: str) -> str: ...
