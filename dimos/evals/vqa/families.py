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

"""Specifications and answer rules for deterministic VQA question families."""

from __future__ import annotations

from typing import TYPE_CHECKING, cast

from pydantic import JsonValue

from dimos.evals.vqa.contracts import (
    FamilyAnswer,
    FamilyName,
    FamilySpec,
    InsufficientEvidenceError,
    ObjectMaskEstimator,
    ObjectRangeEstimator,
    QuestionProposal,
)

if TYPE_CHECKING:
    from dimos.evals.vqa.pointcloud_frame import PointCloudFrame
    from dimos.models.vl.base import VlModel
    from dimos.msgs.sensor_msgs.Image import Image


PRESENCE_FAMILY = FamilySpec(
    name="presence",
    description="Ask whether a clearly visible object category is present.",
)
HORIZONTAL_DIRECTION_FAMILY = FamilySpec(
    name="horizontal_direction",
    description="Use only for one clearly visible object instance.",
)
OBJECT_COUNT_FAMILY = FamilySpec(
    name="object_count",
    description="Count a small number of clearly visible, distinct object instances.",
)
IMAGE_COVERAGE_FAMILY = FamilySpec(
    name="image_coverage",
    description=(
        "Estimate how much of the image one clearly visible object occupies from its segmentation "
        "mask. Do not estimate the percentage yourself."
    ),
    requires_masks=True,
)
LARGEST_VISIBLE_AREA_FAMILY = FamilySpec(
    name="largest_visible_area",
    description=(
        "Choose which of two to five distinct, clearly visible object references occupies the "
        "largest segmented image area. Do not infer which reference is largest."
    ),
    min_objects=2,
    max_objects=5,
    distinct_objects=True,
    requires_masks=True,
    object_order_matters=False,
)
OBJECT_DISTANCE_FAMILY = FamilySpec(
    name="object_distance",
    description="Estimate the range to one clearly visible object instance.",
    requires_pointcloud=True,
)
CLOSEST_OBJECT_FAMILY = FamilySpec(
    name="closest_object",
    description=(
        "Choose the closest of two to five distinct object references with exactly one visible "
        "match each. Spatial descriptions such as 'left person' and 'right person' may distinguish "
        "instances of one category. Do not infer which reference is closest."
    ),
    min_objects=2,
    max_objects=5,
    distinct_objects=True,
    requires_pointcloud=True,
    object_order_matters=False,
)
AVAILABLE_FAMILIES = (
    PRESENCE_FAMILY,
    HORIZONTAL_DIRECTION_FAMILY,
    OBJECT_COUNT_FAMILY,
    IMAGE_COVERAGE_FAMILY,
    LARGEST_VISIBLE_AREA_FAMILY,
    OBJECT_DISTANCE_FAMILY,
    CLOSEST_OBJECT_FAMILY,
)

_FAMILIES_BY_NAME = {family.name: family for family in AVAILABLE_FAMILIES}


def _family_spec(name: FamilyName) -> FamilySpec:
    return _FAMILIES_BY_NAME[name]


def answer_question(
    proposal: QuestionProposal,
    image: Image,
    detector: VlModel,
    pointcloud_frame: PointCloudFrame | None = None,
    range_estimator: ObjectRangeEstimator | None = None,
    mask_estimator: ObjectMaskEstimator | None = None,
) -> FamilyAnswer:
    """Dispatch one constrained proposal to its deterministic family."""
    _family_spec(proposal.family).validate(proposal)
    if proposal.family == "presence":
        return _answer_presence(proposal, image, detector)
    if proposal.family == "horizontal_direction":
        return _answer_horizontal_direction(proposal, image, detector)
    if proposal.family == "object_count":
        return _answer_object_count(proposal, image, detector)
    if proposal.family == "image_coverage":
        if mask_estimator is None:
            raise InsufficientEvidenceError("image coverage requires segmentation-mask evidence")
        return _answer_image_coverage(proposal, image, mask_estimator)
    if proposal.family == "largest_visible_area":
        if mask_estimator is None:
            raise InsufficientEvidenceError(
                "largest visible area requires segmentation-mask evidence"
            )
        return _answer_largest_visible_area(proposal, image, mask_estimator)
    if proposal.family == "object_distance":
        if pointcloud_frame is None or range_estimator is None:
            raise InsufficientEvidenceError(
                "object distance requires image-aligned point-cloud evidence"
            )
        return _answer_object_distance(proposal, pointcloud_frame, range_estimator)
    if proposal.family == "closest_object":
        if pointcloud_frame is None or range_estimator is None:
            raise InsufficientEvidenceError(
                "closest object requires image-aligned point-cloud evidence"
            )
        return _answer_closest_object(proposal, pointcloud_frame, range_estimator)
    raise ValueError(f"unsupported VQA family: {proposal.family}")


def _answer_presence(proposal: QuestionProposal, image: Image, detector: VlModel) -> FamilyAnswer:
    object_name = proposal.object_names[0]
    detections = detector.query_detections(image, object_name)
    if not detections.detections:
        raise InsufficientEvidenceError(f"object detector did not confirm visible {object_name!r}")
    boxes = [list(map(float, detection.bbox)) for detection in detections.detections]
    return FamilyAnswer(
        question=f"Does the image contain any {object_name}?",
        choices=("yes", "no"),
        answer="yes",
        evidence={
            "primitive": "object_detector",
            "object_name": object_name,
            "detection_count": len(detections),
            "boxes": cast("JsonValue", boxes),
        },
    )


def _answer_horizontal_direction(
    proposal: QuestionProposal, image: Image, detector: VlModel
) -> FamilyAnswer:
    object_name = proposal.object_names[0]
    detections = detector.query_detections(image, object_name)
    if len(detections) != 1:
        raise InsufficientEvidenceError(
            f"horizontal direction requires exactly one detected {object_name!r}, "
            f"got {len(detections)}"
        )

    box = detections.detections[0].bbox
    center_x = (box[0] + box[2]) / 2
    center_fraction = center_x / image.width
    if center_fraction < 1 / 3:
        answer = "left"
    elif center_fraction < 2 / 3:
        answer = "center"
    else:
        answer = "right"
    return FamilyAnswer(
        question=f"Which horizontal region contains the visible {object_name}?",
        choices=("left", "center", "right"),
        answer=answer,
        evidence={
            "primitive": "object_detector",
            "object_name": object_name,
            "detection_count": 1,
            "box": cast("JsonValue", list(map(float, box))),
            "center_x_px": center_x,
            "center_x_fraction": center_fraction,
        },
    )


def _answer_object_count(
    proposal: QuestionProposal, image: Image, detector: VlModel
) -> FamilyAnswer:
    object_name = proposal.object_names[0]
    detections = detector.query_detections(image, object_name)
    count = len(detections)
    if count == 0:
        raise InsufficientEvidenceError(
            f"object detector did not confirm visible {object_name!r} to count"
        )

    choices = ("one", "two", "three", "four or more")
    answer = choices[min(count, 4) - 1]
    boxes = [list(map(float, detection.bbox)) for detection in detections.detections]
    return FamilyAnswer(
        question=f"How many instances of {object_name} are visible in the image?",
        choices=choices,
        answer=answer,
        evidence={
            "primitive": "object_detector",
            "object_name": object_name,
            "detection_count": count,
            "boxes": cast("JsonValue", boxes),
        },
    )


def _answer_object_distance(
    proposal: QuestionProposal,
    pointcloud_frame: PointCloudFrame,
    range_estimator: ObjectRangeEstimator,
) -> FamilyAnswer:
    """Answer an object-distance proposal from EdgeTAM and point-cloud evidence."""
    object_name = proposal.object_names[0]
    evidence = range_estimator.estimate(pointcloud_frame, object_name)
    choices = (
        "under 1 meter",
        "1 to under 2 meters",
        "2 to under 3 meters",
        "3 meters or more",
    )
    lower_quartile, _, upper_quartile = evidence.range_quartiles_m
    if _distance_bucket(lower_quartile) != _distance_bucket(upper_quartile):
        raise InsufficientEvidenceError(
            "object range uncertainty crosses a distance answer boundary"
        )
    answer = choices[_distance_bucket(evidence.camera_range_m)]
    return FamilyAnswer(
        question=f"Approximately how far is the visible {object_name} from the camera?",
        choices=choices,
        answer=answer,
        evidence={
            "primitive": "edge_tam_lidar_range",
            **evidence.model_dump(mode="json"),
        },
    )


_COVERAGE_SCHEMES = (
    (
        (25.0, 50.0, 75.0),
        ("under 25%", "25% to under 50%", "50% to under 75%", "75% or more"),
    ),
    (
        (15.0, 35.0, 65.0, 85.0),
        (
            "under 15%",
            "15% to under 35%",
            "35% to under 65%",
            "65% to under 85%",
            "85% or more",
        ),
    ),
)


def _answer_image_coverage(
    proposal: QuestionProposal,
    image: Image,
    mask_estimator: ObjectMaskEstimator,
) -> FamilyAnswer:
    object_name = proposal.object_names[0]
    evidence = mask_estimator.estimate(image, object_name)
    image_area = image.width * image.height
    coverage = 100.0 * evidence.mask_area_px / image_area
    boundaries, choices = max(
        _COVERAGE_SCHEMES,
        key=lambda scheme: min(abs(coverage - boundary) for boundary in scheme[0]),
    )
    boundary_margin = min(abs(coverage - boundary) for boundary in boundaries)
    answer_index = sum(coverage >= boundary for boundary in boundaries)
    return FamilyAnswer(
        question=(
            f"Approximately what percentage of the image is occupied by the visible {object_name}?"
        ),
        choices=choices,
        answer=choices[answer_index],
        evidence={
            "primitive": "edge_tam_mask_area",
            "object_name": object_name,
            "mask_area_px": evidence.mask_area_px,
            "image_area_px": image_area,
            "coverage_percent": coverage,
            "bucket_boundaries_percent": cast("JsonValue", list(boundaries)),
            "nearest_boundary_margin_percent": boundary_margin,
            "prompt_bbox_xyxy": cast("JsonValue", list(evidence.prompt_bbox_xyxy)),
            "mask_bbox_xyxy": cast("JsonValue", list(evidence.mask_bbox_xyxy)),
        },
    )


def _answer_largest_visible_area(
    proposal: QuestionProposal,
    image: Image,
    mask_estimator: ObjectMaskEstimator,
) -> FamilyAnswer:
    object_names = proposal.object_names
    masks = mask_estimator.estimate_many(image, object_names)
    if tuple(mask.object_name for mask in masks) != object_names:
        raise ValueError("mask estimator results do not match the requested object order")
    ranked = sorted(range(len(masks)), key=lambda index: masks[index].mask_area_px, reverse=True)
    winner_index, runner_up_index = ranked[:2]
    winner_area = masks[winner_index].mask_area_px
    runner_up_area = masks[runner_up_index].mask_area_px
    if 5 * winner_area < 6 * runner_up_area:
        raise InsufficientEvidenceError(
            "visible mask areas do not establish an object that is at least 20% larger"
        )
    image_area = image.width * image.height
    return FamilyAnswer(
        question="Which object occupies the largest visible area in the image?",
        choices=object_names,
        answer=object_names[winner_index],
        evidence={
            "primitive": "edge_tam_mask_area",
            "comparison_rule": "winner_at_least_20_percent_larger",
            "objects": cast(
                "JsonValue",
                [
                    {
                        "choice": object_name,
                        "mask_area_px": mask.mask_area_px,
                        "coverage_percent": 100.0 * mask.mask_area_px / image_area,
                        "prompt_bbox_xyxy": list(mask.prompt_bbox_xyxy),
                        "mask_bbox_xyxy": list(mask.mask_bbox_xyxy),
                    }
                    for object_name, mask in zip(object_names, masks, strict=True)
                ],
            ),
        },
    )


def _answer_closest_object(
    proposal: QuestionProposal,
    pointcloud_frame: PointCloudFrame,
    range_estimator: ObjectRangeEstimator,
) -> FamilyAnswer:
    """Choose the closest named object when its range interval is unambiguous."""
    object_names = proposal.object_names
    ranges = range_estimator.estimate_many(pointcloud_frame, object_names)
    if len(ranges) != len(object_names):
        raise ValueError("range estimator returned the wrong number of object ranges")
    if tuple(evidence.object_name for evidence in ranges) != object_names:
        raise ValueError("range estimator results do not match the requested object order")

    winner_index = min(range(len(ranges)), key=lambda index: ranges[index].camera_range_m)
    winner = ranges[winner_index]
    winner_upper = winner.range_quartiles_m[2]
    if any(
        winner_upper >= evidence.range_quartiles_m[0]
        for index, evidence in enumerate(ranges)
        if index != winner_index
    ):
        raise InsufficientEvidenceError(
            "object range uncertainty does not establish a closest object"
        )

    return FamilyAnswer(
        question="Which object is closest to the camera?",
        choices=object_names,
        answer=object_names[winner_index],
        evidence={
            "primitive": "edge_tam_lidar_closest_range",
            "comparison_rule": "closest_non_overlapping_interquartile_range",
            "objects": cast(
                "JsonValue",
                [
                    {"choice": object_name, **evidence.model_dump(mode="json")}
                    for object_name, evidence in zip(object_names, ranges, strict=True)
                ],
            ),
        },
    )


def _distance_bucket(distance_m: float) -> int:
    if distance_m < 1:
        return 0
    if distance_m < 2:
        return 1
    if distance_m < 3:
        return 2
    return 3
