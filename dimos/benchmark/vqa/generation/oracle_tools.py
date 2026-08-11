# Copyright 2026 Dimensional Inc.
"""Typed private perception primitives over one frozen VQA frame."""

from __future__ import annotations

import json
from typing import Any

from langchain_core.tools import StructuredTool

from dimos.benchmark.vqa.generation.primitives.choices import (
    CAMERA_RANGE_CHOICES,
    COUNT_CHOICES,
    camera_range_choice,
    count_choice,
    height_choice_window,
)
from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
from dimos.benchmark.vqa.generation.primitives.selection import select_nearest_object
from dimos.benchmark.vqa.models import (
    GroundedObject,
    GroundPlaneEstimate,
    OracleEvidence,
    OracleMeasurement,
    OracleToolResult,
)


class LocalOracleToolRegistry:
    """Expose the same private perception primitives used by constrained recipes."""

    def __init__(self, primitives: FramePerceptionPrimitives) -> None:
        self._primitives = primitives
        self._results: list[OracleToolResult] = []
        self._detections: dict[str, str] = {}
        self._masks: dict[str, str] = {}
        self._objects: dict[str, GroundedObject] = {}
        self._planes: dict[str, GroundPlaneEstimate] = {}
        self._measurements: dict[str, OracleToolResult] = {}
        self._next_id = 0

    @property
    def results(self) -> tuple[OracleToolResult, ...]:
        return tuple(self._results)

    def tools(self) -> list[StructuredTool]:
        return [
            StructuredTool.from_function(
                self.detect_objects,
                name="detect_objects",
                description="Run private MoonDream detection for one visible semantic query.",
            ),
            StructuredTool.from_function(
                self.segment_detections,
                name="segment_detections",
                description="Run private EdgeTAM segmentation for one opaque detection ID.",
            ),
            StructuredTool.from_function(
                self.ground_masks,
                name="ground_masks",
                description=(
                    "Project visible calibrated point-cloud support through one opaque mask ID. "
                    "Returns grounded object IDs and citable evidence."
                ),
            ),
            StructuredTool.from_function(
                self.select_nearest_object,
                name="select_nearest_object",
                description=(
                    "Select the nearest opaque grounded object ID, optionally restricted to left, center, "
                    "or right."
                ),
            ),
            StructuredTool.from_function(
                self.count_grounded_objects,
                name="count_grounded_objects",
                description="Count opaque grounded object IDs into fixed public count buckets.",
            ),
            StructuredTool.from_function(
                self.bucket_camera_range,
                name="bucket_camera_range",
                description="Bucket one opaque object's private camera-origin range into fixed public choices.",
            ),
            StructuredTool.from_function(
                self.compare_nearest_by_side,
                name="compare_nearest_by_side",
                description="Choose whether the nearest opaque left or right object is closer to the camera.",
            ),
            StructuredTool.from_function(
                self.compare_left_right,
                name="compare_left_right",
                description=(
                    "Choose whether one opaque object is left or right of another from private "
                    "camera-frame support centroids. Rejects ambiguous separation."
                ),
            ),
            StructuredTool.from_function(
                self.select_closest_object,
                name="select_closest_object",
                description=(
                    "Select which opaque candidate object is closest to one opaque target object "
                    "by private point-cloud support. Rejects ambiguous proximity."
                ),
            ),
            StructuredTool.from_function(
                self.fit_ground_plane,
                name="fit_ground_plane",
                description="Fit a quality-gated Open3D ground plane to the frozen visible point cloud.",
            ),
            StructuredTool.from_function(
                self.measure_height,
                name="measure_height",
                description=(
                    "Measure one opaque grounded object above one opaque accepted ground-plane ID."
                ),
            ),
            StructuredTool.from_function(
                self.compare_heights,
                name="compare_heights",
                description=(
                    "Compare two opaque grounded objects against one opaque accepted ground-plane ID. "
                    "Rejects overlapping physical-height uncertainty."
                ),
            ),
            StructuredTool.from_function(
                self.classify_door_state,
                name="classify_door_state",
                description=(
                    "Classify one opaque grounded door as open or closed from point-cloud planes. "
                    "Rejects insufficient or ambiguous geometry."
                ),
            ),
            StructuredTool.from_function(
                self.classify_forward_path,
                name="classify_forward_path",
                description=(
                    "Classify the visible camera-forward corridor as clear or blocked from point-cloud "
                    "ground and obstacle support. Rejects incomplete or ambiguous visibility."
                ),
            ),
            StructuredTool.from_function(
                self.bucket_measurement,
                name="bucket_measurement",
                description=(
                    "Map one opaque height measurement ID to four public, answer-conditioned "
                    "height choices and the one matching choice."
                ),
            ),
        ]

    def detect_objects(self, query: str) -> str:
        """Detect objects and return an opaque ID for a later segmentation call."""
        detections = self._primitives.detect_objects(query)
        detection_id = self._id("detection")
        self._detections[detection_id] = query
        result = OracleToolResult("detect_objects", query, ())
        self._results.append(result)
        boxes = [list(item.bbox) for item in detections]
        return json.dumps(_tool_payload(result, detection_id=detection_id, boxes=boxes))

    def segment_detections(self, detection_id: str) -> str:
        """Segment one earlier opaque detection result and return an opaque mask ID."""
        query = self._detections.get(detection_id)
        if query is None:
            return self._record_rejection("segment_detections", "", [], "unknown_detection_id")
        masks = self._primitives.segment_detections(query)
        mask_id = self._id("mask")
        self._masks[mask_id] = query
        result = OracleToolResult("segment_detections", query, ())
        self._results.append(result)
        return json.dumps(_tool_payload(result, mask_id=mask_id, mask_count=len(masks)))

    def ground_masks(self, mask_id: str) -> str:
        """Ground one earlier opaque mask result against the visible point cloud."""
        query = self._masks.get(mask_id)
        if query is None:
            return self._record_rejection("ground_masks", "", [], "unknown_mask_id")
        objects = self._primitives.ground_masks(query)
        evidence = tuple(_grounding_evidence(item) for item in objects)
        for item in objects:
            self._objects[item.id] = item
        result = OracleToolResult("ground_masks", query, evidence)
        self._results.append(result)
        return json.dumps(_tool_payload(result, object_ids=[item.id for item in objects]))

    def fit_ground_plane(self) -> str:
        """Fit the private ground plane and return an opaque ID for measurements."""
        fit = self._primitives.fit_ground_plane()
        if fit.estimate is None:
            return self._record_rejection(
                "fit_ground_plane", "", list(fit.quality_flags), fit.rejection_reason
            )
        plane_id = self._id("plane")
        self._planes[plane_id] = fit.estimate
        measurement = OracleMeasurement(
            fit.estimate.offset_m,
            "m",
            max(fit.estimate.residual_m, 0.01),
            fit.quality_flags,
            (f"frame:{self._primitives.frame.id}",),
        )
        evidence = OracleEvidence(
            f"ground-plane:v1:{self._primitives.frame.id}",
            "v1",
            "ground-plane",
            "ground",
            0.0,
            "n/a",
            fit.estimate.inlier_count,
            measurement,
        )
        result = OracleToolResult(
            "fit_ground_plane",
            "",
            (evidence,),
            measurement=measurement,
            plane=fit.estimate,
            quality_flags=fit.quality_flags,
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result, plane_id=plane_id))

    def select_nearest_object(self, object_ids: list[str], side: str | None = None) -> str:
        """Select one opaque grounded object by private point-cloud range."""
        selected: list[GroundedObject] = []
        for object_id in object_ids:
            item = self._objects.get(object_id)
            if item is None:
                return self._record_rejection(
                    "select_nearest_object", object_id, [], "unknown_object_id"
                )
            selected.append(item)
        nearest = select_nearest_object(selected, side)
        if nearest is None:
            return self._record_rejection("select_nearest_object", "", [], "no_object_matches_side")
        result = OracleToolResult(
            "select_nearest_object", nearest.label, (_grounding_evidence(nearest),)
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result, object_id=nearest.id))

    def count_grounded_objects(self, object_ids: list[str]) -> str:
        """Count unique grounded object IDs into fixed public count choices."""
        objects = self._lookup_objects("count_grounded_objects", object_ids)
        if objects is None:
            return self._record_rejection(
                "count_grounded_objects", "", [], "unknown_or_duplicate_object_id"
            )
        if not objects:
            return self._record_rejection("count_grounded_objects", "", [], "no_grounded_object")
        result = OracleToolResult(
            "count_grounded_objects",
            objects[0].label,
            tuple(_grounding_evidence(item) for item in objects),
            choice=count_choice(len(objects)),
            choices=COUNT_CHOICES,
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result))

    def bucket_camera_range(self, object_id: str) -> str:
        """Map one grounded object's camera-origin range into fixed public choices."""
        object = self._objects.get(object_id)
        if object is None:
            return self._record_rejection("bucket_camera_range", object_id, [], "unknown_object_id")
        measurement = OracleMeasurement(
            object.range_m,
            "m",
            0.0,
            ("camera_origin_euclidean_range",),
            (f"grounding:v1:{object.id}",),
        )
        result = OracleToolResult(
            "bucket_camera_range",
            object.label,
            (_grounding_evidence(object),),
            measurement=measurement,
            choice=camera_range_choice(object.range_m),
            choices=CAMERA_RANGE_CHOICES,
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result))

    def compare_nearest_by_side(self, object_ids: list[str]) -> str:
        """Compare the nearest left and right grounded objects by camera range."""
        objects = self._lookup_objects("compare_nearest_by_side", object_ids)
        if objects is None:
            return self._record_rejection(
                "compare_nearest_by_side", "", [], "unknown_or_duplicate_object_id"
            )
        left = select_nearest_object(objects, "left")
        right = select_nearest_object(objects, "right")
        if left is None or right is None:
            return self._record_rejection(
                "compare_nearest_by_side", "", [], "missing_grounded_side"
            )
        if left.range_m == right.range_m:
            return self._record_rejection(
                "compare_nearest_by_side", "", [], "ambiguous_nearest_by_side"
            )
        choice = "left" if left.range_m < right.range_m else "right"
        result = OracleToolResult(
            "compare_nearest_by_side",
            left.label,
            (_grounding_evidence(left), _grounding_evidence(right)),
            choice=choice,
            choices=("left", "right"),
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result, left_object_id=left.id, right_object_id=right.id))

    def compare_left_right(self, first_object_id: str, second_object_id: str) -> str:
        """Classify one grounded object's left/right relation to another grounded object."""
        first = self._objects.get(first_object_id)
        second = self._objects.get(second_object_id)
        if first is None or second is None:
            return self._record_rejection("compare_left_right", "", [], "unknown_object_id")
        relation = self._primitives.classify_horizontal_relation(first, second)
        if relation.relation is None:
            return self._record_rejection(
                "compare_left_right",
                f"{first.label},{second.label}",
                list(relation.quality_flags),
                relation.rejection_reason,
            )
        result = OracleToolResult(
            "compare_left_right",
            f"{first.label},{second.label}",
            (_grounding_evidence(first), _grounding_evidence(second)),
            choice=relation.relation,
            choices=("left", "right"),
            quality_flags=relation.quality_flags,
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result))

    def select_closest_object(self, target_id: str, candidate_ids: list[str]) -> str:
        """Select one candidate closest to a target by private 3D support-point proximity."""
        target = self._objects.get(target_id)
        if target is None:
            return self._record_rejection(
                "select_closest_object", target_id, [], "unknown_target_id"
            )
        candidates: list[GroundedObject] = []
        for candidate_id in candidate_ids:
            candidate = self._objects.get(candidate_id)
            if candidate is None:
                return self._record_rejection(
                    "select_closest_object", candidate_id, [], "unknown_candidate_id"
                )
            candidates.append(candidate)
        selected = self._primitives.select_closest_object(target, candidates)
        if selected.object is None:
            return self._record_rejection(
                "select_closest_object",
                target.label,
                list(selected.quality_flags),
                selected.rejection_reason,
            )
        result = OracleToolResult(
            "select_closest_object",
            target.label,
            (_grounding_evidence(target), _grounding_evidence(selected.object)),
            choice=selected.object.label,
            quality_flags=selected.quality_flags,
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result, object_id=selected.object.id))

    def measure_height(self, object_id: str, plane_id: str) -> str:
        """Measure one grounded object against one previously accepted plane."""
        object = self._objects.get(object_id)
        plane = self._planes.get(plane_id)
        if object is None:
            return self._record_rejection("measure_height", object_id, [], "unknown_object_id")
        if plane is None:
            return self._record_rejection("measure_height", object_id, [], "unknown_plane_id")
        measured = self._primitives.measure_height(object, plane)
        if measured.measurement is None:
            return self._record_rejection(
                "measure_height",
                object.label,
                list(measured.quality_flags),
                measured.rejection_reason,
            )
        measurement = measured.measurement
        evidence = _height_evidence(object, measurement)
        result = OracleToolResult(
            "measure_height",
            object.label,
            (evidence,),
            measurement=measurement,
            plane=plane,
            quality_flags=measured.quality_flags,
        )
        measurement_id = self._id("measurement")
        self._measurements[measurement_id] = result
        self._results.append(result)
        return json.dumps(_tool_payload(result, measurement_id=measurement_id))

    def compare_heights(self, first_object_id: str, second_object_id: str, plane_id: str) -> str:
        """Choose the taller object only when one shared-plane measurement is unambiguous."""
        first = self._objects.get(first_object_id)
        second = self._objects.get(second_object_id)
        plane = self._planes.get(plane_id)
        if first is None or second is None:
            return self._record_rejection("compare_heights", "", [], "unknown_object_id")
        if first.id == second.id:
            return self._record_rejection("compare_heights", first.label, [], "duplicate_object_id")
        if plane is None:
            return self._record_rejection("compare_heights", "", [], "unknown_plane_id")
        first_height = self._primitives.measure_height(first, plane)
        second_height = self._primitives.measure_height(second, plane)
        if first_height.measurement is None or second_height.measurement is None:
            return self._record_rejection(
                "compare_heights",
                "",
                [*first_height.quality_flags, *second_height.quality_flags],
                first_height.rejection_reason
                or second_height.rejection_reason
                or "height_measurement_rejected",
            )
        first_measurement = first_height.measurement
        second_measurement = second_height.measurement
        first_lower = first_measurement.value - first_measurement.tolerance
        second_lower = second_measurement.value - second_measurement.tolerance
        first_upper = first_measurement.value + first_measurement.tolerance
        second_upper = second_measurement.value + second_measurement.tolerance
        if first_lower <= second_upper and second_lower <= first_upper:
            return self._record_rejection("compare_heights", "", [], "ambiguous_height_comparison")
        choice = first.label if first_lower > second_upper else second.label
        result = OracleToolResult(
            "compare_heights",
            f"{first.label},{second.label}",
            (
                _height_evidence(first, first_measurement),
                _height_evidence(second, second_measurement),
            ),
            choice=choice,
            choices=(first.label, second.label),
            plane=plane,
            quality_flags=(*first_height.quality_flags, *second_height.quality_flags),
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result))

    def classify_door_state(self, object_id: str) -> str:
        """Classify one grounded door against the surrounding point-cloud plane."""
        object = self._objects.get(object_id)
        if object is None:
            return self._record_rejection("classify_door_state", object_id, [], "unknown_object_id")
        if "door" not in object.label.lower():
            return self._record_rejection(
                "classify_door_state", object.label, [], "door_state_requires_door_query"
            )
        result = self._primitives.classify_door_state(object)
        if result.state is None:
            return self._record_rejection(
                "classify_door_state",
                object.label,
                list(result.quality_flags),
                result.rejection_reason,
            )
        evidence = OracleEvidence(
            f"door-state:v1:{object.id}",
            "v1",
            object.id,
            object.label,
            object.range_m,
            object.horizontal_direction,
            object.point_count,
        )
        tool_result = OracleToolResult(
            "classify_door_state",
            object.label,
            (evidence,),
            choice=result.state,
            quality_flags=result.quality_flags,
        )
        self._results.append(tool_result)
        return json.dumps(_tool_payload(tool_result))

    def classify_forward_path(self) -> str:
        """Classify the observed local corridor directly ahead of the camera."""
        result = self._primitives.classify_forward_path()
        if result.state is None:
            return self._record_rejection(
                "classify_forward_path",
                "",
                list(result.quality_flags),
                result.rejection_reason,
            )
        evidence = OracleEvidence(
            f"forward-path:v1:{self._primitives.frame.id}",
            "v1",
            "forward-path",
            "forward path",
            0.0,
            "center",
            result.point_count,
        )
        tool_result = OracleToolResult(
            "classify_forward_path",
            "forward path",
            (evidence,),
            choice=result.state,
            quality_flags=result.quality_flags,
        )
        self._results.append(tool_result)
        return json.dumps(_tool_payload(tool_result))

    def bucket_measurement(self, measurement_id: str) -> str:
        """Map one accepted private height measurement to its public choice."""
        source = self._measurements.get(measurement_id)
        if source is None or source.measurement is None:
            return self._record_rejection("bucket_measurement", "", [], "unknown_measurement_id")
        choices, choice = height_choice_window(source.measurement.value)
        result = OracleToolResult(
            "bucket_measurement",
            source.query,
            source.evidence,
            measurement=source.measurement,
            choice=choice,
            choices=choices,
            plane=source.plane,
            quality_flags=source.quality_flags,
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result))

    def _record_rejection(self, tool: str, query: str, flags: list[str], reason: str | None) -> str:
        result = OracleToolResult(
            tool, query, (), quality_flags=tuple(flags), rejection_reason=reason
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result))

    def _lookup_objects(self, tool: str, object_ids: list[str]) -> list[GroundedObject] | None:
        if len(set(object_ids)) != len(object_ids):
            return None
        objects: list[GroundedObject] = []
        for object_id in object_ids:
            object = self._objects.get(object_id)
            if object is None:
                return None
            objects.append(object)
        return objects

    def _id(self, kind: str) -> str:
        self._next_id += 1
        return f"{kind}:v1:{self._next_id:04d}"


def _grounding_evidence(item: GroundedObject) -> OracleEvidence:
    return OracleEvidence(
        f"grounding:v1:{item.id}",
        "v1",
        item.id,
        item.label,
        item.range_m,
        item.horizontal_direction,
        item.point_count,
    )


def _height_evidence(item: GroundedObject, measurement: OracleMeasurement) -> OracleEvidence:
    return OracleEvidence(
        f"height:v1:{item.id}",
        "v1",
        item.id,
        item.label,
        item.range_m,
        item.horizontal_direction,
        item.point_count,
        measurement,
    )


def _tool_payload(result: OracleToolResult, **identifiers: Any) -> dict[str, Any]:
    return {
        "tool": result.tool,
        "query": result.query,
        "version": result.version,
        "measurement": (
            {
                "value": result.measurement.value,
                "unit": result.measurement.unit,
                "tolerance": result.measurement.tolerance,
                "quality_flags": result.measurement.quality_flags,
                "provenance_ids": result.measurement.provenance_ids,
            }
            if result.measurement is not None
            else None
        ),
        "choice": result.choice,
        "choices": result.choices,
        "quality_flags": result.quality_flags,
        "rejection_reason": result.rejection_reason,
        "plane": (
            {
                "normal": result.plane.normal,
                "offset_m": result.plane.offset_m,
                "sample_count": result.plane.sample_count,
                "inlier_count": result.plane.inlier_count,
                "residual_m": result.plane.residual_m,
            }
            if result.plane is not None
            else None
        ),
        "objects": [_evidence_payload(item) for item in result.evidence],
        **identifiers,
    }


def _evidence_payload(item: OracleEvidence) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "evidence_id": item.id,
        "id": item.object_id,
        "label": item.label,
        "range_m": item.range_m,
        "side": item.side,
        "point_count": item.point_count,
    }
    if item.measurement is not None:
        payload["measurement"] = {
            "value": item.measurement.value,
            "unit": item.measurement.unit,
            "tolerance": item.measurement.tolerance,
            "quality_flags": item.measurement.quality_flags,
            "provenance_ids": item.measurement.provenance_ids,
        }
    return payload
