# Copyright 2026 Dimensional Inc.
"""Typed private perception primitives over one frozen VQA frame."""

from __future__ import annotations

import json
from typing import Any

from langchain_core.tools import StructuredTool

from dimos.benchmark.vqa.generation.primitives.choices import height_choice_window
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
        evidence = OracleEvidence(
            f"height:v1:{object.id}",
            "v1",
            object.id,
            object.label,
            object.range_m,
            object.horizontal_direction,
            object.point_count,
            measurement,
        )
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
