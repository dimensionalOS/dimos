# Copyright 2026 Dimensional Inc.
"""Agentic adapters for object detection, segmentation, and grounding primitives."""

from __future__ import annotations

import json
from typing import Any

from langchain_core.tools import StructuredTool

from dimos.benchmark.vqa.contracts import (
    GroundedObject,
    OracleEvidence,
    OracleToolResult,
    VisualObject,
)
from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg


class VqaPrimitiveToolRegistry:
    """Expose opaque handles for the core frame-scoped perception primitives."""

    def __init__(self, primitives: FramePerceptionPrimitives) -> None:
        self._primitives = primitives
        self._results: list[OracleToolResult] = []
        self._detections: dict[str, tuple[str, int]] = {}
        self._masks: dict[str, tuple[str, Detection2DSeg]] = {}
        self._objects: dict[str, GroundedObject] = {}
        self._next_id = 0

    @property
    def results(self) -> tuple[OracleToolResult, ...]:
        return tuple(self._results)

    def tools(self) -> list[StructuredTool]:
        return [
            StructuredTool.from_function(
                self.detect_objects,
                name="detect_objects",
                description="Detect visible objects for one semantic query and return opaque IDs.",
            ),
            StructuredTool.from_function(
                self.segment_object,
                name="segment_object",
                description="Segment one opaque detection ID and return opaque mask IDs.",
            ),
            StructuredTool.from_function(
                self.ground_mask,
                name="ground_mask",
                description="Associate one opaque mask ID with calibrated point-cloud support.",
            ),
            StructuredTool.from_function(
                self.get_object_position,
                name="get_object_position",
                description="Return camera range and horizontal side for one grounded object ID.",
            ),
        ]

    def detect_objects(self, query: str) -> str:
        """Detect objects and return opaque IDs for later segmentation."""
        detections = self._primitives.detect_objects(query)
        visual_objects = self._primitives.visual_objects(query)
        result = OracleToolResult(
            "detect_objects", query, tuple(_visual_evidence(item) for item in visual_objects)
        )
        self._results.append(result)
        handles = []
        for index, detection in enumerate(detections):
            detection_id = self._id("detection")
            self._detections[detection_id] = (query, index)
            handles.append({"detection_id": detection_id, "confidence": detection.confidence})
        return json.dumps(_tool_payload(result, detections=handles))

    def segment_object(self, detection_id: str) -> str:
        """Segment one earlier opaque detection and return opaque mask IDs."""
        handle = self._detections.get(detection_id)
        if handle is None:
            return self._record_rejection("segment_object", "", "unknown_detection_id")
        query, index = handle
        masks = self._primitives.segment_detection(query, index)
        mask_ids = []
        for mask in masks:
            mask_id = self._id("mask")
            self._masks[mask_id] = (query, mask)
            mask_ids.append(mask_id)
        result = OracleToolResult("segment_object", query, ())
        if not mask_ids:
            result = OracleToolResult(
                "segment_object", query, (), rejection_reason="no_segmentation_mask"
            )
        self._results.append(result)
        return json.dumps(_tool_payload(result, mask_ids=mask_ids))

    def ground_mask(self, mask_id: str) -> str:
        """Ground one earlier opaque mask against the visible point cloud."""
        handle = self._masks.get(mask_id)
        if handle is None:
            return self._record_rejection("ground_mask", "", "unknown_mask_id")
        query, mask = handle
        object = self._primitives.ground_mask(mask)
        if object is None:
            return self._record_rejection("ground_mask", query, "insufficient_foreground_support")
        self._objects[object.id] = object
        result = OracleToolResult("ground_mask", query, (_grounding_evidence(object),))
        self._results.append(result)
        return json.dumps(_tool_payload(result, object_id=object.id))

    def get_object_position(self, object_id: str) -> str:
        """Return camera-relative position evidence for one grounded object."""
        object = self._objects.get(object_id)
        if object is None:
            return self._record_rejection("get_object_position", object_id, "unknown_object_id")
        positioned = self._primitives.get_object_position(object)
        result = OracleToolResult(
            "get_object_position", positioned.label, (_grounding_evidence(positioned),)
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result, object_id=positioned.id))

    def _record_rejection(self, tool: str, query: str, reason: str) -> str:
        result = OracleToolResult(tool, query, (), rejection_reason=reason)
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


def _visual_evidence(item: VisualObject) -> OracleEvidence:
    return OracleEvidence(
        f"visual:v1:{item.id}",
        "v1",
        item.id,
        item.label,
        None,
        item.horizontal_direction,
        None,
        item.bbox,
    )


def _tool_payload(result: OracleToolResult, **identifiers: Any) -> dict[str, Any]:
    return {
        "tool": result.tool,
        "query": result.query,
        "version": result.version,
        "quality_flags": result.quality_flags,
        "rejection_reason": result.rejection_reason,
        "objects": [_evidence_payload(item) for item in result.evidence],
        **identifiers,
    }


def _evidence_payload(item: OracleEvidence) -> dict[str, Any]:
    return {
        "evidence_id": item.id,
        "id": item.object_id,
        "label": item.label,
        "range_m": item.range_m,
        "side": item.side,
        "point_count": item.point_count,
        "bbox": item.bbox,
    }
