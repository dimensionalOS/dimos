# Copyright 2026 Dimensional Inc.
"""Typed private perception primitives over one frozen VQA frame."""

from __future__ import annotations

from dataclasses import replace
import json
from typing import Any

from langchain_core.tools import StructuredTool

from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
from dimos.benchmark.vqa.generation.primitives.geometry import ForwardCorridorMeasurement
from dimos.benchmark.vqa.models import (
    GroundedObject,
    GroundPlaneEstimate,
    OracleEvidence,
    OracleMeasurement,
    OracleToolResult,
)
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg


class LocalOracleToolRegistry:
    """Expose the same private perception primitives used by constrained recipes."""

    def __init__(self, primitives: FramePerceptionPrimitives) -> None:
        self._primitives = primitives
        self._results: list[OracleToolResult] = []
        self._detections: dict[str, tuple[str, int]] = {}
        self._masks: dict[str, tuple[str, Detection2DSeg]] = {}
        self._objects: dict[str, GroundedObject] = {}
        self._planes: dict[str, GroundPlaneEstimate] = {}
        self._ground_planes: set[str] = set()
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
                self.segment_detection,
                name="segment_detection",
                description="Run private EdgeTAM segmentation for one opaque detection ID.",
            ),
            StructuredTool.from_function(
                self.ground_mask,
                name="ground_mask",
                description=(
                    "Project visible calibrated point-cloud support through one opaque mask ID."
                ),
            ),
            StructuredTool.from_function(
                self.fit_ground_plane,
                name="fit_ground_plane",
                description="Fit a quality-gated Open3D ground plane to the frozen visible point cloud.",
            ),
            StructuredTool.from_function(
                self.get_object_pose,
                name="get_object_pose",
                description="Return robust private camera-frame position evidence for one grounded object.",
            ),
            StructuredTool.from_function(
                self.fit_object_surface_plane,
                name="fit_object_surface_plane",
                description="Fit a private surface plane to one grounded object's visible support.",
            ),
            StructuredTool.from_function(
                self.fit_mask_surrounding_plane,
                name="fit_mask_surrounding_plane",
                description="Fit a private structural plane from the visible ring around one mask.",
            ),
            StructuredTool.from_function(
                self.measure_object_pair_distance,
                name="measure_object_pair_distance",
                description="Measure private 3D support-centroid distance between two grounded objects.",
            ),
            StructuredTool.from_function(
                self.measure_relative_plane_angle,
                name="measure_relative_plane_angle",
                description="Measure the unsigned angle between two accepted opaque plane IDs.",
            ),
            StructuredTool.from_function(
                self.measure_object_plane_relation,
                name="measure_object_plane_relation",
                description=(
                    "Measure private clearance, contact support, and projected separation of one object "
                    "relative to a support object plane."
                ),
            ),
            StructuredTool.from_function(
                self.measure_aperture_geometry,
                name="measure_aperture_geometry",
                description=(
                    "Measure a selected mask's ground-connected aperture geometry against an accepted "
                    "ground plane."
                ),
            ),
            StructuredTool.from_function(
                self.measure_forward_corridor,
                name="measure_forward_corridor",
                description=(
                    "Measure private ground and elevated-obstacle support in the camera-forward corridor "
                    "against an accepted ground plane."
                ),
            ),
            StructuredTool.from_function(
                self.measure_height,
                name="measure_height",
                description=(
                    "Measure one opaque grounded object above one opaque accepted ground-plane ID."
                ),
            ),
        ]

    def detect_objects(self, query: str) -> str:
        """Detect objects and return an opaque ID for a later segmentation call."""
        detections = self._primitives.detect_objects(query)
        result = OracleToolResult("detect_objects", query, ())
        self._results.append(result)
        handles = []
        for index, detection in enumerate(detections):
            detection_id = self._id("detection")
            self._detections[detection_id] = (query, index)
            handles.append({"detection_id": detection_id, "confidence": detection.confidence})
        return json.dumps(_tool_payload(result, detections=handles))

    def segment_detection(self, detection_id: str) -> str:
        """Segment one earlier opaque detection and return opaque per-mask handles."""
        handle = self._detections.get(detection_id)
        if handle is None:
            return self._record_rejection("segment_detection", "", [], "unknown_detection_id")
        query, index = handle
        masks = self._primitives.segment_detection(query, index)
        mask_ids = []
        for mask in masks:
            mask_id = self._id("mask")
            self._masks[mask_id] = (query, mask)
            mask_ids.append(mask_id)
        result = OracleToolResult("segment_detection", query, ())
        self._results.append(result)
        return json.dumps(_tool_payload(result, mask_ids=mask_ids))

    def ground_mask(self, mask_id: str) -> str:
        """Ground one earlier opaque mask against the visible point cloud."""
        handle = self._masks.get(mask_id)
        if handle is None:
            return self._record_rejection("ground_mask", "", [], "unknown_mask_id")
        query, mask = handle
        object = self._primitives.ground_mask(mask)
        if object is None:
            return self._record_rejection(
                "ground_mask", query, [], "insufficient_foreground_support"
            )
        self._objects[object.id] = object
        result = OracleToolResult("ground_mask", query, (_grounding_evidence(object),))
        self._results.append(result)
        return json.dumps(_tool_payload(result, object_id=object.id))

    def fit_ground_plane(self) -> str:
        """Fit the private ground plane and return an opaque ID for measurements."""
        fit = self._primitives.fit_ground_plane()
        if fit.estimate is None:
            return self._record_rejection(
                "fit_ground_plane", "", list(fit.quality_flags), fit.rejection_reason
            )
        plane_id = self._id("plane")
        self._planes[plane_id] = fit.estimate
        self._ground_planes.add(plane_id)
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

    def get_object_pose(self, object_id: str) -> str:
        """Return existing grounding evidence for one object without selecting an answer."""
        object = self._objects.get(object_id)
        if object is None:
            return self._record_rejection("get_object_pose", object_id, [], "unknown_object_id")
        result = OracleToolResult("get_object_pose", object.label, (_grounding_evidence(object),))
        self._results.append(result)
        return json.dumps(_tool_payload(result, object_id=object.id))

    def fit_object_surface_plane(self, object_id: str) -> str:
        """Fit a plane to one grounded object's visible point support."""
        object = self._objects.get(object_id)
        if object is None:
            return self._record_rejection(
                "fit_object_surface_plane", object_id, [], "unknown_object_id"
            )
        fit = self._primitives.fit_object_surface_plane(object)
        if fit.estimate is None:
            return self._record_rejection(
                "fit_object_surface_plane",
                object.label,
                list(fit.quality_flags),
                fit.rejection_reason,
            )
        plane_id = self._id("plane")
        self._planes[plane_id] = fit.estimate
        result = OracleToolResult(
            "fit_object_surface_plane",
            object.label,
            (_grounding_evidence(object),),
            plane=fit.estimate,
            quality_flags=fit.quality_flags,
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result, plane_id=plane_id))

    def fit_mask_surrounding_plane(self, mask_id: str) -> str:
        """Fit a plane around one selected segmentation mask."""
        handle = self._masks.get(mask_id)
        if handle is None or not isinstance(handle, tuple) or handle[1] is None:
            return self._record_rejection("fit_mask_surrounding_plane", "", [], "unknown_mask_id")
        query, mask = handle
        fit = self._primitives.fit_mask_surrounding_plane(mask)
        if fit.estimate is None:
            return self._record_rejection(
                "fit_mask_surrounding_plane", query, list(fit.quality_flags), fit.rejection_reason
            )
        plane_id = self._id("plane")
        self._planes[plane_id] = fit.estimate
        result = OracleToolResult(
            "fit_mask_surrounding_plane",
            query,
            (),
            plane=fit.estimate,
            quality_flags=fit.quality_flags,
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result, plane_id=plane_id))

    def measure_object_pair_distance(self, first_object_id: str, second_object_id: str) -> str:
        """Measure support-centroid distance between two grounded objects."""
        first, second = self._objects.get(first_object_id), self._objects.get(second_object_id)
        if first is None or second is None:
            return self._record_rejection(
                "measure_object_pair_distance", "", [], "unknown_object_id"
            )
        if first.id == second.id:
            return self._record_rejection(
                "measure_object_pair_distance", first.label, [], "duplicate_object_id"
            )
        measurement = self._primitives.measure_object_pair_distance(first, second)
        if measurement is None:
            return self._record_rejection(
                "measure_object_pair_distance", "", [], "insufficient_object_support"
            )
        result = OracleToolResult(
            "measure_object_pair_distance",
            f"{first.label},{second.label}",
            (_grounding_evidence(first), _grounding_evidence(second)),
            measurement=measurement,
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result))

    def measure_relative_plane_angle(self, first_plane_id: str, second_plane_id: str) -> str:
        """Measure the unsigned angle between two accepted planes."""
        first, second = self._planes.get(first_plane_id), self._planes.get(second_plane_id)
        if first is None or second is None:
            return self._record_rejection(
                "measure_relative_plane_angle", "", [], "unknown_plane_id"
            )
        measurement = self._primitives.measure_relative_plane_angle(first, second)
        result = OracleToolResult("measure_relative_plane_angle", "", (), measurement=measurement)
        self._results.append(result)
        return json.dumps(_tool_payload(result))

    def measure_object_plane_relation(
        self, object_id: str, support_id: str, plane_id: str, ground_plane_id: str
    ) -> str:
        """Measure one object's clearance/contact relation to a selected support plane."""
        object, support = self._objects.get(object_id), self._objects.get(support_id)
        plane, ground = self._planes.get(plane_id), self._planes.get(ground_plane_id)
        if object is None or support is None:
            return self._record_rejection(
                "measure_object_plane_relation", "", [], "unknown_object_id"
            )
        if object.id == support.id:
            return self._record_rejection(
                "measure_object_plane_relation", object.label, [], "duplicate_object_id"
            )
        if plane is None or ground is None or ground_plane_id not in self._ground_planes:
            return self._record_rejection(
                "measure_object_plane_relation", "", [], "unknown_plane_id"
            )
        relation = self._primitives.measure_object_plane_relation(
            object, support, plane, ground.normal
        )
        if relation.rejection_reason is not None:
            return self._record_rejection(
                "measure_object_plane_relation",
                f"{object.label},{support.label}",
                list(relation.quality_flags),
                relation.rejection_reason,
            )
        metrics = tuple(
            (name, value)
            for name, value in (
                ("lower_clearance_m", relation.lower_clearance_m),
                ("upper_clearance_m", relation.upper_clearance_m),
                ("elevated_fraction", relation.elevated_fraction),
                ("contact_point_count", float(relation.contact_point_count)),
                ("planar_separation_m", relation.planar_separation_m),
                ("contact_overlap_count", float(relation.contact_overlap_count)),
            )
            if value is not None
        )
        result = OracleToolResult(
            "measure_object_plane_relation",
            f"{object.label},{support.label}",
            (_grounding_evidence(object), _grounding_evidence(support)),
            quality_flags=relation.quality_flags,
            metrics=metrics,
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result))

    def measure_aperture_geometry(self, mask_id: str, ground_plane_id: str) -> str:
        """Measure one selected aperture mask using an accepted ground plane."""
        handle = self._masks.get(mask_id)
        ground = self._planes.get(ground_plane_id)
        if handle is None or not isinstance(handle, tuple) or handle[1] is None:
            return self._record_rejection("measure_aperture_geometry", "", [], "unknown_mask_id")
        if ground is None or ground_plane_id not in self._ground_planes:
            return self._record_rejection(
                "measure_aperture_geometry", "", [], "unknown_ground_plane_id"
            )
        query, mask = handle
        result = self._primitives.measure_opening_width_from_mask(mask, ground)
        if result.measurement is None:
            return self._record_rejection(
                "measure_aperture_geometry",
                query,
                list(result.quality_flags),
                result.rejection_reason,
            )
        evidence = OracleEvidence(
            f"aperture:v1:{self._primitives.frame.id}:{mask_id}",
            "v1",
            mask_id,
            query,
            0.0,
            "n/a",
            0,
            result.measurement,
        )
        tool_result = OracleToolResult(
            "measure_aperture_geometry",
            query,
            (evidence,),
            measurement=result.measurement,
            quality_flags=result.quality_flags,
        )
        self._results.append(tool_result)
        return json.dumps(_tool_payload(tool_result))

    def measure_forward_corridor(self, ground_plane_id: str) -> str:
        """Measure visible forward ground and obstacle support using one accepted ground plane."""
        ground = self._planes.get(ground_plane_id)
        if ground is None or ground_plane_id not in self._ground_planes:
            return self._record_rejection(
                "measure_forward_corridor", "", [], "unknown_ground_plane_id"
            )
        measured: ForwardCorridorMeasurement = self._primitives.measure_forward_corridor(ground)
        if measured.rejection_reason is not None:
            return self._record_rejection(
                "measure_forward_corridor", "", [], measured.rejection_reason
            )
        evidence = OracleEvidence(
            f"forward-corridor:v1:{self._primitives.frame.id}",
            "v1",
            "forward-corridor",
            "forward corridor",
            0.0,
            "n/a",
            measured.point_count,
        )
        result = OracleToolResult(
            "measure_forward_corridor",
            "",
            (evidence,),
            quality_flags=("forward_ground_supported",),
            metrics=(
                ("ground_band_1_count", float(measured.ground_band_counts[0])),
                ("ground_band_2_count", float(measured.ground_band_counts[1])),
                ("ground_band_3_count", float(measured.ground_band_counts[2])),
                ("elevated_obstacle_count", float(measured.obstacle_count)),
                (
                    "largest_obstacle_cluster_count",
                    float(measured.largest_obstacle_cluster_count),
                ),
                ("corridor_point_count", float(measured.point_count)),
            ),
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result))

    def measure_height(self, object_id: str, plane_id: str) -> str:
        """Measure one grounded object against one previously accepted plane."""
        object = self._objects.get(object_id)
        plane = self._planes.get(plane_id)
        if object is None:
            return self._record_rejection("measure_height", object_id, [], "unknown_object_id")
        if plane is None or plane_id not in self._ground_planes:
            return self._record_rejection(
                "measure_height", object_id, [], "unknown_ground_plane_id"
            )
        measured = self._primitives.measure_height(object, plane)
        if measured.measurement is None:
            return self._record_rejection(
                "measure_height",
                object.label,
                list(measured.quality_flags),
                measured.rejection_reason,
            )
        measurement = measured.measurement
        measurement = replace(measurement, provenance_ids=(*measurement.provenance_ids, plane_id))
        evidence = _height_evidence(object, measurement)
        result = OracleToolResult(
            "measure_height",
            object.label,
            (evidence,),
            measurement=measurement,
            plane=plane,
            quality_flags=measured.quality_flags,
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
        "metrics": dict(result.metrics),
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
