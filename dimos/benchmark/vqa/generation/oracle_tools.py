# Copyright 2026 Dimensional Inc.
"""Direct local LangChain tools over one frozen VQA frame."""

from __future__ import annotations

import json
from typing import Any

from langchain_core.tools import StructuredTool
import numpy as np

from dimos.benchmark.vqa.generation.ground_truth_generator import VqaGroundTruthGenerator
from dimos.benchmark.vqa.generation.measurements import estimate_ground_plane, points_in_mask
from dimos.benchmark.vqa.models import (
    CalibratedFrame,
    OracleEvidence,
    OracleMeasurement,
    OracleToolResult,
)


class LocalOracleToolRegistry:
    """Expose only named, side-effect-free perception operations to an oracle."""

    def __init__(self, frame: CalibratedFrame, grounding: VqaGroundTruthGenerator) -> None:
        self._frame = frame
        self._grounding = grounding
        self._results: list[OracleToolResult] = []

    @property
    def results(self) -> tuple[OracleToolResult, ...]:
        return tuple(self._results)

    def tools(self) -> list[StructuredTool]:
        return [
            StructuredTool.from_function(
                self.ground_semantic_object,
                name="ground_semantic_object",
                description=(
                    "Ground a visible semantic object query with private MoonDream, EdgeTAM, "
                    "and calibrated point-cloud geometry. Returns object geometry and evidence IDs."
                ),
            ),
            StructuredTool.from_function(
                self.estimate_ground_plane,
                name="estimate_ground_plane",
                description=(
                    "Estimate a visible ground plane from the frozen calibrated point cloud. "
                    "Returns a quality-gated local geometric result."
                ),
            ),
            StructuredTool.from_function(
                self.measure_object_height,
                name="measure_object_height",
                description=(
                    "Ground one visible semantic object with local MoonDream, EdgeTAM, and LiDAR "
                    "then measure its visible point-cloud height above the estimated ground plane."
                ),
            ),
            StructuredTool.from_function(
                self.measure_object_height_bucket,
                name="measure_object_height_bucket",
                description=(
                    "Measure one visible object's height, then return its public choice: under 0.5 m, "
                    "0.5-1.0 m, 1.0-1.5 m, or over 1.5 m."
                ),
            ),
        ]

    def ground_semantic_object(self, query: str) -> str:
        """Return grounded objects for a visible object query in the frozen frame."""
        objects, _ = self._grounding.ground(self._frame, query)
        evidence = tuple(
            OracleEvidence(
                id=f"grounding:v1:{item.id}",
                version="v1",
                object_id=item.id,
                label=item.label,
                range_m=item.range_m,
                side=item.horizontal_direction,
                point_count=item.point_count,
            )
            for item in objects
        )
        result = OracleToolResult("ground_semantic_object", query, evidence)
        self._results.append(result)
        return json.dumps(_tool_payload(result))

    def estimate_ground_plane(self) -> str:
        """Estimate the lower-image visible ground plane without fabricating a result."""
        fit = estimate_ground_plane(self._frame)
        if fit.estimate is None:
            result = OracleToolResult(
                "estimate_ground_plane",
                "",
                (),
                quality_flags=fit.quality_flags,
                rejection_reason=fit.rejection_reason,
            )
        else:
            measurement = OracleMeasurement(
                fit.estimate.offset_m,
                "m",
                max(fit.estimate.residual_m, 0.01),
                fit.quality_flags,
                (f"frame:{self._frame.id}",),
            )
            evidence = OracleEvidence(
                f"ground-plane:v1:{self._frame.id}",
                "v1",
                "ground-plane",
                "ground",
                0.0,
                "n/a",
                fit.estimate.inlier_count,
                measurement,
            )
            result = OracleToolResult(
                "estimate_ground_plane",
                "",
                (evidence,),
                measurement=measurement,
                plane=fit.estimate,
                quality_flags=fit.quality_flags,
            )
        self._results.append(result)
        return json.dumps(_tool_payload(result))

    def measure_object_height(self, query: str) -> str:
        """Measure one unambiguous grounded object's visible height above local ground."""
        objects, _ = self._grounding.ground(self._frame, query)
        masks_for_query = getattr(self._grounding, "masks_for_query", None)
        masks = tuple(masks_for_query(query)) if callable(masks_for_query) else ()
        fit = estimate_ground_plane(self._frame)
        flags = list(fit.quality_flags)
        if fit.estimate is None:
            return self._record_rejection(
                "measure_object_height", query, flags, fit.rejection_reason
            )
        if len(objects) != 1 or len(masks) != 1:
            flags.append("ambiguous_or_missing_object_mask")
            return self._record_rejection(
                "measure_object_height", query, flags, "ambiguous_object_evidence"
            )
        selected = points_in_mask(self._frame, masks[0].mask)
        if len(selected) < 6:
            flags.append("sparse_object_point_support")
            return self._record_rejection(
                "measure_object_height", query, flags, "insufficient_object_support"
            )
        normal = np.asarray(fit.estimate.normal)
        distances = selected @ normal + fit.estimate.offset_m
        positive = distances[distances > 0.02]
        if len(positive) < 4 or len(positive) / len(selected) < 0.6:
            flags.append("partial_or_non_elevated_object_support")
            return self._record_rejection(
                "measure_object_height", query, flags, "ambiguous_object_extent"
            )
        height = float(np.percentile(positive, 85))
        tolerance = float(max(0.05, fit.estimate.residual_m + np.std(positive) * 0.25))
        flags.extend(("visible_point_cloud_height", "conservative_upper_percentile"))
        measurement = OracleMeasurement(
            height,
            "m",
            tolerance,
            tuple(flags),
            (
                f"frame:{self._frame.id}",
                f"ground-plane:v1:{self._frame.id}",
                f"grounding:v1:{objects[0].id}",
            ),
        )
        evidence = OracleEvidence(
            f"height:v1:{objects[0].id}",
            "v1",
            objects[0].id,
            objects[0].label,
            objects[0].range_m,
            objects[0].horizontal_direction,
            len(selected),
            measurement,
        )
        result = OracleToolResult(
            "measure_object_height",
            query,
            (evidence,),
            measurement=measurement,
            plane=fit.estimate,
            quality_flags=tuple(flags),
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result))

    def measure_object_height_bucket(self, query: str) -> str:
        """Measure an object, then map its private height to a fixed public choice."""
        self.measure_object_height(query)
        height_result = self._results[-1]
        if height_result.measurement is None:
            return json.dumps(_tool_payload(height_result))
        result = OracleToolResult(
            "measure_object_height_bucket",
            query,
            height_result.evidence,
            measurement=height_result.measurement,
            choice=_height_bucket(height_result.measurement.value),
            plane=height_result.plane,
            quality_flags=height_result.quality_flags,
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result))

    def _record_rejection(self, tool: str, query: str, flags: list[str], reason: str | None) -> str:
        result = OracleToolResult(
            tool, query, (), quality_flags=tuple(flags), rejection_reason=reason
        )
        self._results.append(result)
        return json.dumps(_tool_payload(result))


def _tool_payload(result: OracleToolResult) -> dict[str, Any]:
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


def _height_bucket(height_m: float) -> str:
    if height_m < 0.5:
        return "under 0.5 m"
    if height_m < 1.0:
        return "0.5-1.0 m"
    if height_m < 1.5:
        return "1.0-1.5 m"
    return "over 1.5 m"
