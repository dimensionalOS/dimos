# Copyright 2026 Dimensional Inc.
"""Shared frame state and grounding workflow for deterministic VQA families."""

from __future__ import annotations

from dataclasses import dataclass

from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
from dimos.benchmark.vqa.models import CalibratedFrame, GroundedObject, ToolTrace


@dataclass(frozen=True)
class GroundingResult:
    """Grounded objects and the operations used to establish them."""

    objects: tuple[GroundedObject, ...]
    trace: tuple[ToolTrace, ...]


@dataclass
class FamilyContext:
    """Shared calibrated frame and cached primitives for deterministic families."""

    frame: CalibratedFrame
    primitives: FramePerceptionPrimitives

    def ground(self, object_query: str) -> GroundingResult:
        """Detect, segment, and ground one semantic query with traceable cache reuse."""
        if self.primitives.has_grounding(object_query):
            return GroundingResult(
                tuple(self.primitives.ground_masks(object_query)),
                (ToolTrace("reuse_grounding", object_query),),
            )
        trace: list[ToolTrace] = [ToolTrace("detect_objects", object_query)]
        detections = self.primitives.detect_objects(object_query)
        if len(detections):
            trace.append(ToolTrace("segment_objects", f"count={len(detections)}"))
        elif self.primitives.can_localize_points:
            trace.append(ToolTrace("locate_object_point", object_query))
        masks = self.primitives.segment_detections(object_query)
        if not len(detections) and self.primitives.used_point_localization(object_query):
            trace.append(ToolTrace("segment_object_point", object_query))
        trace.append(ToolTrace("get_foreground_geometry", f"masks={len(masks)}"))
        return GroundingResult(tuple(self.primitives.ground_masks(object_query)), tuple(trace))
