"""Typed results returned by frame-scoped private perception primitives."""

from __future__ import annotations

from dataclasses import dataclass

from dimos.benchmark.vqa.models import GroundedObject, GroundPlaneEstimate, OracleMeasurement


@dataclass(frozen=True)
class HeightMeasurementResult:
    """A private object-height measurement or its explicit rejection."""

    object: GroundedObject
    plane: GroundPlaneEstimate
    measurement: OracleMeasurement | None
    quality_flags: tuple[str, ...]
    rejection_reason: str | None = None
