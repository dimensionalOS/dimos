"""Typed results returned by frame-scoped private perception primitives."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from dimos.benchmark.vqa.models import GroundedObject, GroundPlaneEstimate, OracleMeasurement


@dataclass(frozen=True)
class HeightMeasurementResult:
    """A private object-height measurement or its explicit rejection."""

    object: GroundedObject
    plane: GroundPlaneEstimate
    measurement: OracleMeasurement | None
    quality_flags: tuple[str, ...]
    rejection_reason: str | None = None


@dataclass(frozen=True)
class DoorStateResult:
    """A conservative point-cloud classification of one door's state."""

    object: GroundedObject
    state: Literal["open", "closed"] | None
    quality_flags: tuple[str, ...]
    rejection_reason: str | None = None
    angle_deg: float | None = None
