"""Typed results returned by frame-scoped private perception primitives."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from dimos.benchmark.vqa.contracts import GroundedObject, GroundPlaneEstimate, OracleMeasurement


@dataclass(frozen=True)
class HeightMeasurementResult:
    """A private object-height measurement or its explicit rejection."""

    object: GroundedObject
    plane: GroundPlaneEstimate
    measurement: OracleMeasurement | None
    quality_flags: tuple[str, ...]
    rejection_reason: str | None = None


@dataclass(frozen=True)
class ClosestObjectResult:
    """A point-cloud selected candidate nearest to one grounded target object."""

    object: GroundedObject | None
    distance_m: float | None
    quality_flags: tuple[str, ...]
    rejection_reason: str | None = None


@dataclass(frozen=True)
class HorizontalRelationResult:
    """A pairwise camera-frame horizontal relation or its explicit rejection."""

    relation: Literal["left", "right"] | None
    quality_flags: tuple[str, ...]
    rejection_reason: str | None = None


@dataclass(frozen=True)
class ObjectPlaneRelationResult:
    """Private geometric relation of one visible object to a selected support plane."""

    lower_clearance_m: float | None
    upper_clearance_m: float | None
    elevated_fraction: float | None
    contact_point_count: int
    planar_separation_m: float | None
    contact_overlap_count: int
    quality_flags: tuple[str, ...]
    rejection_reason: str | None = None


@dataclass(frozen=True)
class OpeningWidthResult:
    """A metric doorway opening width, or an explicit conservative rejection."""

    measurement: OracleMeasurement | None
    quality_flags: tuple[str, ...]
    rejection_reason: str | None = None


@dataclass(frozen=True)
class ForwardPathResult:
    """A conservative visible-corridor classification from point-cloud evidence."""

    state: Literal["clear", "blocked"] | None
    point_count: int
    quality_flags: tuple[str, ...]
    rejection_reason: str | None = None
