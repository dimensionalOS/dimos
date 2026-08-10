"""Deterministic selection over grounded object evidence."""

from __future__ import annotations

from dimos.benchmark.vqa.models import GroundedObject


def select_nearest_object(
    objects: list[GroundedObject], side: str | None = None
) -> GroundedObject | None:
    """Return the nearest grounded object, optionally restricted to one image side."""
    candidates = [item for item in objects if side is None or item.horizontal_direction == side]
    return min(candidates, key=lambda item: item.range_m) if candidates else None
