"""Deterministic public-choice resolution from private measurements."""

from __future__ import annotations

COUNT_CHOICES = ("0", "1-2", "3-4", "5-7", "8+")
CAMERA_RANGE_CHOICES = ("under 1 m", "1 to under 2 m", "2 to under 4 m", "4 m or more")


def count_choice(count: int) -> str:
    """Return the exhaustive public count bucket for a non-negative count."""
    if count < 0:
        raise ValueError("count must be non-negative")
    if count == 0:
        return COUNT_CHOICES[0]
    if count <= 2:
        return COUNT_CHOICES[1]
    if count <= 4:
        return COUNT_CHOICES[2]
    if count <= 7:
        return COUNT_CHOICES[3]
    return COUNT_CHOICES[4]


def camera_range_choice(range_m: float) -> str:
    """Return the public camera-origin range bucket for a grounded object."""
    if range_m < 0:
        raise ValueError("range must be non-negative")
    if range_m < 1.0:
        return CAMERA_RANGE_CHOICES[0]
    if range_m < 2.0:
        return CAMERA_RANGE_CHOICES[1]
    if range_m < 4.0:
        return CAMERA_RANGE_CHOICES[2]
    return CAMERA_RANGE_CHOICES[3]
