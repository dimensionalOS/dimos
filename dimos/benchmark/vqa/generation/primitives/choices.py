"""Deterministic public-choice resolution from private measurements."""

from __future__ import annotations

from bisect import bisect_right

COUNT_CHOICES = ("1-2", "3-4", "5-7", "8+")
CAMERA_RANGE_CHOICES = ("under 1 m", "1 to under 2 m", "2 to under 4 m", "4 m or more")


def count_choice(count: int) -> str:
    """Return the public count bucket for one or more grounded instances."""
    if count < 1:
        raise ValueError("count must be positive")
    if count <= 2:
        return COUNT_CHOICES[0]
    if count <= 4:
        return COUNT_CHOICES[1]
    if count <= 7:
        return COUNT_CHOICES[2]
    return COUNT_CHOICES[3]


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


def height_choice_window(height_m: float) -> tuple[tuple[str, ...], str]:
    """Generate a local, exhaustive four-choice window around a private height."""
    breakpoints = (0.1, 0.2, 0.6, 1.0, 2.0)
    start = min(max(bisect_right(breakpoints, height_m) - 1, 0), len(breakpoints) - 3)
    lower, middle, upper = breakpoints[start : start + 3]
    choices = (
        f"under {_format_height(lower)} m",
        f"{_format_height(lower)}-{_format_height(middle)} m",
        f"{_format_height(middle)}-{_format_height(upper)} m",
        f"over {_format_height(upper)} m",
    )
    return choices, choices[bisect_right((lower, middle, upper), height_m)]


def _format_height(value: float) -> str:
    return f"{value:.1f}"
