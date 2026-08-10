"""Deterministic public-choice resolution from private measurements."""

from __future__ import annotations

from bisect import bisect_right


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
