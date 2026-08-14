"""Typed results returned by frame-scoped private perception primitives."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


@dataclass(frozen=True)
class HorizontalRelationResult:
    """A pairwise camera-frame horizontal relation or its explicit rejection."""

    relation: Literal["left", "right"] | None
    quality_flags: tuple[str, ...]
    rejection_reason: str | None = None
