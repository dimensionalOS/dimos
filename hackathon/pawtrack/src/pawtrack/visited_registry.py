"""Pure visited-subject memory + random target selection (no DimOS).

The greeter must not greet the same person twice in quick succession and, when
several people are seated in view, should pick one at random. This module is the
dependency-free core of that: it remembers the floor position (world/map frame)
and time of each greeted subject, treats a freshly detected candidate within
``revisit_radius_m`` of any *recently* greeted position as already greeted, and
chooses one of the remaining candidates at random.

A greeting is forgotten after ``forget_after_s`` seconds, so the same person
becomes greet-eligible again -- the demo stays lively instead of going quiet once
everyone in the room has been greeted once. ``forget_after_s=None`` keeps the
old "greet each subject only once per run" behaviour.

Identity is positional, not visual: two detections close together on the floor
are the same subject. That is robust for seated people (their chair stays put)
and needs only the ground position the tracker already publishes.

Pure: stdlib only, unit tested. The caller injects a ``random.Random`` so runs
are reproducible, and the current timestamp so the forget logic is testable
without a clock. The DimOS container supplies candidate floor positions (from the
ground raycast) and marks a subject visited once it has been greeted.
"""

from __future__ import annotations

import dataclasses
import math
import random

Position = tuple[float, float]


@dataclasses.dataclass(frozen=True)
class Candidate:
    """One detected person-on-a-chair to consider greeting.

    Attributes:
        position: ``(x, y)`` floor position in the world/map frame.
        bbox: Detection box ``(x1, y1, x2, y2)`` in pixels, for the tracker.
    """

    position: Position
    bbox: tuple[float, float, float, float]


class VisitedRegistry:
    """Remembers recently greeted floor positions and filters candidates.

    A candidate within ``revisit_radius_m`` of a position greeted less than
    ``forget_after_s`` ago counts as already greeted. ``revisit_radius_m`` should
    comfortably exceed the tracker's position noise and a chair's footprint so
    one subject is not greeted twice in a row; ``forget_after_s`` sets how long
    before the same subject may be greeted again (``None`` = never).
    """

    def __init__(
        self,
        revisit_radius_m: float = 1.0,
        forget_after_s: float | None = None,
    ):
        if revisit_radius_m <= 0.0:
            raise ValueError("revisit_radius_m must be positive")
        if forget_after_s is not None and forget_after_s <= 0.0:
            raise ValueError("forget_after_s must be positive or None")
        self._radius = revisit_radius_m
        self._forget_after = forget_after_s
        self._visited: list[tuple[Position, float]] = []

    @property
    def visited_count(self) -> int:
        """How many greetings have been recorded (counts repeats)."""
        return len(self._visited)

    def _expired(self, when: float, now: float) -> bool:
        return self._forget_after is not None and now - when > self._forget_after

    def is_visited(self, position: Position, now: float = 0.0) -> bool:
        """Whether ``position`` was greeted within the radius and not forgotten."""
        return any(
            math.dist(position, seen) <= self._radius
            and not self._expired(when, now)
            for seen, when in self._visited
        )

    def mark_visited(self, position: Position, now: float = 0.0) -> None:
        """Record ``position`` as greeted at ``now``."""
        self._visited.append(
            ((float(position[0]), float(position[1])), float(now)))

    def unvisited(
        self, candidates: list[Candidate], now: float = 0.0
    ) -> list[Candidate]:
        """The candidates not greeted recently (within ``forget_after_s``)."""
        return [c for c in candidates if not self.is_visited(c.position, now)]


def select_target(
    candidates: list[Candidate],
    registry: VisitedRegistry,
    rng: random.Random,
    now: float = 0.0,
) -> Candidate | None:
    """Pick a random not-recently-greeted candidate, or None if there are none.

    Args:
        candidates: Detected persons-on-chairs this tick (with floor positions).
        registry: The visited memory used to filter out recently greeted ones.
        rng: Source of randomness for the choice.
        now: Current timestamp, for the registry's forget logic.

    Returns:
        A randomly chosen candidate that has not been greeted within
        ``forget_after_s``, or None when every candidate was greeted recently or
        the list is empty (the greeter then keeps wandering).
    """
    options = registry.unvisited(candidates, now)
    if not options:
        return None
    return rng.choice(options)
