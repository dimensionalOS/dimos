# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Deciding whether a trial succeeded from the world before and after it: a
``Snapshot`` holds the tracked bodies' poses and a ``Judge`` turns two of them
into a ``Verdict``. The sim judges are pure functions of the snapshots;
``OperatorJudge`` asks a person instead, for the real arm."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
from typing import Protocol

from pydantic import BaseModel

# A body that moved less than this between snapshots counts as untouched.
UNTOUCHED_M = 0.01


class BodyPose(BaseModel):
    """A body's world pose: ``position`` in metres, ``orientation`` as an xyzw quaternion."""

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


class Snapshot(BaseModel):
    """The world at one moment: every tracked body's pose, by body name."""

    bodies: dict[str, BodyPose]


class Verdict(BaseModel):
    """Whether the trial succeeded. ``stage`` is where it got to (``done`` on
    success, otherwise the step that failed) and ``note`` says why in words."""

    success: bool
    stage: str
    note: str


class Judge(Protocol):
    def verdict(self, before: Snapshot, after: Snapshot) -> Verdict: ...


class Region(BaseModel):
    """An axis-aligned box in world metres: ``low`` and ``high`` corners."""

    low: tuple[float, float, float]
    high: tuple[float, float, float]

    def contains(self, point: tuple[float, float, float]) -> bool:
        return all(
            low <= value <= high
            for low, value, high in zip(self.low, point, self.high, strict=True)
        )


def moved(before: BodyPose, after: BodyPose) -> float:
    """How far a body's centre travelled between two snapshots, in metres."""
    return math.dist(before.position, after.position)


@dataclass(frozen=True)
class LiftJudge:
    """Success when ``body`` ends at least ``min_rise_m`` metres higher than it started."""

    body: str
    min_rise_m: float = 0.05

    def verdict(self, before: Snapshot, after: Snapshot) -> Verdict:
        start, end = before.bodies[self.body], after.bodies[self.body]
        rise = end.position[2] - start.position[2]
        if rise >= self.min_rise_m:
            return Verdict(success=True, stage="done", note=f"{self.body} rose {rise:.3f} m")
        if moved(start, end) < UNTOUCHED_M:
            return Verdict(success=False, stage="reach", note=f"{self.body} was not touched")
        return Verdict(
            success=False,
            stage="grasp",
            note=f"{self.body} moved but rose only {rise:.3f} m of {self.min_rise_m} m",
        )


@dataclass(frozen=True)
class PlaceJudge:
    """Success when ``body`` ends inside ``region``."""

    body: str
    region: Region

    def verdict(self, before: Snapshot, after: Snapshot) -> Verdict:
        start, end = before.bodies[self.body], after.bodies[self.body]
        if self.region.contains(end.position):
            return Verdict(success=True, stage="done", note=f"{self.body} is in the region")
        if moved(start, end) < UNTOUCHED_M:
            return Verdict(success=False, stage="reach", note=f"{self.body} was not touched")
        x, y, z = end.position
        return Verdict(
            success=False,
            stage="place",
            note=f"{self.body} ended at ({x:.3f}, {y:.3f}, {z:.3f}), outside the region",
        )


@dataclass(frozen=True)
class OperatorJudge:
    """Asks the person at the robot. They answer ``y`` or ``n``, optionally
    followed by a note, e.g. ``n dropped it at the bowl rim``."""

    question: str = "Did the trial succeed? [y/n, then an optional note]: "
    ask: Callable[[str], str] = input

    def verdict(self, before: Snapshot, after: Snapshot) -> Verdict:
        answer = self.ask(self.question).strip()
        if answer[:1].lower() not in ("y", "n"):
            raise ValueError(f"expected an answer starting with y or n, got {answer!r}")
        success = answer[:1].lower() == "y"
        return Verdict(
            success=success, stage="done" if success else "operator", note=answer[1:].strip()
        )
