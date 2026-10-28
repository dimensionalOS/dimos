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

"""Analytic J3 trajectory used in the final calibration experiment."""

import math


class CosineJointTrajectory:
    """Monotone J3 segment with two second cosine speed ramps."""

    def __init__(self, start: float, end: float, speed: float) -> None:
        if not all(math.isfinite(x) for x in (start, end, speed)):
            raise ValueError("Trajectory inputs must be finite")
        if not (0.16 <= start <= 0.40 and 0.16 <= end <= 0.40 and 0 < speed <= 0.04):
            raise ValueError("Trajectory is outside the measured envelope")
        if abs(end - start) < speed * 2.0:
            raise ValueError("Segment is too short for two second ramps")
        self.start = start
        self.end = end
        self.speed = speed
        self.direction = 1.0 if end > start else -1.0
        self.duration = abs(end - start) / speed + 2.0

    def sample(self, elapsed: float) -> tuple[float, float, float]:
        if not math.isfinite(elapsed):
            raise ValueError("Elapsed time must be finite")
        t = min(max(elapsed, 0.0), self.duration)
        if t < 2.0:
            displacement = self.speed * 0.5 * (t - 2 / math.pi * math.sin(math.pi * t / 2))
            velocity = self.speed * 0.5 * (1 - math.cos(math.pi * t / 2))
            acceleration = self.speed * math.pi / 4 * math.sin(math.pi * t / 2)
        elif t <= self.duration - 2.0:
            displacement = self.speed * (t - 1.0)
            velocity = self.speed
            acceleration = 0.0
        else:
            remaining = self.duration - t
            displacement = abs(self.end - self.start) - self.speed * 0.5 * (
                remaining - 2 / math.pi * math.sin(math.pi * remaining / 2)
            )
            velocity = self.speed * 0.5 * (1 - math.cos(math.pi * remaining / 2))
            acceleration = -self.speed * math.pi / 4 * math.sin(math.pi * remaining / 2)
        return (
            self.start + self.direction * displacement,
            self.direction * velocity,
            self.direction * acceleration,
        )
