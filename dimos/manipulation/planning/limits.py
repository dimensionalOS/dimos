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

"""Joint position limits: where rounding ends and a violation begins.

A position that floating-point arithmetic meant to put on a limit lands an ulp past
it (an IK solve against the limit, a spline sample beside a waypoint on it), and the
strict checks downstream, the planner's joint space and the native TOPP-RA, refuse
it. The helpers here move such a position a hair inside the limit and leave a real
violation alone for those checks to report as before.
"""

from __future__ import annotations

from collections.abc import Iterable

# How far past a position limit is rounding, not a violation (rad or m).
POSITION_LIMIT_TOLERANCE = 1e-6
# Where a position that close to a limit lands: inside it, by this much (rad or m).
POSITION_LIMIT_MARGIN = 1e-7


def snapped_to_limits(value: float, low: float, high: float) -> float:
    """``value`` moved ``POSITION_LIMIT_MARGIN`` inside a limit it is on, less than the
    margin inside of, or at most ``POSITION_LIMIT_TOLERANCE`` past; any other value as is."""
    # A range narrower than the margin has no inside to land in: stay on the limit.
    if low - POSITION_LIMIT_TOLERANCE <= value < low + POSITION_LIMIT_MARGIN:
        return min(low + POSITION_LIMIT_MARGIN, high)
    if high - POSITION_LIMIT_MARGIN < value <= high + POSITION_LIMIT_TOLERANCE:
        return max(high - POSITION_LIMIT_MARGIN, low)
    return value


def snap_to_limits(
    positions: Iterable[float], lower: Iterable[float], upper: Iterable[float]
) -> list[float]:
    """Each of ``positions`` snapped to its joint's ``[lower, upper]`` limits."""
    return [
        snapped_to_limits(float(value), float(low), float(high))
        for value, low, high in zip(positions, lower, upper, strict=True)
    ]
