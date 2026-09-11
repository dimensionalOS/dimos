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

"""Conservative upright-bottle packing with space for opening the gripper.

A failed fit is a normal result. This planner never rearranges placed objects.
Coordinates and footprints are expressed in the tray frame, in metres.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class OccupiedFootprint:
    x: float
    y: float
    radius: float


def empty_slots(
    radius: float,
    occupied: tuple[OccupiedFootprint, ...] = (),
    *,
    inner_half_size: tuple[float, float] = (0.105, 0.105),
) -> list[tuple[float, float]]:
    """Return neat feasible slots, back row first, including release clearance.

    This is a conservative grid for upright circular bottles and a vertical
    gripper. It does not claim an optimal packing or handle arbitrary objects.
    The extra side clearance leaves room for imperfect learned placements.
    """
    numbers = (radius, *inner_half_size, *(v for o in occupied for v in (o.x, o.y, o.radius)))
    if not all(math.isfinite(v) for v in numbers) or radius <= 0:
        raise ValueError("Packing geometry must be finite with positive bottle radius")
    if any(v <= 0 for v in inner_half_size) or any(o.radius <= 0 for o in occupied):
        raise ValueError("Tray dimensions and occupied radii must be positive")
    bound_x = inner_half_size[0] - radius - 0.015
    bound_y = inner_half_size[1] - radius - 0.020
    if min(bound_x, bound_y) < 0:
        return []
    nx = min(3, 1 + math.floor(2 * bound_x / (2 * radius + 0.015) + 1e-9))
    ny = min(2, 1 + math.floor(2 * bound_y / max(0.12, 2 * radius + 0.04) + 1e-9))
    xs = [0.0] if nx == 1 else [-bound_x + 2 * bound_x * i / (nx - 1) for i in range(nx)]
    ys = [0.0] if ny == 1 else [bound_y, -bound_y]
    slots = []
    for y in ys:
        for x in xs:
            # Rectangle swept by the open vertical fingers. A neighbouring
            # circular bottle must clear this rectangle, not just the new bottle.
            if all(
                math.hypot(
                    max(abs(o.x - x) - (radius + 0.004), 0.0), max(abs(o.y - y) - 0.075, 0.0)
                )
                > o.radius + 0.003
                for o in occupied
            ):
                slots.append((x, y))
    return slots


def clear_pick_order(
    positions: tuple[tuple[float, float], ...],
    priority: tuple[int, ...] | None = None,
) -> list[int]:
    """Pick front bottles before a rear bottle in the same transfer corridor.

    The current vertical grasp cannot carry a bottle over another tall bottle.
    Positive Y is toward the tray in this workstation. Tie-breaking priority
    permits variation among sources that already have a clear approach.
    By default, work from left to right among accessible bottles.
    """
    if not all(math.isfinite(v) for xy in positions for v in xy):
        raise ValueError("Source positions must be finite")
    if priority is None:
        priority = tuple(
            sorted(range(len(positions)), key=lambda i: (positions[i][0], -positions[i][1], i))
        )
    if sorted(priority) != list(range(len(positions))):
        raise ValueError("Pick priority must contain every bottle exactly once")
    remaining = list(priority)
    ordered = []
    while remaining:
        index = next(
            i
            for i in remaining
            if not any(
                abs(positions[j][0] - positions[i][0]) < 0.07
                and positions[j][1] > positions[i][1] + 0.06
                for j in remaining
                if j != i
            )
        )
        ordered.append(index)
        remaining.remove(index)
    return ordered
