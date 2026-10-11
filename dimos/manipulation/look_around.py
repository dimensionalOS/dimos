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

"""Where to put a wrist camera to see an object from around it.

Viewpoints sit on a ring around the object: the camera a set distance from the
object's centre, looking down at it from a set angle above the horizontal, at
evenly spaced bearings. A frame's Z axis must point up.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
import math
from typing import Any

import numpy as np
from numpy.typing import NDArray

# What a ``look`` move returns when the planner finds no path to a viewpoint.
UNREACHABLE = "unreachable"


def viewpoint(
    center: Sequence[float],
    bearing: float,
    distance: float,
    elevation: float,
    camera_from_tip: NDArray[np.float64],
) -> NDArray[np.float64]:
    """The tool tip pose that puts the camera at one viewpoint, looking at the object.

    Args:
        center: The object's centre, metres.
        bearing: Direction from the object to the camera, seen from above,
            radians from the frame's X axis toward its Y axis.
        distance: Camera to object centre, metres.
        elevation: The camera's height angle above the object, radians, below
            pi / 2. The image's rows stay level with the table.
        camera_from_tip: The camera's optical frame (Z along its view) in the
            tool tip's frame, 4x4.

    Returns the tool tip's pose, 4x4, in the frame of ``center``.
    """
    center = np.asarray(center, dtype=np.float64)
    eye = center + distance * np.array(
        [
            math.cos(elevation) * math.cos(bearing),
            math.cos(elevation) * math.sin(bearing),
            math.sin(elevation),
        ]
    )
    forward = (center - eye) / np.linalg.norm(center - eye)
    # Image rows level with the table: X across the view, horizontal.
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    camera = np.eye(4)
    camera[:3, :3] = np.column_stack([right, np.cross(forward, right), forward])
    camera[:3, 3] = eye
    return camera @ np.linalg.inv(camera_from_tip)


def look(
    center: Sequence[float],
    tip: NDArray[np.float64],
    camera_from_tip: NDArray[np.float64],
    views: int,
    ring_of: tuple[float, float, int],
    move: Callable[[NDArray[np.float64]], str | None],
    detect: Callable[[], object],
) -> dict[str, Any]:
    """Look at an object from up to ``views`` more viewpoints around it.

    The first view is where the camera is. Each next viewpoint is the one
    farthest around the ring from the views taken that ``move`` reaches.

    Args:
        center: The object's centre, metres.
        tip: The tool tip's pose now, 4x4.
        camera_from_tip: The camera's optical frame in the tool tip's frame, 4x4.
        views: How many more views to take.
        ring_of: The ring's camera distance (metres), elevation (radians) and
            number of evenly spaced bearings (see ``viewpoint``).
        move: Takes the arm to a tool tip pose (4x4); returns None on arrival,
            UNREACHABLE when there is no path, else why it stopped.
        detect: Looks at the object from where the arm is.

    Returns ``views`` taken and ``unreachable`` viewpoints, and ``stopped``
    when a move stopped part way.
    """
    distance, elevation, count = ring_of
    seen = [bearing_of(center, (tip @ camera_from_tip)[:3, 3])]
    bearings = ring(seen[0], count)[1:]
    facts: dict[str, Any] = {"views": 0, "unreachable": 0}
    while facts["views"] < views and bearings:
        bearing = farthest(bearings, seen)
        bearings.remove(bearing)
        why = move(viewpoint(center, bearing, distance, elevation, camera_from_tip))
        if why == UNREACHABLE:
            facts["unreachable"] += 1
            continue
        if why is not None:
            facts["stopped"] = why
            break
        seen.append(bearing)
        facts["views"] += 1
        detect()
    return facts


def back_away(
    tip: NDArray[np.float64], camera_from_tip: NDArray[np.float64], distance: float
) -> NDArray[np.float64]:
    """The straight move (metres) taking the camera ``distance`` back from what it looks at."""
    return -(tip @ camera_from_tip)[:3, 2] * distance


def bearing_of(center: Sequence[float], camera: Sequence[float]) -> float:
    """Direction (radians, as in ``viewpoint``) from the object to a camera position."""
    return math.atan2(camera[1] - center[1], camera[0] - center[0])


def ring(first: float, count: int) -> list[float]:
    """``count`` bearings evenly spaced around the object, starting from ``first`` (radians)."""
    return [first + 2.0 * math.pi * index / count for index in range(count)]


def farthest(bearings: Sequence[float], seen: Sequence[float]) -> float:
    """The bearing farthest, around the circle, from every bearing already seen."""
    return max(bearings, key=lambda bearing: min(_apart(bearing, other) for other in seen))


def _apart(a: float, b: float) -> float:
    """Angle (radians, 0 to pi) between two bearings."""
    return abs(math.remainder(a - b, 2.0 * math.pi))
