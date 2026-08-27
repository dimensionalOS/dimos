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

"""Pluggable planner interface for the motion environment.

A candidate is a factory `make(emb, resolution) -> PlannerEpisode`. The episode
is stateful across plan() calls (warm starts, hysteresis live inside it) and
nothing survives reset(). Honest candidates read only what plan() receives —
obstacles, pose, goal; the factory sees the embodiment and the waypoint
spacing, never a world.

`plan` also takes the route the caller has PUBLISHED, or None on the first call
and after a reset. The shell owns that memory (`adapter/planner.py` and the
episode loop already hold the last plan); the planner owns the judgment of
whether a fresh answer has earned the switch.

The search is PLANAR. `plan` is handed obstacle positions as (N, 2) xy, with
no z to read and therefore none to re-interpret: which returns are obstacles
was decided before the call, by an obstacle model that knows the body
(`motion/obstacles.py`). A candidate that wanted a z rule of its own would be
a second source of truth for the same question, which is the bug this shape
exists to make unrepresentable.
"""

from __future__ import annotations

from collections.abc import Callable
import itertools
import math
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path

RESOLUTION = 0.1  # waypoint spacing of the published route (m)
# Largest yaw change one published waypoint may command (rad). A consumer that
# stations the body along the route sweeps the yaw entering each station, and a
# station spans several waypoints, so this sits well under any sweep threshold
# a station is judged at (the rust planner's YAW_STEP is the same number).
YAW_STEP = 0.045


class PlannerEpisode(Protocol):
    def reset(self) -> None: ...

    def plan(
        self,
        obstacles: NDArray[np.floating[Any]],
        pose: Pose,
        goal: Pose,
        incumbent: Path | None = None,
        ground: NDArray[np.floating[Any]] | None = None,
        unseen_cost: float = 1.0,
    ) -> Path: ...


PlannerFactory = Callable[..., PlannerEpisode]


# Path <-> states, shared by every candidate: the search speaks (x, y, yaw)
# rows, the caller speaks nav_msgs Path, and only these three convert.
def states_of(path: Path | None) -> NDArray[np.float64] | None:
    """A published path back as the (N, 3) SE(2) the search speaks."""
    if path is None or not path.poses:
        return None
    return np.array(
        [[p.position.x, p.position.y, p.orientation.euler[2]] for p in path.poses]
    ).reshape(-1, 3)


def densify_states(states: NDArray[np.float64], res: float) -> list[NDArray[np.float64]]:
    """Interpolate sparse SE(2) vertices to path resolution (yaw = shortest arc)."""
    dense = [states[0]]
    for a, b in itertools.pairwise(states):
        dyaw = math.remainder(b[2] - a[2], 2 * math.pi)
        n = max(
            1,
            int(math.hypot(b[0] - a[0], b[1] - a[1]) / res),
            math.ceil(abs(dyaw) / YAW_STEP),  # ceil: a waypoint never exceeds YAW_STEP
        )
        for t in np.linspace(1.0 / n, 1.0, n):
            dense.append(
                np.array([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * dyaw])
            )
    return dense


def pose_stamped(x: float, y: float, yaw: float) -> PoseStamped:
    return PoseStamped(
        ts=0.0,  # deterministic: nothing here reads a stamp, and caches get pickled
        frame_id="world",
        position=[float(x), float(y), 0.0],
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, float(yaw))),
    )
