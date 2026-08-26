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
from importlib import import_module
import itertools
import math
from typing import Any, Protocol

import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path

RESOLUTION = 0.1  # waypoint spacing of the published route (m)


class PlannerEpisode(Protocol):
    def reset(self) -> None: ...

    def plan(
        self,
        obstacles: np.ndarray,
        pose: Pose,
        goal: Pose,
        incumbent: Path | None = None,
    ) -> Path: ...


PlannerFactory = Callable[..., PlannerEpisode]

# name -> "module:factory"; arbitrary "module:factory" strings load too, so
# generated candidates plug in without registering. Registry entries are
# package-relative so the package works wherever it is copied.
REGISTRY = {
    "target-py": ".target:make_py",  # port spec (python)
    "target": ".target:make",  # rust candidate
}


def load(name: str) -> PlannerFactory:
    """Resolve a registry name or a dotted "module:factory" string."""
    target = REGISTRY.get(name, name)
    mod, _, attr = target.partition(":")
    module = import_module(mod, package=__package__) if mod.startswith(".") else import_module(mod)
    factory: Any = getattr(module, attr or "make")
    return factory  # type: ignore[no-any-return]


# Path <-> states, shared by every candidate: the search speaks (x, y, yaw)
# rows, the caller speaks nav_msgs Path, and only these three convert.
def states_of(path: Path | None) -> np.ndarray | None:
    """A published path back as the (N, 3) SE(2) the search speaks."""
    if path is None or not path.poses:
        return None
    return np.array(
        [[p.position.x, p.position.y, p.orientation.euler[2]] for p in path.poses]
    ).reshape(-1, 3)


def densify_states(states: np.ndarray, res: float) -> list[np.ndarray]:
    """Interpolate sparse SE(2) vertices to path resolution (yaw = shortest arc)."""
    dense = [states[0]]
    for a, b in itertools.pairwise(states):
        dyaw = math.remainder(b[2] - a[2], 2 * math.pi)
        n = max(
            1,
            int(math.hypot(b[0] - a[0], b[1] - a[1]) / res),
            # CEIL, not int. The judge scores a station with the body swept
            # over all yaw entering it, and swaps the box for its
            # circumscribing cylinder above turn_yaw_eps (0.5 rad). One
            # lattice bin is 2*pi/16 = 0.3927 rad, and int(0.3927 / 0.15) = 2
            # published 0.196 rad per waypoint -- over sweep_yaw_step (0.15),
            # which this very term meant to stay under. Stations then
            # accumulated up to 0.511 rad and scored as the cylinder (radius
            # ~0.452 m vs a 0.155 m body half-width), which is what made
            # `--planner gold` veto its own path on gen028.
            # 0.045, not 0.15: a station spans several waypoints, so publishing
            # exactly at sweep_yaw_step still lets a station accumulate past
            # it. This keeps per-station yaw-in well under the threshold, which
            # is what makes scored clearance track truth instead of merely
            # clearing the veto.
            math.ceil(abs(dyaw) / 0.045),
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
