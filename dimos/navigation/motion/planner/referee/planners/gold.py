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

"""The SE(2) brute-force reference as a candidate — judge-side by role.

Kept behind the same interface for uniformity; it reads the scenario's true
boxes (a privilege honest candidates do not have), scores perfect deviation
by definition, and is ~100x over the live frame budget. Its jobs: harness
sanity, executor validation, and being the behavior others chase.
"""

from __future__ import annotations

import itertools
import math
from typing import Any

import numpy as np

from ..geometry import AvoidanceConfig
from ..scenarios import Scenario, se2_path
from ..types import Path, PointCloud2, PoseStamped, Quaternion, Vector3


class GoldEpisode:
    def __init__(self, sc: Scenario, resolution: float) -> None:
        self._sc = sc
        self._res = resolution

    def reset(self) -> None:
        pass

    def plan(
        self, cloud: PointCloud2, pose: tuple[float, float, float], goal: tuple[float, float]
    ) -> Path:
        states = se2_path(self._sc.boxes, pose, goal, self._sc.emb)
        if states is None:
            # Sealed: refuse — a single-pose stub, the follower runs out of path.
            return Path(frame_id="world", poses=[pose_stamped(*pose)])
        dense = densify_states(states, self._res)
        return Path(frame_id="world", poses=[pose_stamped(x, y, yaw) for x, y, yaw in dense])


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
            # clearing the veto. Same constant exp_0010 adopted candidate-side.
            math.ceil(abs(dyaw) / 0.045),
        )
        for t in np.linspace(1.0 / n, 1.0, n):
            dense.append(
                np.array([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * dyaw])
            )
    return dense


def pose_stamped(x: float, y: float, yaw: float) -> PoseStamped:
    return PoseStamped(
        frame_id="world",
        position=[float(x), float(y), 0.0],
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, float(yaw))),
    )


def make(sc: Scenario, cfg: AvoidanceConfig | None = None, **_: Any) -> GoldEpisode:
    res = (cfg or AvoidanceConfig()).resolution
    return GoldEpisode(sc, res)
