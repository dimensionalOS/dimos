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

"""Which returns are obstacles -- a property of the BODY, not of the scene.

The planner's world is a z-slice of the cloud, and where that slice belongs is
something the robot already knows: the base rides `base_height` above the
surface its feet stand on. Re-reference the cloud to that surface and the
geometry reads itself -- below `steppable` the legs negotiate it, above
`height` the body passes underneath, in between it is a wall. Nothing is
estimated off the scene, so nothing can be estimated wrong.

A model is z-only and says nothing about xy. The frame is the caller's: the
adapter re-references the cloud (`adapter/planner.py`) before handing it over,
and `hard_points` is where the two meet.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field as dc_field
from typing import TYPE_CHECKING, Protocol

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.navigation.motion.embodiment.base import Embodiment

# The absolute band `planners/target.py` slices, which is only the body's band
# if the map's z origin happens to be the ground.
# Ground exclusion for the body-referenced band. TWO voxel layers, not one: a
# floor whose true height sits near a voxel boundary quantises into both layers
# either side of it, and one layer leaves the upper one standing as a carpet
# the search cannot cross: at one layer the robot
# is inside its own band on every tick, at two on 7 % of them.
LOW = 0.16


def _no_soft() -> NDArray[np.float32]:
    return np.empty((0, 2), dtype=np.float32)


@dataclass(frozen=True)
class ObstacleField:
    """What a model made of one cloud, as indices into that cloud."""

    hard: NDArray[np.int32]  # (K,) never traversable
    soft: NDArray[np.float32] = dc_field(default_factory=_no_soft)  # (M, 2) [index, cost]


class ObstacleModel(Protocol):
    """A z-rule over a cloud referenced to the surface the feet stand on."""

    def field(self, cloud: NDArray[np.float32]) -> ObstacleField: ...


class BodyBand:
    """Obstacles by the body's own geometry: clear of the ground, under the belly."""

    def __init__(self, emb: Embodiment) -> None:
        self.low = LOW
        self.high = emb.height

    def field(self, cloud: NDArray[np.float32]) -> ObstacleField:
        z = cloud[:, 2]
        hard = np.flatnonzero((z > self.low) & (z <= self.high))
        return ObstacleField(hard=hard.astype(np.int32))


OBSTACLE_MODELS: dict[str, Callable[[Embodiment], ObstacleModel]] = {
    "body_band": BodyBand,  # z-referenced zones from the embodiment
}


def load(name: str, emb: Embodiment) -> ObstacleModel:
    """The named model, built for this body."""
    factory = OBSTACLE_MODELS.get(name)
    if factory is None:
        raise ValueError(f"unknown obstacle model {name!r}; known: {sorted(OBSTACLE_MODELS)}")
    return factory(emb)


def referenced(cloud: NDArray[np.float32], ground_z: float) -> NDArray[np.float32]:
    """The cloud in the frame a model reads: z off the support surface, finite rows only."""
    pts = np.asarray(cloud, dtype=np.float32).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    return pts - np.array([0.0, 0.0, ground_z], dtype=np.float32)


def hard_points(
    model: ObstacleModel, cloud: NDArray[np.float32], ground_z: float
) -> NDArray[np.float32]:
    """The obstacles this model sees -- the cloud the search plans on."""
    pts = referenced(cloud, ground_z)
    return np.ascontiguousarray(pts[model.field(pts).hard])
