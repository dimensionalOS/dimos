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

"""Local sampled state and stable array ordering, independent of engine and transport.

SI units; poses are world-from-link [x,y,z,qx,qy,qz,qw], velocities at link
origins [vx,vy,vz,wx,wy,wz] in world axes. Scalar joints use m/rad and N/Nm.
WorldState is sampled truth, not a solver checkpoint or an agent observation.
"""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

Array = NDArray[np.float64]


@dataclass(frozen=True)
class Layout:
    """Stable named array order, sent once when binding a consumer."""

    bodies: tuple[str, ...]
    joints: tuple[str, ...]
    drives: tuple[str, ...]
    roots: tuple[str, ...]
    imu_frames: tuple[str, ...]
    revision: str


@dataclass(frozen=True)
class WorldState:
    epoch: int
    tick: int
    sim_time: float
    wall_time: float  # publication time, for existing DimOS/TF consumers
    poses: Array  # (bodies, 7)
    twists: Array  # (bodies, 6)
    position: Array  # (scalar joints,)
    velocity: Array
    effort: Array  # actuator effort only; excludes contact and constraint reaction
    revision: str


@dataclass(frozen=True)
class Frame:
    state: WorldState
    imu: Array  # (layout.imu_frames, 6): local specific force, then angular velocity
