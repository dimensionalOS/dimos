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

"""Batched ray casts against a live MuJoCo scene."""

from __future__ import annotations

import mujoco
import numpy as np
from numpy.typing import NDArray

# Groups 0 and 1 are scene geometry and group 2 is robot visual meshes. Collision primitives
# are left out.
SCENE_AND_VISUAL_GROUPS = np.array((1, 1, 1, 0, 0, 0), dtype=np.uint8)
INCLUDE_STATIC = 1
NO_BODY_EXCLUDED = -1


class MujocoRaycaster:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model = model
        self.data = data

    def cast(
        self, origin: NDArray[np.float64], directions: NDArray[np.float64], max_range: float
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        n = len(directions)
        dist = np.full(n, -1.0)
        geom = np.full(n, -1, dtype=np.int32)
        normals = np.zeros(3 * n)
        mujoco.mj_multiRay(
            self.model,
            self.data,
            np.asarray(origin, dtype=np.float64),
            np.ascontiguousarray(directions, dtype=np.float64).ravel(),
            SCENE_AND_VISUAL_GROUPS,
            INCLUDE_STATIC,
            NO_BODY_EXCLUDED,
            geom,
            dist,
            normals,
            n,
            max_range,
        )
        dist[geom < 0] = -1.0
        return dist, normals.reshape(n, 3)
