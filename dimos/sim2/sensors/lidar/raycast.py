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

"""Batched MuJoCo queries; self geometry is hidden only on the worker's model."""

import mujoco
import numpy as np
from numpy.typing import NDArray


class Raycaster:
    def __init__(
        self,
        model: mujoco.MjModel,
        robot_root: int | None = None,
        *,
        self_occlusion: bool = False,
        excluded_meshes: tuple[str, ...] = (),
    ) -> None:
        self.model = model
        excluded = {model.mesh(name).id for name in excluded_meshes}
        # Each sensor owns this model copy. Hiding self does not alter physics.
        model.geom_group[:] = 0
        for geom, body in enumerate(model.geom_bodyid):
            ancestor = int(body)
            while ancestor and ancestor != robot_root:
                ancestor = int(model.body_parentid[ancestor])
            if (
                (robot_root is not None and ancestor == robot_root and not self_occlusion)
                or model.geom_rgba[geom, 3] == 0
                or (
                    model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_MESH
                    and int(model.geom_dataid[geom]) in excluded
                )
            ):
                model.geom_group[geom] = 5
        self.groups = np.ones(6, dtype=np.uint8)
        self.groups[5] = 0

    def cast(
        self,
        data: mujoco.MjData,
        origin: NDArray[np.float64],
        directions: NDArray[np.float64],
        min_range: float,
        max_range: float,
    ) -> NDArray[np.float64]:
        distances = self.ranges(data, origin, directions, max_range)
        valid = (distances >= min_range) & (distances <= max_range)
        return np.asarray(origin + directions[valid] * distances[valid, None], dtype=np.float64)

    def ranges(
        self,
        data: mujoco.MjData,
        origin: NDArray[np.float64],
        directions: NDArray[np.float64],
        max_range: float,
    ) -> NDArray[np.float64]:
        return self.hits(data, origin, directions, max_range)[0]

    def hits(
        self,
        data: mujoco.MjData,
        origin: NDArray[np.float64],
        directions: NDArray[np.float64],
        max_range: float,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Ranges and world normals, aligned with input rays; misses remain negative."""
        rays = np.ascontiguousarray(directions, dtype=np.float64)
        distances = np.full(len(rays), -1.0)
        ids = np.full(len(rays), -1, dtype=np.int32)
        normals = np.zeros_like(rays)
        mujoco.mj_multiRay(
            self.model,
            data,
            origin,
            rays.ravel(),
            self.groups,
            1,
            -1,
            ids,
            distances,
            normals.ravel(),
            len(rays),
            max_range,
        )
        return distances, normals
