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

"""Generic geometry queries. No scan pattern, device model or physics step."""

from dimos_lcm.sim_msgs import BodyPoses
import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.sim2.description import SceneConfig
from dimos.sim2.geometry.mujoco import build_scene
from dimos.sim2.geometry.query import MujocoGeometry


class MujocoRaycaster:
    def load(self, scene: SceneConfig) -> None:
        self.geometry = MujocoGeometry(build_scene(scene, kinematic=True).compile(), scene.revision)

    def set_poses(self, sample: BodyPoses) -> None:
        self.geometry.set_poses(sample)

    def cast(
        self,
        origins: NDArray[np.float64],
        directions: NDArray[np.float64],
        exclude_body: str | None = None,
    ) -> NDArray[np.float64]:
        if origins.shape != directions.shape or origins.ndim != 2 or origins.shape[1] != 3:
            raise ValueError("ray origins and directions must be matching N x 3 arrays")
        distances = np.empty(len(directions))
        hit = np.empty(len(directions), dtype=np.int32)
        excluded = -1 if exclude_body is None else self.geometry.model.body(exclude_body).id
        if len(origins) and np.all(origins == origins[0]):
            mujoco.mj_multiRay(
                self.geometry.model,
                self.geometry.data,
                origins[0],
                directions.ravel(),
                None,
                True,
                excluded,
                hit,
                distances,
                None,
                len(directions),
                1e6,
            )
        else:
            for i, (origin, direction) in enumerate(zip(origins, directions, strict=True)):
                distances[i] = mujoco.mj_ray(
                    self.geometry.model,
                    self.geometry.data,
                    origin,
                    direction,
                    None,
                    True,
                    excluded,
                    hit[i : i + 1],
                )
        return distances

    def close(self) -> None:
        pass
