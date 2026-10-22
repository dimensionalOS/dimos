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

"""Native RGB/depth camera capture in a dedicated sensor worker."""

import mujoco
import numpy as np
from numpy.typing import NDArray


class MujocoCamera:
    def __init__(
        self, model: mujoco.MjModel, width: int, height: int, *, shadows: bool = True
    ) -> None:
        self._renderer = mujoco.Renderer(model, height=height, width=width)
        self._shadows = shadows
        self._options = mujoco.MjvOption()
        # Source assets can put visible geometry in any MuJoCo group.
        self._options.geomgroup[:] = 1

    def capture(
        self, data: mujoco.MjData, camera: int, depth: bool
    ) -> tuple[NDArray[np.uint8], NDArray[np.float32] | None]:
        self._renderer.disable_depth_rendering()
        self._renderer.update_scene(data, camera=camera, scene_option=self._options)
        self._renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = self._shadows
        rgb = self._renderer.render().copy()
        distances = None
        if depth:
            self._renderer.enable_depth_rendering()
            distances = self._renderer.render().copy()
            self._renderer.disable_depth_rendering()
        return rgb, distances

    def close(self) -> None:
        self._renderer.close()
