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

"""Camera device parameters in optical coordinates: +X right, +Y down, +Z forward."""

from dataclasses import dataclass
import math
from typing import Protocol

from dimos_lcm.geometry_msgs import Pose
from dimos_lcm.sim_msgs import WorldState
import numpy as np
from numpy.typing import NDArray

from dimos.sim2.description import SceneConfig


@dataclass(frozen=True)
class CameraConfig:
    frame_id: str
    source_id: str = "camera"
    hz: float = 15
    width: int = 320
    height: int = 240
    fx: float = 188.36
    fy: float = 188.36
    cx: float = 160
    cy: float = 120

    def __post_init__(self) -> None:
        if min(self.width, self.height, self.fx, self.fy, self.hz) <= 0 or not all(
            map(math.isfinite, (self.fx, self.fy, self.cx, self.cy, self.hz))
        ):
            raise ValueError("invalid camera intrinsics/rate")


class CameraRenderer(Protocol):
    def load(self, scene: SceneConfig, camera: CameraConfig) -> None: ...
    def set_poses(self, sample: WorldState) -> None: ...
    def render(self, pose_world: Pose) -> tuple[NDArray[np.uint8], NDArray[np.float32]]: ...
    def close(self) -> None: ...
