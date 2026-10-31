# Copyright 2025-2026 Dimensional Inc.
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

"""What a simulated sensor sees: the world as Simulation describes it, posed only from world_change.

A sensor never reads a physics engine, so it works the same whichever backend steps the world.
The renderer drawing the view is chosen by name, as backends are.
"""

from collections.abc import Mapping, Sequence
from importlib import import_module
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.sim2.world_change import WorldMirror
from dimos.simulation.sensors.mid360.lidar import Raycaster

RENDERERS = {"mujoco": "dimos.sim2.mujoco_view.MujocoView"}

Vector = NDArray[np.float64]


class WorldView(Protocol):
    """Frames are "<robot_id>/<name>" (or "world/<name>"): a body, or a sensor's mount."""

    @property
    def time(self) -> float:
        """Sim seconds of the world when last updated."""

    def update(self, world: WorldMirror) -> None:
        """Pose the view as the world is now; every read answers from it until the next update."""

    def pose(self, frame: str) -> tuple[Vector, Vector]:
        """World position and (w, x, y, z) orientation."""

    def render(self, camera: str, width: int, height: int) -> NDArray[np.uint8]:
        """An RGB image from a camera mount."""

    def raycaster(self, groups: Sequence[int]) -> Raycaster:
        """Casts into the geoms in these groups (robots' meshes sit outside group 0)."""

    def imu(self, sensor: str) -> tuple[Vector, Vector, Vector]:
        """Gyro, accelerometer (specific force) and (w, x, y, z) orientation at an IMU mount, in its frame."""

    def close(self) -> None: ...


def rotation_matrix(wxyz: Vector) -> Vector:
    return Rotation.from_quat(wxyz, scalar_first=True).as_matrix()


def open_view(description: Mapping[str, Any], renderer: str = "mujoco") -> WorldView:
    """A view of the world that Simulation.describe() returned."""
    module, _, cls = RENDERERS[renderer].rpartition(".")
    return getattr(import_module(module), cls)(description)  # type: ignore[no-any-return]
