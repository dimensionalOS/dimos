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


from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Protocol

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.world_observations import WorldMirror
from dimos.simulation.sensors.lidar import Raycaster

Vector = NDArray[np.float64]


class WorldView(Protocol):
    @property
    def time(self) -> float:
        pass

    def update(self, world: WorldMirror) -> None:
        pass

    def pose(self, frame: str) -> tuple[Vector, Vector]:
        pass

    def render(self, camera: str, width: int, height: int) -> NDArray[np.uint8]:
        pass

    def owner(self, frame: str) -> str | None:
        pass

    def velocity(self, frame: str) -> tuple[Vector, Vector]:
        pass

    def raycaster(
        self,
        groups: Sequence[int],
        mounted_at: str | None = None,
        blind_to: Sequence[str] = (),
    ) -> Raycaster:
        pass

    def imu(self, sensor: str) -> tuple[Vector, Vector, Vector]:
        pass

    def close(self) -> None: ...


def rotation_matrix(wxyz: Vector) -> Vector:
    return Rotation.from_quat(wxyz, scalar_first=True).as_matrix()


def robot_ids(world: WorldMirror) -> list[str]:
    with world.lock:
        return sorted(entity.path for entity in world.entities.values() if "model" in entity.cbor)


def open_view(scene: Path, ids: Iterable[str] = (), mount: SensorMount | None = None) -> WorldView:
    from dimos.sim2.mujoco.view import MujocoView

    return MujocoView(scene, ids, mount)
