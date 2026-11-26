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

from dimos.sim2.mujoco.renders import RenderQuality
from dimos.sim2.mujoco.sensor_mount import SensorMount
from dimos.sim2.spec import ROBOT
from dimos.sim2.world_observations import WorldMirror
from dimos.simulation.sensors.lidar import Raycaster

Vector = NDArray[np.float64]

# Standard gravity in world axes (z up): an accelerometer at rest reads its negative.
GRAVITY = np.array([0.0, 0.0, -9.81])


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
        blind_to_robots: Sequence[str] = (),
    ) -> Raycaster:
        pass

    def imu(self, sensor: str) -> tuple[Vector, Vector, Vector]:
        pass

    def twin(self) -> "WorldView":
        pass

    def close(self) -> None: ...


def rotation_matrix(wxyz: Vector) -> Vector:
    return np.asarray(Rotation.from_quat(wxyz, scalar_first=True).as_matrix(), dtype=np.float64)


def imu_reading(
    pose: Vector, velocity: Vector, acceleration: Vector, imu_in_body: Vector
) -> tuple[Vector, Vector, Vector]:
    """The gyro, accelerometer (specific force) and x, y, z, w orientation of an IMU fixed at ``imu_in_body`` (a 4x4) on a body.

    ``pose`` is the body's x, y, z, qx, qy, qz, qw; ``velocity`` and ``acceleration`` are linear then
    angular, world axes, at the body's origin, without gravity: a BodyState as world_observations carries it.
    """
    world_from_body = Rotation.from_quat(pose[3:7]).as_matrix()
    world_from_imu = world_from_body @ imu_in_body[:3, :3]
    angular_velocity, linear_acceleration = velocity[3:], acceleration[:3]
    lever = world_from_body @ imu_in_body[:3, 3]
    # A point fixed on the body: tangential and centripetal acceleration on top of the origin's.
    at_imu = (
        linear_acceleration
        + np.cross(acceleration[3:], lever)
        + np.cross(angular_velocity, np.cross(angular_velocity, lever))
    )
    gyro = world_from_imu.T @ angular_velocity
    accelerometer = world_from_imu.T @ (at_imu - GRAVITY)
    return gyro, accelerometer, Rotation.from_matrix(world_from_imu).as_quat()


def robot_ids(world: WorldMirror) -> list[str]:
    with world.lock:
        return sorted(entity.path for entity in world.entities.values() if ROBOT in entity.cbor)


def open_view(
    scene: Path,
    ids: Iterable[str] = (),
    mount: SensorMount | None = None,
    quality: RenderQuality = RenderQuality(),
) -> WorldView:
    from dimos.sim2.mujoco.view import MujocoView

    return MujocoView(scene, ids, mount, quality)
