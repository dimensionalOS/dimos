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

"""What a simulated sensor sees: its own world, built from the composed scene and posed from observations.

A sensor never reads a physics engine and is never described to one. It asks the manager for the
scene -- the same layer the engine is running -- builds a world of its own out of it, adds
whatever it needs at its own mount, applies the observations as they arrive, and reports what
its copy says. So it works the same whichever engine steps the world, and a scan lands on the
furniture that is drawn rather than on a generated stand-in.
"""

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
    """Frames are "<robot_id>/<name>" (or "world/<name>"): a body, or a sensor's frame_id."""

    @property
    def time(self) -> float:
        """Sim seconds of the world when last updated."""

    def update(self, world: WorldMirror) -> None:
        """Pose the view as the world is now; every read answers from it until the next update."""

    def pose(self, frame: str) -> tuple[Vector, Vector]:
        """World position and (w, x, y, z) orientation."""

    def render(self, camera: str, width: int, height: int) -> NDArray[np.uint8]:
        """An RGB image from a camera mount."""

    def owner(self, frame: str) -> str | None:
        """The robot whose subtree this frame is in, or None where it is the room's."""

    def velocity(self, frame: str) -> tuple[Vector, Vector]:
        """Linear and angular velocity in the frame's own axes."""

    def raycaster(
        self,
        groups: Sequence[int],
        mounted_at: str | None = None,
        blind_to: Sequence[str] = (),
    ) -> Raycaster:
        """Casts into the geoms in these groups (robots' meshes sit outside group 0).

        ``mounted_at`` names the frame whose own body the rays pass through, because a sensor
        sits inside its mount rather than proud of it; ``blind_to`` names any further frames to
        ignore.
        """

    def imu(self, sensor: str) -> tuple[Vector, Vector, Vector]:
        """Gyro, accelerometer (specific force) and (w, x, y, z) orientation at an IMU mount, in its frame."""

    def close(self) -> None: ...


def rotation_matrix(wxyz: Vector) -> Vector:
    return Rotation.from_quat(wxyz, scalar_first=True).as_matrix()


def robot_ids(world: WorldMirror) -> list[str]:
    """The robots in the world, as the world itself announced them.

    A sensor is not told what is in the world by anybody; it is told by the world. The first
    frame of observations adds each entity, and the ones that are robots carry the layer they
    were drawn from, which is what distinguishes a robot from a crate. Those ids are what the
    view has to shorten its own copy for, so that an entity in the observations names a body it
    can pose.
    """
    with world.lock:
        return sorted(entity.path for entity in world.entities.values() if "model" in entity.cbor)


def open_view(scene: Path, ids: Iterable[str] = (), mount: SensorMount | None = None) -> WorldView:
    """A sensor's own world: the composed scene, with what this sensor needs at its own mount.

    This belongs to the sensors and to the engine they build with -- not to sim2's core, which
    instantiates a backend, publishes observations and takes actions, and has no business
    knowing that anything is watching.
    """
    from dimos.sim2.mujoco.view import MujocoView

    return MujocoView(scene, ids, mount)
