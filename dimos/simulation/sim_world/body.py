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


"""What the simulated world needs from a robot: how to build it, where its lidars sit, and how to drive it."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Literal, Protocol

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.sim_msgs.Contacts import Part
from dimos.simulation.sensors.lidar import Raycaster, SimLidar

Vec3 = tuple[float, float, float]
CloudFrame = Literal["sensor", "base"]

ROBOT_VISUAL_GROUP = 2
COLLISION_GROUP = 3


@dataclass(frozen=True)
class BodyShape:
    """The robot's outside. The top box, in the base frame, bounds its highest point at any tilt."""

    half_width: float
    headroom: float
    step: float
    base_height: float
    top_center: Vec3
    top_rotation: tuple[Vec3, Vec3, Vec3]
    top_half_extents: Vec3
    lying_parts: tuple[Part, ...]


@dataclass(frozen=True)
class Sensor:
    """A lidar mounted on the robot's base.

    A lidar that looks out through a window in the base mesh does not see the base.
    """

    name: str
    mount_xyz: NDArray[np.float64]
    mount_r: NDArray[np.float64]
    lidar: Callable[[Raycaster, int], SimLidar]
    housing_half_extents: Vec3
    lio_blind_range: float = 0.0
    sees_base: bool = True


@dataclass(frozen=True)
class Rig:
    """A robot as a scene is built around it. Odometry and the cloud report at cloud_frame.

    The lidars see the scene and the robot's geoms in lidar_group.
    """

    spec: Callable[[], mujoco.MjSpec]
    base_body: str
    sensors: tuple[Sensor, ...]
    shape: BodyShape
    command_timeout: float
    tune: Callable[[mujoco.MjModel], None]
    world_frame: str
    odom_child_frame: str
    cloud_frame: CloudFrame
    lidar_group: int = ROBOT_VISUAL_GROUP

    def __post_init__(self) -> None:
        if self.cloud_frame == "sensor" and len(self.sensors) != 1:
            raise ValueError("a rig reporting at its sensor needs exactly one sensor")


class Body(Protocol):
    """A robot in a compiled model, stepped one control tick at a time from a velocity command."""

    @property
    def dt(self) -> float: ...

    @property
    def joint_names(self) -> tuple[str, ...]: ...

    def reset(self, x: float, y: float, z_feet: float, yaw: float) -> None: ...

    def tick(
        self, command: NDArray[np.float64], on_substep: Callable[[], None] | None = None
    ) -> None: ...

    def base_pose(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]: ...

    def base_velocity(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Linear velocity in the world frame and angular velocity in the base frame."""
        ...

    def joint_state(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]: ...

    def part(self, geom: int) -> Part: ...
