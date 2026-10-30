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

"""Small public simulation contract. No engine IDs, native arrays or policy code.

SI units; poses are world-from-link [x,y,z,qx,qy,qz,qw], velocities at link
origins [vx,vy,vz,wx,wy,wz] in world axes. Scalar joints use m/rad and N/Nm.
WorldState is sampled truth, not a solver checkpoint or an agent observation.
"""

from collections.abc import Callable
from dataclasses import asdict, dataclass
from functools import cached_property
import hashlib
import json
from typing import Literal, Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.robot.assets.model import RobotModel

Array = NDArray[np.float64]


@dataclass(frozen=True)
class Drive:
    joint: str
    kp: float = 200.0
    kd: float = 20.0
    initial: float = 0.0
    armature: float = 0.0  # reflected rotor inertia, kg m² (rotary) or kg (linear)
    friction: float = 0.0  # Coulomb joint friction, Nm or N


@dataclass(frozen=True)
class Instance:
    id: str
    model: RobotModel
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    floating: bool = False
    drives: tuple[Drive, ...] = ()
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Camera:
    """Optical axes; the shared model owns the mount."""

    frame: str
    kind: Literal["camera"] = "camera"
    hz: float = 5.0
    width: int = 320
    height: int = 240
    fovy: float = 65.0


@dataclass(frozen=True)
class Lidar:
    frame: str
    kind: Literal["lidar"] = "lidar"
    hz: float = 10.0
    rays: int = 192
    packets: int = 6
    elevations: tuple[float, ...] = (-0.65, -0.3, 0.0, 0.15)
    min_range: float = 0.05
    max_range: float = 10.0
    max_pose_age: float = 0.03


@dataclass(frozen=True)
class Imu:
    frame: str
    kind: Literal["imu"] = "imu"


@dataclass(frozen=True)
class World:
    instances: tuple[Instance, ...]
    sensors: tuple[Camera | Lidar | Imu, ...] = ()
    dt: float = 0.002

    @cached_property
    def revision(self) -> str:
        """Identity of the materialized description; not a native solver checkpoint."""
        payload = {
            "dt": self.dt,
            "instances": [
                {
                    "id": i.id,
                    "xml": i.model.load().xml,
                    "xyz": i.xyz,
                    "rpy": i.rpy,
                    "floating": i.floating,
                    "drives": [asdict(d) for d in i.drives],
                }
                for i in self.instances
            ],
            "sensors": [{"kind": s.kind, **asdict(s)} for s in self.sensors],
        }
        return hashlib.sha256(
            json.dumps(json.loads(json.dumps(payload), parse_int=float), sort_keys=True).encode()
        ).hexdigest()


@dataclass(frozen=True)
class Layout:
    """Stable named array order, sent once when binding a consumer."""

    bodies: tuple[str, ...]
    joints: tuple[str, ...]
    drives: tuple[str, ...]
    roots: tuple[str, ...]
    imu_frames: tuple[str, ...]
    revision: str


@dataclass(frozen=True)
class WorldState:
    epoch: str
    tick: int
    sim_time: float
    wall_time: float  # publication time, for existing DimOS/TF consumers
    poses: Array  # (bodies, 7)
    twists: Array  # (bodies, 6)
    position: Array  # (scalar joints,)
    velocity: Array
    effort: Array  # actuator effort only; excludes contact and constraint reaction
    revision: str
    # Sensor measurements are carried alongside this state, not embedded in it.


@dataclass(frozen=True)
class Frame:
    dt: float  # interval of the source feedback clock, in simulation seconds
    state: WorldState
    imu: Array  # (layout.imu_frames, 6): local specific force, then angular velocity


@dataclass(frozen=True)
class JointEfforts:
    epoch: str
    joints: tuple[str, ...]
    efforts: tuple[float, ...]
    ttl: float = 0.25
    tick: int | None = None


@dataclass(frozen=True)
class JointTargets:
    epoch: str
    joints: tuple[str, ...]
    positions: tuple[float, ...]
    ttl: float = 0.25  # simulation seconds; expiration holds measured position
    tick: int | None = None  # response to this exact feedback tick, for paced control
    # Empty fields use zero velocity/feedforward and the world's Drive gains.
    # effort = kp*(position-q) + kd*(velocity-dq) + feedforward, at each physics step.
    velocities: tuple[float, ...] = ()
    kp: tuple[float, ...] = ()
    kd: tuple[float, ...] = ()
    feedforward: tuple[float, ...] = ()


@dataclass(frozen=True)
class Wrench:
    epoch: str
    body: str
    force: tuple[float, float, float]
    torque: tuple[float, float, float] = (0.0, 0.0, 0.0)
    point: tuple[float, float, float] | None = None  # world point; None = link origin
    duration: float = 0.1  # simulation seconds, not a persistent external force


class PhysicsBackend(Protocol):
    """One in-process dynamics owner. The caller serializes these operations.

    Position servos and direct joint effort are the actuation capabilities of v1.
    A future backend must satisfy the behavior tests, not just these signatures.
    """

    layout: Layout
    dt: float

    def targets(self, command: JointTargets) -> None:
        """Validate every entry before changing any drive, then latch targets."""
        ...

    def efforts(self, command: JointEfforts) -> None: ...
    def hold(self, joints: tuple[str, ...]) -> None: ...
    def validate_wrench(self, wrench: Wrench) -> None: ...
    def step(self, wrenches: tuple[Wrench, ...]) -> None: ...
    def snapshot(self, epoch: str, tick: int) -> Frame: ...
    def reset(self) -> None: ...
    def place(self, body: str, pose: tuple[float, ...]) -> None:
        """Privileged scene edit: free root only, zero its velocity; not robot control."""
        ...


@dataclass(frozen=True)
class Scan:
    points: Array  # world-frame hits, preserving acquisition time per point
    times: Array
    epoch: str


class SensorReader(Protocol):
    """Read-only acquisition provider; a separate seam from dynamics."""

    gaps: int

    def observe(self, state: WorldState) -> None: ...
    def camera(
        self, frame: str, state: WorldState
    ) -> tuple[NDArray[np.uint8], NDArray[np.float32]]: ...
    def lidar(self, frame: str, duration: float = 0.1) -> Scan | None: ...
    def close(self) -> None: ...


SensorFactory = Callable[[World, Layout], SensorReader]
