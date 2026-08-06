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

from collections.abc import AsyncIterator, Callable
from dataclasses import dataclass
from importlib import import_module
from typing import Protocol, TypeAlias

import numpy as np

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.robot.drone.px4.errors import MavsdkUnavailableError

__all__ = ("MavsdkOdometryValues",)

_MavsdkVector: TypeAlias = tuple[float, float, float]
_MavsdkQuaternion: TypeAlias = tuple[float, float, float, float]
_MavsdkCovariance: TypeAlias = tuple[float, ...]


@dataclass(frozen=True, slots=True)
class MavsdkOdometryValues:
    """MAVSDK-independent values for one external-vision odometry sample."""

    time_usec: int
    position_body: _MavsdkVector
    quaternion: _MavsdkQuaternion
    speed_body: _MavsdkVector
    angular_velocity_body: _MavsdkVector
    pose_covariance: _MavsdkCovariance
    velocity_covariance: _MavsdkCovariance


class MavsdkConnectionState(Protocol):
    is_connected: bool


class MavsdkCore(Protocol):
    def connection_state(self) -> AsyncIterator[MavsdkConnectionState]: ...


class MavsdkVelocityBodyYawspeed(Protocol):
    forward_m_s: float
    right_m_s: float
    down_m_s: float
    yawspeed_deg_s: float


class MavsdkPositionNedYaw(Protocol):
    north_m: float
    east_m: float
    down_m: float
    yaw_deg: float


class MavsdkVisionPositionEstimate(Protocol):
    pass


class MavsdkAction(Protocol):
    async def arm(self) -> None: ...

    async def disarm(self) -> None: ...

    async def set_takeoff_altitude(self, altitude: float) -> None: ...

    async def takeoff(self) -> None: ...

    async def land(self) -> None: ...


class MavsdkOffboard(Protocol):
    async def set_velocity_body(self, setpoint: MavsdkVelocityBodyYawspeed) -> None: ...

    async def set_position_ned(self, setpoint: MavsdkPositionNedYaw) -> None: ...

    async def start(self) -> None: ...

    async def stop(self) -> None: ...


class MavsdkMocap(Protocol):
    async def set_vision_position_estimate(
        self, estimate: MavsdkVisionPositionEstimate
    ) -> None: ...


class MavsdkPosition(Protocol):
    north_m: float
    east_m: float
    down_m: float


class MavsdkPositionVelocityNed(Protocol):
    position: MavsdkPosition


class MavsdkEulerAngle(Protocol):
    yaw_deg: float


class MavsdkFlightMode(Protocol):
    name: str


class MavsdkTelemetry(Protocol):
    def armed(self) -> AsyncIterator[bool]: ...

    def in_air(self) -> AsyncIterator[bool]: ...

    def flight_mode(self) -> AsyncIterator[MavsdkFlightMode | None]: ...

    def position_velocity_ned(self) -> AsyncIterator[MavsdkPositionVelocityNed]: ...

    def attitude_euler(self) -> AsyncIterator[MavsdkEulerAngle]: ...


class MavsdkSystem(Protocol):
    @property
    def core(self) -> MavsdkCore: ...

    @property
    def action(self) -> MavsdkAction: ...

    @property
    def offboard(self) -> MavsdkOffboard: ...

    @property
    def mocap(self) -> MavsdkMocap: ...

    @property
    def telemetry(self) -> MavsdkTelemetry: ...

    async def connect(self, system_address: str | None = None) -> None: ...


class MavsdkRuntime(Protocol):
    @property
    def action_error(self) -> type[Exception]: ...

    @property
    def mocap_error(self) -> type[Exception]: ...

    @property
    def offboard_error(self) -> type[Exception]: ...

    def create_system(self) -> MavsdkSystem: ...

    def velocity_body_yawspeed(
        self, forward: float, right: float, down: float, yawspeed: float
    ) -> MavsdkVelocityBodyYawspeed: ...

    def position_ned_yaw(
        self, north: float, east: float, down: float, yaw: float
    ) -> MavsdkPositionNedYaw: ...

    def vision_position_estimate(
        self, values: MavsdkOdometryValues
    ) -> MavsdkVisionPositionEstimate: ...


class _PositionBody(Protocol):
    pass


class _AngleBody(Protocol):
    pass


class _Covariance(Protocol):
    pass


@dataclass(frozen=True, slots=True)
class _MavsdkBindings:
    system_factory: Callable[[], MavsdkSystem]
    action_error: type[Exception]
    mocap_error: type[Exception]
    offboard_error: type[Exception]
    velocity_factory: Callable[[float, float, float, float], MavsdkVelocityBodyYawspeed]
    position_factory: Callable[[float, float, float, float], MavsdkPositionNedYaw]
    position_body_factory: Callable[[float, float, float], _PositionBody]
    angle_body_factory: Callable[[float, float, float], _AngleBody]
    covariance_factory: Callable[[list[float]], _Covariance]
    estimate_factory: Callable[
        [int, _PositionBody, _AngleBody, _Covariance, int], MavsdkVisionPositionEstimate
    ]


class _LoadedMavsdkRuntime:
    def __init__(self, bindings: _MavsdkBindings) -> None:
        self._bindings: _MavsdkBindings = bindings

    @property
    def action_error(self) -> type[Exception]:
        return self._bindings.action_error

    @property
    def mocap_error(self) -> type[Exception]:
        return self._bindings.mocap_error

    @property
    def offboard_error(self) -> type[Exception]:
        return self._bindings.offboard_error

    def create_system(self) -> MavsdkSystem:
        return self._bindings.system_factory()

    def velocity_body_yawspeed(
        self, forward: float, right: float, down: float, yawspeed: float
    ) -> MavsdkVelocityBodyYawspeed:
        return self._bindings.velocity_factory(forward, right, down, yawspeed)

    def position_ned_yaw(
        self, north: float, east: float, down: float, yaw: float
    ) -> MavsdkPositionNedYaw:
        return self._bindings.position_factory(north, east, down, yaw)

    def vision_position_estimate(
        self, values: MavsdkOdometryValues
    ) -> MavsdkVisionPositionEstimate:
        w, x, y, z = values.quaternion
        orientation = Quaternion.from_rotation_matrix(np.eye(3))
        orientation.x, orientation.y, orientation.z, orientation.w = x, y, z, w
        euler = orientation.to_euler()
        return self._bindings.estimate_factory(
            values.time_usec,
            self._bindings.position_body_factory(*values.position_body),
            self._bindings.angle_body_factory(euler.roll, euler.pitch, euler.yaw),
            self._bindings.covariance_factory(list(values.pose_covariance)),
            0,
        )


def _load_mavsdk_runtime() -> MavsdkRuntime:
    try:
        mavsdk = import_module("mavsdk")
        mavsdk_action = import_module("mavsdk.action")
        mavsdk_mocap = import_module("mavsdk.mocap")
        mavsdk_offboard = import_module("mavsdk.offboard")
    except ModuleNotFoundError as error:
        if error.name in {"mavsdk", "mavsdk.action", "mavsdk.mocap", "mavsdk.offboard"}:
            raise MavsdkUnavailableError() from error
        raise
    return _LoadedMavsdkRuntime(
        _MavsdkBindings(
            system_factory=mavsdk.System,
            action_error=mavsdk_action.ActionError,
            mocap_error=mavsdk_mocap.MocapError,
            offboard_error=mavsdk_offboard.OffboardError,
            velocity_factory=mavsdk_offboard.VelocityBodyYawspeed,
            position_factory=mavsdk_offboard.PositionNedYaw,
            position_body_factory=mavsdk_mocap.PositionBody,
            angle_body_factory=mavsdk_mocap.AngleBody,
            covariance_factory=mavsdk_mocap.Covariance,
            estimate_factory=mavsdk_mocap.VisionPositionEstimate,
        )
    )


class LazyMavsdkRuntime:
    def __init__(self) -> None:
        self._runtime: MavsdkRuntime | None = None

    def _load(self) -> MavsdkRuntime:
        if self._runtime is None:
            self._runtime = _load_mavsdk_runtime()
        return self._runtime

    @property
    def action_error(self) -> type[Exception]:
        return self._load().action_error

    @property
    def mocap_error(self) -> type[Exception]:
        return self._load().mocap_error

    @property
    def offboard_error(self) -> type[Exception]:
        return self._load().offboard_error

    def create_system(self) -> MavsdkSystem:
        return self._load().create_system()

    def velocity_body_yawspeed(
        self, forward: float, right: float, down: float, yawspeed: float
    ) -> MavsdkVelocityBodyYawspeed:
        return self._load().velocity_body_yawspeed(forward, right, down, yawspeed)

    def position_ned_yaw(
        self, north: float, east: float, down: float, yaw: float
    ) -> MavsdkPositionNedYaw:
        return self._load().position_ned_yaw(north, east, down, yaw)

    def vision_position_estimate(
        self, values: MavsdkOdometryValues
    ) -> MavsdkVisionPositionEstimate:
        return self._load().vision_position_estimate(values)
