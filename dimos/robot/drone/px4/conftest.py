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

import asyncio
from collections.abc import AsyncIterator
from dataclasses import dataclass
from typing import Protocol

import pytest

from dimos.robot.drone.px4.mavsdk_runtime import (
    MavsdkOdometryValues,
    MavsdkRuntime,
    MavsdkSystem,
    MavsdkVisionPositionEstimate,
)


class _ConnectedState:
    is_connected: bool = True


class _Core:
    async def connection_state(self) -> AsyncIterator[_ConnectedState]:
        yield _ConnectedState()


class _Telemetry:
    def __init__(self) -> None:
        self.position_received: asyncio.Event = asyncio.Event()
        self.yaw_received: asyncio.Event = asyncio.Event()

    async def armed(self) -> AsyncIterator[bool]:
        _ = await asyncio.Event().wait()
        yield False

    async def in_air(self) -> AsyncIterator[bool]:
        _ = await asyncio.Event().wait()
        yield False

    async def flight_mode(self) -> AsyncIterator[None]:
        _ = await asyncio.Event().wait()
        yield None

    async def position_velocity_ned(self) -> AsyncIterator["_PositionVelocityNed"]:
        _ = self.position_received.set()
        yield _PositionVelocityNed()
        _ = await asyncio.Event().wait()

    async def attitude_euler(self) -> AsyncIterator["_EulerAngle"]:
        _ = self.yaw_received.set()
        yield _EulerAngle()
        _ = await asyncio.Event().wait()


@dataclass(frozen=True, slots=True)
class _PositionNed:
    north_m: float = 1.0
    east_m: float = 2.0
    down_m: float = -3.0


@dataclass(frozen=True, slots=True)
class _PositionVelocityNed:
    position: _PositionNed = _PositionNed()


@dataclass(frozen=True, slots=True)
class _EulerAngle:
    yaw_deg: float = 30.0


class _VelocitySetpoint(Protocol):
    forward_m_s: float
    right_m_s: float
    down_m_s: float
    yawspeed_deg_s: float


class _PositionSetpoint(Protocol):
    north_m: float
    east_m: float
    down_m: float
    yaw_deg: float


class _VelocityBodyYawspeed:
    def __init__(self, forward: float, right: float, down: float, yawspeed: float) -> None:
        self.forward_m_s: float = forward
        self.right_m_s: float = right
        self.down_m_s: float = down
        self.yawspeed_deg_s: float = yawspeed


class _PositionNedYaw:
    def __init__(self, north: float, east: float, down: float, yaw: float) -> None:
        self.north_m: float = north
        self.east_m: float = east
        self.down_m: float = down
        self.yaw_deg: float = yaw


class _Offboard:
    def __init__(self) -> None:
        self.velocity_setpoints: list[tuple[float, float, float, float]] = []
        self.position_setpoints: list[tuple[float, float, float, float]] = []
        self.start_calls: int = 0
        self.stop_calls: int = 0

    async def set_velocity_body(self, setpoint: _VelocitySetpoint) -> None:
        self.velocity_setpoints.append(
            (
                setpoint.forward_m_s,
                setpoint.right_m_s,
                setpoint.down_m_s,
                setpoint.yawspeed_deg_s,
            )
        )

    async def set_position_ned(self, setpoint: _PositionSetpoint) -> None:
        self.position_setpoints.append(
            (setpoint.north_m, setpoint.east_m, setpoint.down_m, setpoint.yaw_deg)
        )

    async def start(self) -> None:
        self.start_calls += 1

    async def stop(self) -> None:
        self.stop_calls += 1


class _Action:
    async def arm(self) -> None:
        return None

    async def disarm(self) -> None:
        return None

    async def set_takeoff_altitude(self, altitude: float) -> None:
        del altitude

    async def takeoff(self) -> None:
        return None

    async def land(self) -> None:
        return None


class _VisionPositionEstimate:
    pass


class _Mocap:
    async def set_vision_position_estimate(self, estimate: MavsdkVisionPositionEstimate) -> None:
        del estimate


class _System:
    def __init__(self) -> None:
        self._core: _Core = _Core()
        self._action: _Action = _Action()
        self._telemetry: _Telemetry = _Telemetry()
        self._offboard: _Offboard = _Offboard()
        self._mocap: _Mocap = _Mocap()
        self.connection_urls: list[str | None] = []

    @property
    def core(self) -> _Core:
        return self._core

    @property
    def action(self) -> _Action:
        return self._action

    @property
    def telemetry(self) -> _Telemetry:
        return self._telemetry

    @property
    def offboard(self) -> _Offboard:
        return self._offboard

    @property
    def mocap(self) -> _Mocap:
        return self._mocap

    async def connect(self, system_address: str | None = None) -> None:
        self.connection_urls.append(system_address)


class _MavsdkRuntime:
    def __init__(self, system: MavsdkSystem) -> None:
        self._system: MavsdkSystem = system

    def create_system(self) -> MavsdkSystem:
        return self._system

    @property
    def action_error(self) -> type[Exception]:
        return RuntimeError

    @property
    def mocap_error(self) -> type[Exception]:
        return RuntimeError

    @property
    def offboard_error(self) -> type[Exception]:
        return RuntimeError

    def velocity_body_yawspeed(
        self, forward: float, right: float, down: float, yawspeed: float
    ) -> _VelocityBodyYawspeed:
        return _VelocityBodyYawspeed(forward, right, down, yawspeed)

    def position_ned_yaw(
        self, north: float, east: float, down: float, yaw: float
    ) -> _PositionNedYaw:
        return _PositionNedYaw(north, east, down, yaw)

    def vision_position_estimate(self, values: MavsdkOdometryValues) -> _VisionPositionEstimate:
        del values
        return _VisionPositionEstimate()


@pytest.fixture
def fake_system() -> _System:
    return _System()


@pytest.fixture
def fake_runtime(fake_system: MavsdkSystem) -> MavsdkRuntime:
    return _MavsdkRuntime(fake_system)
