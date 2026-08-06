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

"""Direct MAVSDK gRPC flight control and external-vision forwarding."""
# noqa: SIZE_OK - Flight control and vision are intentionally one hardware boundary.

import asyncio  # noqa: ANYIO_OK - Module lifecycle exposes a native asyncio loop.
from collections.abc import Sequence
from dataclasses import dataclass
import json
from logging import Logger
import math
from typing import Final, TypeAlias

import numpy as np

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.drone.px4.config import MavsdkConfig
from dimos.robot.drone.px4.errors import MavsdkConnectionTimeoutError
from dimos.robot.drone.px4.frames import flu_to_frd_body_velocity
from dimos.robot.drone.px4.mavsdk_runtime import (
    LazyMavsdkRuntime,
    MavsdkOdometryValues,
    MavsdkRuntime,
    MavsdkSystem,
)
from dimos.robot.drone.px4.mid360_mount_tf import BASE_TO_MID360
from dimos.utils.logging_config import setup_logger

logger: Logger = setup_logger()

_VisionVector: TypeAlias = tuple[float, float, float]
_VisionQuaternion: TypeAlias = tuple[float, float, float, float]
_VisionCovariance: TypeAlias = tuple[float, ...]
_FLU_TO_FRD: Final[np.ndarray] = np.diag((1.0, -1.0, -1.0))
_UNKNOWN_COVARIANCE: Final[_VisionCovariance] = (float("nan"),)


@dataclass(frozen=True, slots=True)
class _InvalidExternalVisionSampleError(Exception):
    detail: str

    def __str__(self) -> str:
        return self.detail


class _MavsdkLoopUnavailableError(RuntimeError):
    pass


def _require_finite(values: Sequence[float], detail: str) -> None:
    if not all(math.isfinite(value) for value in values):
        raise _InvalidExternalVisionSampleError(detail)


def _vector_tuple(vector: Vector3) -> _VisionVector:
    return (vector.x, vector.y, vector.z)


def _flu_to_frd_vector(vector: _VisionVector) -> _VisionVector:
    _require_finite(vector, "position must be finite")
    return (vector[0], -vector[1], -vector[2])


def _flu_to_frd_hamilton_quaternion(quaternion: _VisionQuaternion) -> _VisionQuaternion:
    _require_finite(quaternion, "orientation quaternion must be finite")
    return (quaternion[0], quaternion[1], -quaternion[2], -quaternion[3])


def _quaternion_from_hamilton(quaternion: _VisionQuaternion) -> Quaternion:
    result = Quaternion.from_rotation_matrix(np.eye(3))
    result.x, result.y, result.z, result.w = (
        quaternion[1],
        quaternion[2],
        quaternion[3],
        quaternion[0],
    )
    return result


def _validated_quaternion(quaternion: Quaternion) -> Quaternion:
    values = quaternion.to_tuple()
    _require_finite(values, "orientation quaternion must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isclose(norm, 1.0, abs_tol=1e-6):
        raise _InvalidExternalVisionSampleError("orientation quaternion must be normalized")
    return _quaternion_from_hamilton((values[3], values[0], values[1], values[2])).normalize()


def _mounted_frd_hamilton(odom_to_base: Transform) -> _VisionQuaternion:
    rotation = odom_to_base.rotation
    return _flu_to_frd_hamilton_quaternion((rotation.w, rotation.x, rotation.y, rotation.z))


def _transform_sensor_twist_to_base_frd(
    linear_sensor: _VisionVector, angular_sensor: _VisionVector
) -> tuple[_VisionVector, _VisionVector]:
    _require_finite((*linear_sensor, *angular_sensor), "twist must be finite")
    angular_base = BASE_TO_MID360.rotation.rotate_vector(Vector3(angular_sensor))
    rotated_linear = BASE_TO_MID360.rotation.rotate_vector(Vector3(linear_sensor))
    lever_velocity = angular_base.cross(BASE_TO_MID360.translation)
    linear_base = rotated_linear - lever_velocity
    return _flu_to_frd_vector(_vector_tuple(linear_base)), _flu_to_frd_vector(
        _vector_tuple(angular_base)
    )


def _transform_pose_covariance_to_frd(
    covariance: Sequence[float] | np.ndarray, odom_mid360_quaternion: _VisionQuaternion
) -> _VisionCovariance:
    if len(covariance) != 36 or not all(math.isfinite(value) for value in covariance):
        return _UNKNOWN_COVARIANCE
    w, x, y, z = odom_mid360_quaternion
    rotation = _validated_quaternion(_quaternion_from_hamilton((w, x, y, z)))
    lever = rotation.rotate_vector(BASE_TO_MID360.inverse().translation)
    skew = np.array(
        (
            (0.0, -lever.z, lever.y),
            (lever.z, 0.0, -lever.x),
            (-lever.y, lever.x, 0.0),
        )
    )
    jacobian = np.zeros((6, 6))
    jacobian[:3, :3] = _FLU_TO_FRD
    jacobian[:3, 3:] = -_FLU_TO_FRD @ skew
    jacobian[3:, 3:] = _FLU_TO_FRD
    transformed = jacobian @ np.asarray(covariance, dtype=float).reshape((6, 6)) @ jacobian.T
    return tuple(float(transformed[row, column]) for row in range(6) for column in range(row, 6))


def _seconds_to_microseconds(timestamp_s: float) -> int:
    if not math.isfinite(timestamp_s) or timestamp_s <= 0.0:
        raise _InvalidExternalVisionSampleError("timestamp must be finite and positive")
    return round(timestamp_s * 1_000_000.0)


def _build_mavsdk_odometry_values(message: Odometry) -> MavsdkOdometryValues:
    if message.frame_id != "odom" or message.child_frame_id != "mid360_link":
        raise _InvalidExternalVisionSampleError("PointLIO odometry must be odom -> mid360_link")
    odom_to_mid360 = Transform.from_pose("mid360_link", message.to_pose_stamped())
    odom_to_mid360.rotation = _validated_quaternion(message.orientation)
    odom_to_base = odom_to_mid360 + BASE_TO_MID360.inverse()
    linear_frd, angular_frd = _transform_sensor_twist_to_base_frd(
        _vector_tuple(message.linear_velocity), _vector_tuple(message.angular_velocity)
    )
    quaternion = message.orientation
    return MavsdkOdometryValues(
        time_usec=_seconds_to_microseconds(message.ts),
        position_body=_flu_to_frd_vector(_vector_tuple(odom_to_base.translation)),
        quaternion=_mounted_frd_hamilton(odom_to_base),
        speed_body=linear_frd,
        angular_velocity_body=angular_frd,
        pose_covariance=_transform_pose_covariance_to_frd(
            message.pose.covariance.tolist(),
            (quaternion.w, quaternion.x, quaternion.y, quaternion.z),
        ),
        velocity_covariance=_UNKNOWN_COVARIANCE,
    )


class FlightController(Module):
    """Expose MAVSDK gRPC flight skills, velocity control, and vision pose."""

    cmd_vel: In[Twist]
    odometry: In[Odometry]

    # RPC handlers are registered before start(); callers must not invoke skills until startup completes.
    def __init__(
        self,
        *,
        g: GlobalConfig = global_config,
        mavsdk_config: MavsdkConfig | None = None,
        mavsdk_runtime: MavsdkRuntime | None = None,
    ) -> None:
        super().__init__(g=g)
        self._mavsdk_config: MavsdkConfig = mavsdk_config or MavsdkConfig()
        self._mavsdk_runtime: MavsdkRuntime = mavsdk_runtime or LazyMavsdkRuntime()
        self._system: MavsdkSystem
        self._connected: bool = False
        self._armed: bool | None = None
        self._in_air: bool | None = None
        self._flight_mode: str | None = None
        self._offboard_active: bool = False
        self._position_ned: tuple[float, float, float] | None = None
        self._yaw_deg: float | None = None
        self._telemetry_tasks: list[asyncio.Task[None]] = []

    def _set_offboard_active(self, active: bool) -> None:
        self._offboard_active = active

    @skill
    async def arm(self) -> str:
        """Arm the vehicle motors."""
        try:
            await self._system.action.arm()
        except self._mavsdk_runtime.action_error as error:
            return f"arm failed: {error}"
        return "arm command sent"

    @skill
    async def disarm(self) -> str:
        """Disarm the vehicle motors when the autopilot permits it."""
        try:
            await self._system.action.disarm()
        except self._mavsdk_runtime.action_error as error:
            return f"disarm failed: {error}"
        return "disarm command sent"

    @skill
    async def takeoff(self, altitude: float = 3.0) -> str:
        """Set a positive takeoff altitude and command takeoff.

        Args:
            altitude: Target altitude above the takeoff point in meters.
        """
        if not math.isfinite(altitude) or altitude <= 0.0:
            return "takeoff failed: altitude must be a finite positive number"
        try:
            await self._system.action.set_takeoff_altitude(altitude)
            await self._system.action.takeoff()
        except self._mavsdk_runtime.action_error as error:
            return f"takeoff failed: {error}"
        return f"takeoff command sent for {altitude} m"

    @skill
    async def land(self) -> str:
        """Command the vehicle to land at its current position."""
        try:
            await self._system.action.land()
        except (self._mavsdk_runtime.action_error, self._mavsdk_runtime.offboard_error) as error:
            return f"land failed: {error}"
        return "land command sent"

    @skill
    async def move(
        self, forward: float = 0.0, left: float = 0.0, up: float = 0.0, yaw_rate: float = 0.0
    ) -> str:
        """Set a body-FLU velocity setpoint.

        Args:
            forward: Forward velocity in meters per second.
            left: Left velocity in meters per second.
            up: Up velocity in meters per second.
            yaw_rate: Counter-clockwise yaw rate in radians per second.
        """
        try:
            velocity = flu_to_frd_body_velocity(forward, left, up, yaw_rate)
        except ValueError as error:
            return f"move failed: {error}"
        try:
            await self._system.offboard.set_velocity_body(
                self._mavsdk_runtime.velocity_body_yawspeed(*velocity)
            )
        except self._mavsdk_runtime.offboard_error as error:
            return f"move failed: {error}"
        return "move command sent"

    @skill
    async def goto(self, north: float, east: float, down: float, yaw: float = 0.0) -> str:
        """Set a local-NED position and yaw target.

        Args:
            north: Target north position in meters.
            east: Target east position in meters.
            down: Target down position in meters.
            yaw: Target clockwise yaw in degrees.
        """
        values = (north, east, down, yaw)
        if not all(math.isfinite(value) for value in values):
            return "goto failed: position and yaw must be finite"
        try:
            await self._system.offboard.set_position_ned(
                self._mavsdk_runtime.position_ned_yaw(*values)
            )
        except self._mavsdk_runtime.offboard_error as error:
            return f"goto failed: {error}"
        return "goto command sent"

    @skill
    async def enter_offboard(self) -> str:
        """Enter Offboard mode with a zero body-velocity setpoint."""
        try:
            await self._system.offboard.set_velocity_body(
                self._mavsdk_runtime.velocity_body_yawspeed(0.0, 0.0, 0.0, 0.0)
            )
            await self._system.offboard.start()
        except self._mavsdk_runtime.offboard_error as error:
            return f"enter offboard failed: {error}"
        self._set_offboard_active(True)
        return "offboard mode entered"

    @skill
    async def exit_offboard(self) -> str:
        """Exit Offboard mode."""
        try:
            await self._system.offboard.stop()
        except self._mavsdk_runtime.offboard_error as error:
            return f"exit offboard failed: {error}"
        self._set_offboard_active(False)
        return "offboard mode exited"

    @skill
    async def hold(self) -> str:
        """Hold the latest local-NED position and yaw target."""
        position = self._position_ned
        yaw = self._yaw_deg
        if position is None or yaw is None:
            return "hold failed: local NED position and yaw are unavailable"
        try:
            await self._system.offboard.set_position_ned(
                self._mavsdk_runtime.position_ned_yaw(*position, yaw)
            )
        except self._mavsdk_runtime.offboard_error as error:
            return f"hold failed: {error}"
        return "hold command sent"

    @skill
    async def hover(self) -> str:
        """Hold the latest local-NED position and yaw target."""
        position = self._position_ned
        yaw = self._yaw_deg
        if position is None or yaw is None:
            return "hover failed: local NED position and yaw are unavailable"
        try:
            await self._system.offboard.set_position_ned(
                self._mavsdk_runtime.position_ned_yaw(*position, yaw)
            )
        except self._mavsdk_runtime.offboard_error as error:
            return f"hover failed: {error}"
        return "hover command sent"

    @rpc
    def start(self) -> None:
        """Connect MAVSDK and start telemetry before accepting stream input."""
        loop = self._loop
        if loop is None or not loop.is_running():
            raise _MavsdkLoopUnavailableError
        started = False
        try:
            asyncio.run_coroutine_threadsafe(self._start_mavsdk(), loop).result()
            super().start()
            started = True
        finally:
            if not started:
                asyncio.run_coroutine_threadsafe(self._stop_mavsdk(), loop).result()

    @rpc
    def stop(self) -> None:
        """Stop telemetry and MAVSDK before closing the Module event loop."""
        loop = self._loop
        try:
            if loop is not None:
                asyncio.run_coroutine_threadsafe(self._stop_mavsdk(), loop).result()
        finally:
            super().stop()

    async def _start_mavsdk(self) -> None:
        system = self._mavsdk_runtime.create_system()
        await system.connect(system_address=self._mavsdk_config.connection_url)
        try:
            await asyncio.wait_for(
                self._wait_until_connected(system),
                timeout=self._mavsdk_config.connection_timeout_s,
            )
        except TimeoutError as error:
            raise MavsdkConnectionTimeoutError(
                connection_url=self._mavsdk_config.connection_url,
                timeout_s=self._mavsdk_config.connection_timeout_s,
            ) from error
        self._system = system
        self._connected = True
        self._telemetry_tasks = [
            asyncio.create_task(self._watch_armed()),
            asyncio.create_task(self._watch_in_air()),
            asyncio.create_task(self._watch_flight_mode()),
            asyncio.create_task(self._watch_position_velocity_ned()),
            asyncio.create_task(self._watch_attitude_euler()),
        ]

    @staticmethod
    async def _wait_until_connected(system: MavsdkSystem) -> None:
        async for state in system.core.connection_state():
            if state.is_connected:
                return

    async def get_status(self) -> str:
        """Return the latest MAVSDK connection and telemetry state as JSON."""
        return json.dumps(
            {
                "connected": self._connected,
                "armed": self._armed,
                "in_air": self._in_air,
                "flight_mode": self._flight_mode,
            },
            sort_keys=True,
        )

    async def handle_cmd_vel(self, twist: Twist) -> None:
        _ = await self.move(twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z)

    async def handle_odometry(self, message: Odometry) -> None:
        try:
            await self._send_odometry(_build_mavsdk_odometry_values(message))
        except self._mavsdk_runtime.mocap_error as error:
            logger.warning("external vision odometry was rejected: %s", error)

    async def _send_odometry(self, values: MavsdkOdometryValues) -> None:
        estimate = self._mavsdk_runtime.vision_position_estimate(values)
        await self._system.mocap.set_vision_position_estimate(estimate)

    async def _watch_armed(self) -> None:
        async for value in self._system.telemetry.armed():
            self._armed = value

    async def _watch_in_air(self) -> None:
        async for value in self._system.telemetry.in_air():
            self._in_air = value

    async def _watch_flight_mode(self) -> None:
        async for value in self._system.telemetry.flight_mode():
            if value is None:
                continue
            self._flight_mode = value.name
            self._offboard_active = value.name == "OFFBOARD"

    async def _watch_position_velocity_ned(self) -> None:
        async for value in self._system.telemetry.position_velocity_ned():
            position = value.position
            self._position_ned = (position.north_m, position.east_m, position.down_m)

    async def _watch_attitude_euler(self) -> None:
        async for value in self._system.telemetry.attitude_euler():
            self._yaw_deg = value.yaw_deg

    async def _stop_mavsdk(self) -> None:
        system = getattr(self, "_system", None)
        if system is None:
            return
        for task in self._telemetry_tasks:
            _ = task.cancel()
        if self._telemetry_tasks:
            _ = await asyncio.gather(*self._telemetry_tasks, return_exceptions=True)
        self._telemetry_tasks.clear()
        if self._offboard_active:
            try:
                await system.offboard.stop()
            except self._mavsdk_runtime.offboard_error as error:
                logger.warning("failed to stop Offboard control during shutdown: %s", error)
        self._connected = False
        self._offboard_active = False
        del self._system
        self._armed = None
        self._in_air = None
        self._flight_mode = None
