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

"""Minimal Boston Dynamics Spot velocity control via the `bosdyn-client` SDK.

A single async `Module`: `main()` acquires the lease/E-stop, powers on, and
stands the robot, then hands control to the `cmd_vel` stream. Each Twist is
forwarded to Spot as a synchronized body velocity command. Teardown sits the
robot and powers the motors back off.

`bosdyn` is an optional extra (`uv sync --extra spot`); its imports live inside
`main()` so this file stays importable — and blueprint discovery keeps working —
on hosts where the SDK isn't installed.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from dataclasses import field
import os
import time
from typing import Any

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_POWER_ON_TIMEOUT_S = 20.0
_POWER_OFF_TIMEOUT_S = 20.0
_STAND_TIMEOUT_S = 10.0
_SIT_TIMEOUT_S = 10.0

# Spot's fixed default addresses: 192.168.80.3 when it hosts its own WiFi AP,
# 10.0.0.3 on the rear Ethernet port. Probed in order when no ip is given.
_SPOT_WIFI_AP_IP = "192.168.80.3"
_SPOT_ETHERNET_IP = "10.0.0.3"
_IP_LABELS = {_SPOT_WIFI_AP_IP: "WiFi", _SPOT_ETHERNET_IP: "Ethernet"}
# Spot's gRPC API listens on HTTPS/443; a TCP connect confirms real reachability
# (and matches what the SDK does) better than an ICMP ping.
_SPOT_API_PORT = 443
_REACHABILITY_PROBE_TIMEOUT_S = 2.0


class SpotCmdVelConfig(ModuleConfig):
    """Connection, credentials, and safety gating for a Spot robot."""

    # Explicit address always wins (`-o spotcmdvel.ip=<addr>`). When left blank,
    # main() probes `candidate_ips` and uses the first that answers on the API
    # port — so plugging in over Ethernet or joining Spot's WiFi both "just work".
    ip: str = ""
    candidate_ips: list[str] = field(default_factory=lambda: [_SPOT_WIFI_AP_IP, _SPOT_ETHERNET_IP])

    # Auth — falls back to BOSDYN_CLIENT_USERNAME / BOSDYN_CLIENT_PASSWORD at
    # start time when left as None. Startup fails fast if neither is supplied.
    username: str | None = None
    password: str | None = None

    # Safety / startup gating.
    enable_estop: bool = True
    acquire_lease: bool = True
    power_on_at_start: bool = True
    stand_at_start: bool = True

    # Spot rejects velocity commands without an `end_time_secs`. This duration
    # is added to now() for each command and doubles as the auto-stop window:
    # if the cmd_vel stream stalls for longer than this, the robot halts.
    cmd_vel_timeout: float = 0.5

    # Spot E-stops itself if it doesn't see a keep-alive check-in within this
    # window. 9.0 s matches the bosdyn-client default.
    estop_timeout: float = 9.0


class SpotCmdVel(Module):
    """Drives a Boston Dynamics Spot from a `cmd_vel` Twist stream."""

    # A hardware-driving module gets its own worker process, matching the other
    # robot connection modules (go2/b1/drone). Sharing a process with a GUI
    # module like KeyboardTeleop wedges this module's RPC server at startup.
    dedicated_worker = True

    cmd_vel: In[Twist]

    config: SpotCmdVelConfig

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._robot: Any = None
        self._command_client: Any = None
        self._state_client: Any = None
        self._estop_keepalive: Any = None
        self._lease_keepalive: Any = None
        self._standing = False
        # cmd_vel handlers wait on this so a velocity command issued mid-setup
        # never reaches a half-initialised SDK.
        self._ready = asyncio.Event()

    async def main(self) -> AsyncIterator[None]:
        username, password = self._resolve_credentials()
        ip = self.config.ip or await self._resolve_ip()

        from bosdyn.client import create_standard_sdk  # type: ignore[import-not-found]
        from bosdyn.client.estop import (  # type: ignore[import-not-found]
            EstopClient,
            EstopEndpoint,
            EstopKeepAlive,
        )
        from bosdyn.client.lease import (  # type: ignore[import-not-found]
            LeaseClient,
            LeaseKeepAlive,
        )
        from bosdyn.client.robot_command import (  # type: ignore[import-not-found]
            RobotCommandClient,
            blocking_stand,
        )
        from bosdyn.client.robot_state import (  # type: ignore[import-not-found]
            RobotStateClient,
        )

        logger.info(f"Connecting to Spot at {ip}")
        sdk = await asyncio.to_thread(create_standard_sdk, "dimos-spot")
        self._robot = await asyncio.to_thread(sdk.create_robot, ip)
        await asyncio.to_thread(self._robot.authenticate, username, password)
        await asyncio.to_thread(self._robot.time_sync.wait_for_sync)

        if self.config.enable_estop:
            estop_client = self._robot.ensure_client(EstopClient.default_service_name)
            endpoint = EstopEndpoint(
                client=estop_client,
                name="dimos-spot",
                estop_timeout=self.config.estop_timeout,
            )
            await asyncio.to_thread(endpoint.force_simple_setup)
            self._estop_keepalive = EstopKeepAlive(endpoint)

        if self.config.acquire_lease:
            lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
            await asyncio.to_thread(lease_client.take)
            self._lease_keepalive = LeaseKeepAlive(lease_client)

        self._command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
        self._state_client = self._robot.ensure_client(RobotStateClient.default_service_name)

        if self.config.power_on_at_start:
            logger.info("Powering on Spot motors")
            await asyncio.to_thread(self._robot.power_on, timeout_sec=_POWER_ON_TIMEOUT_S)

        if self.config.stand_at_start:
            logger.info("Standing Spot")
            await asyncio.to_thread(
                blocking_stand, self._command_client, timeout_sec=_STAND_TIMEOUT_S
            )
            self._standing = True

        self._ready.set()
        logger.info("Spot cmd_vel control ready")

        yield

        self._ready.clear()
        if self._standing:
            try:
                from bosdyn.client.robot_command import (  # type: ignore[import-not-found]
                    blocking_sit,
                )

                await asyncio.to_thread(
                    blocking_sit, self._command_client, timeout_sec=_SIT_TIMEOUT_S
                )
            except Exception as error:
                logger.error(f"Spot sit during teardown failed: {error}")
            self._standing = False

        if self.config.power_on_at_start and self._robot is not None:
            try:
                await asyncio.to_thread(
                    self._robot.power_off, cut_immediately=False, timeout_sec=_POWER_OFF_TIMEOUT_S
                )
            except Exception as error:
                logger.error(f"Spot power_off during teardown failed: {error}")

        for keepalive_name, keepalive in (
            ("lease", self._lease_keepalive),
            ("estop", self._estop_keepalive),
        ):
            if keepalive is not None:
                try:
                    await asyncio.to_thread(keepalive.shutdown)
                except Exception as error:
                    logger.error(f"Spot {keepalive_name} shutdown failed: {error}")
        self._lease_keepalive = None
        self._estop_keepalive = None
        logger.info("Spot cmd_vel control torn down")

    async def handle_cmd_vel(self, msg: Twist) -> None:
        if not self._ready.is_set():
            return
        await self._send_velocity(msg.linear.x, msg.linear.y, msg.angular.z)

    async def _send_velocity(
        self, forward: float, strafe: float, yaw: float, duration: float = 0.0
    ) -> bool:
        from bosdyn.client.robot_command import (  # type: ignore[import-not-found]
            RobotCommandBuilder,
        )

        command = RobotCommandBuilder.synchro_velocity_command(v_x=forward, v_y=strafe, v_rot=yaw)
        window = duration if duration > 0 else self.config.cmd_vel_timeout
        try:
            await asyncio.to_thread(
                self._command_client.robot_command,
                command,
                end_time_secs=time.time() + window,
            )
            return True
        except Exception as error:
            logger.error(f"Spot velocity command failed: {error}")
            return False

    @rpc
    async def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send a Twist as a body velocity command, optionally for `duration` seconds."""
        return await self._send_velocity(twist.linear.x, twist.linear.y, twist.angular.z, duration)

    @rpc
    async def get_state(self) -> str:
        if self._state_client is None:
            return "DISCONNECTED"
        try:
            state = await asyncio.to_thread(self._state_client.get_robot_state)
            return str(state.power_state.motor_power_state)
        except Exception as error:
            logger.error(f"Spot get_state failed: {error}")
            return "UNKNOWN"

    @skill
    async def move_velocity(
        self, x: float, y: float = 0.0, yaw: float = 0.0, duration: float = 0.0
    ) -> str:
        """Move Spot with a direct body velocity command.

        Args:
            x: Forward velocity (m/s).
            y: Left/right velocity (m/s).
            yaw: Rotational velocity (rad/s).
            duration: Seconds to move. 0 uses one `cmd_vel_timeout` window.
        """
        twist = Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw))
        if await self.move(twist, duration=duration):
            return f"Moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"
        return f"Failed to move with velocity=({x}, {y}, {yaw})"

    @skill
    async def stand(self) -> str:
        """Make Spot stand up. Spot must already be powered on."""
        if self._command_client is None:
            return "Spot is not connected."
        try:
            from bosdyn.client.robot_command import (  # type: ignore[import-not-found]
                blocking_stand,
            )

            await asyncio.to_thread(
                blocking_stand, self._command_client, timeout_sec=_STAND_TIMEOUT_S
            )
            self._standing = True
            return "Spot is standing."
        except Exception as error:
            logger.error(f"Spot stand failed: {error}")
            return f"Stand failed: {error}"

    @skill
    async def sit(self) -> str:
        """Make Spot sit down."""
        if self._command_client is None:
            return "Spot is not connected."
        try:
            from bosdyn.client.robot_command import (  # type: ignore[import-not-found]
                blocking_sit,
            )

            await asyncio.to_thread(blocking_sit, self._command_client, timeout_sec=_SIT_TIMEOUT_S)
            self._standing = False
            return "Spot is sitting."
        except Exception as error:
            logger.error(f"Spot sit failed: {error}")
            return f"Sit failed: {error}"

    async def _resolve_ip(self) -> str:
        for candidate in self.config.candidate_ips:
            if await self._is_reachable(candidate):
                logger.info(f"Spot reachable at {candidate}")
                return candidate
        described = " or ".join(
            f"{candidate} ({_IP_LABELS[candidate]})" if candidate in _IP_LABELS else candidate
            for candidate in self.config.candidate_ips
        )
        raise ConnectionError(
            f"I'm unable to connect to {described}. Did you forget to connect to "
            "Spot's WiFi or plug in an Ethernet cable to Spot?"
        )

    async def _is_reachable(self, ip: str) -> bool:
        try:
            _, writer = await asyncio.wait_for(
                asyncio.open_connection(ip, _SPOT_API_PORT),
                timeout=_REACHABILITY_PROBE_TIMEOUT_S,
            )
        except (OSError, asyncio.TimeoutError):
            return False
        # Reachable — a successful TCP handshake is all we need. Close without
        # awaiting wait_closed(); the API port speaks TLS and never completes a
        # clean plaintext close, which would otherwise hang the probe.
        writer.close()
        return True

    def _resolve_credentials(self) -> tuple[str, str]:
        username = self.config.username or os.environ.get("BOSDYN_CLIENT_USERNAME")
        password = self.config.password or os.environ.get("BOSDYN_CLIENT_PASSWORD")
        if not username or not password:
            raise ValueError(
                "Spot credentials missing — set username/password in config "
                "or BOSDYN_CLIENT_USERNAME / BOSDYN_CLIENT_PASSWORD env vars"
            )
        return username, password


__all__ = ["SpotCmdVel", "SpotCmdVelConfig"]
