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

"""Guarded high-level velocity connection for the Deep Robotics Lynx M20."""

from __future__ import annotations

from datetime import datetime
import json
import socket
import struct
from threading import Condition, RLock
from typing import Any, Literal

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.msgs.std_msgs.UInt32 import UInt32
from dimos.utils.logging_config import setup_logger
from dimos.utils.sequential_ids import SequentialIds

logger = setup_logger()

MOTION_STAND = 1
MOTION_SIT = 4
MOTION_RL_CONTROL = 17

GAIT_BASIC = 0x1001
GAIT_FLAT_AGILE = 0x3002
GAIT_STAIR_AGILE = 0x3003
NavigationTerrain = Literal["flat", "stairs"]
_NAVIGATION_GAITS: dict[NavigationTerrain, int] = {
    "flat": GAIT_FLAT_AGILE,
    "stairs": GAIT_STAIR_AGILE,
}

USAGE_MODE_NAVIGATION = 1

_BASIC_SERVER_MAGIC = bytes.fromhex("eb91eb90")
_BASIC_SERVER_JSON = 1
_BASIC_SERVER_HEADER = struct.Struct("<4sHHB7s")
_BASIC_SERVER_MODE_TYPE = 1101
_BASIC_SERVER_MODE_COMMAND = 5


class M20ConnectionConfig(ModuleConfig):
    """State-transition and basic-server settings for M20 control."""

    stand_timeout_s: float = Field(default=12.0, gt=0.0)
    rl_control_timeout_s: float = Field(default=5.0, gt=0.0)
    gait_timeout_s: float = Field(default=5.0, gt=0.0)
    control_ready_timeout_s: float = Field(default=15.0, gt=0.0)
    basic_server_host: str = "10.21.31.103"
    basic_server_tcp_port: int = Field(default=30001, ge=1, le=65535)
    basic_server_timeout_s: float = Field(default=3.0, gt=0.0)


class M20Connection(Module):
    """Expose the planner-facing M20 command and terrain surface.

    The hardware bridge owns ROS 2/DrDDS and the command watchdog. This module
    remains transport-agnostic: it accepts the standard DimOS ``cmd_vel`` stream,
    rejects it until ``standup()`` has enabled control, and emits ``safe_cmd_vel``
    for the robot-local bridge. The native bridge is the single command-validation
    and velocity-clamping boundary.

    ``standup()`` is the normal one-call operator entry point: it completes the
    vendor state and gait transitions, waits for the guarded command path, and
    enables velocity output. ``set_navigation_terrain()`` is the only exposed
    vendor-specific control and selects one of the documented agile navigation
    gaits without exposing raw gait values.
    """

    config: M20ConnectionConfig

    cmd_vel: In[Twist]
    command_ready: In[Bool]
    motion_state: In[Int32]
    gait_state: In[UInt32]
    odometry: In[Odometry]
    safe_cmd_vel: Out[Twist]
    motion_state_cmd: Out[Int32]
    gait_cmd: Out[UInt32]
    odom: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._state_condition = Condition(self._lock)
        self._commands_enabled = False
        self._command_ready = False
        self._motion_state: int | None = None
        self._gait_state: int | None = None
        self._navigation_terrain: NavigationTerrain = "stairs"
        self._basic_server_message_ids = SequentialIds()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.command_ready.subscribe(self._on_command_ready)))
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))
        self.register_disposable(Disposable(self.motion_state.subscribe(self._on_motion_state)))
        self.register_disposable(Disposable(self.gait_state.subscribe(self._on_gait_state)))
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.safe_cmd_vel.publish(Twist.zero())

    @rpc
    def stop(self) -> None:
        self._disable_commands()
        super().stop()

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Forward velocity to the native validation boundary when enabled.

        ``duration`` is accepted for connection compatibility. Command lifetime
        is enforced by the native bridge's monotonic watchdog.
        """
        del duration
        with self._lock:
            enabled = self._commands_enabled and self._command_ready
        self.safe_cmd_vel.publish(twist if enabled else Twist.zero())
        return enabled

    @rpc
    def stop_movement(self) -> None:
        """Publish an immediate zero velocity without disabling future commands."""
        self.safe_cmd_vel.publish(Twist.zero())

    @rpc
    def standup(self) -> bool:
        """Bring the M20 to a command-enabled, navigation-ready standing state."""
        self._disable_commands()
        if not self._enter_navigation_mode():
            logger.error("M20 basic_server rejected the navigation usage mode")
            return False
        if not self._ensure_rl_control():
            return False

        # Switching usage mode resets the gait to Basic. Confirm that reset,
        # then select the documented autonomous-navigation gait.
        if not self._set_gait_and_wait(GAIT_BASIC):
            logger.error("M20 did not confirm the Basic gait")
            return False
        with self._lock:
            terrain = self._navigation_terrain
        if not self._set_gait_and_wait(_NAVIGATION_GAITS[terrain]):
            logger.error("M20 did not confirm the agile %s navigation gait", terrain)
            return False
        if not self._wait_for_control_readiness(self.config.control_ready_timeout_s):
            logger.error("M20 robot control path did not become ready")
            return False
        return self._enable_commands()

    @rpc
    def liedown(self) -> bool:
        """Disable velocity output and command the M20 to its Sit/prone state."""
        self._disable_commands()
        self.motion_state_cmd.publish(Int32(MOTION_SIT))
        return True

    @rpc
    def set_navigation_terrain(self, terrain: NavigationTerrain) -> bool:
        """Select the agile navigation gait for flat ground or stairs.

        Call this while the robot is stationary. In RL Control the switch is
        applied immediately and confirmed through ``/MOTION_INFO``. Otherwise
        the selection is retained and applied by the next ``standup()``.
        """
        gait = _NAVIGATION_GAITS.get(terrain)
        if gait is None:
            logger.error("Unsupported M20 navigation terrain: %s", terrain)
            return False

        with self._lock:
            if self._motion_state != MOTION_RL_CONTROL:
                self._navigation_terrain = terrain
                logger.info("M20 will select the agile %s gait on next standup", terrain)
                return True
            if self._gait_state == gait:
                self._navigation_terrain = terrain
                return True
            restore_commands = self._commands_enabled

        self._disable_commands()
        switched = self._set_gait_and_wait(gait)
        if switched:
            with self._lock:
                self._navigation_terrain = terrain
            logger.info("M20 selected the agile %s navigation gait", terrain)
        else:
            logger.error("M20 did not confirm the agile %s navigation gait", terrain)

        if restore_commands and not self._enable_commands():
            return False
        return switched

    def _on_command_ready(self, msg: Bool) -> None:
        ready = bool(msg.data)
        with self._state_condition:
            commands_were_enabled = self._commands_enabled
            self._command_ready = ready
            self._state_condition.notify_all()
        if commands_were_enabled and not ready:
            self.safe_cmd_vel.publish(Twist.zero())
            logger.warning("M20 command output temporarily inhibited: robot control path is stale")

    def _on_motion_state(self, msg: Int32) -> None:
        with self._state_condition:
            self._motion_state = int(msg.data)
            self._state_condition.notify_all()

    def _on_gait_state(self, msg: UInt32) -> None:
        with self._state_condition:
            self._gait_state = int(msg.data)
            self._state_condition.notify_all()

    def _on_odometry(self, odometry: Odometry) -> None:
        self.odom.publish(odometry.to_pose_stamped())

    def _ensure_rl_control(self) -> bool:
        with self._lock:
            motion_state = self._motion_state
        if motion_state == MOTION_RL_CONTROL:
            return True
        if motion_state != MOTION_STAND:
            self.motion_state_cmd.publish(Int32(MOTION_STAND))
            if not self._wait_for_motion_state(MOTION_STAND, self.config.stand_timeout_s):
                logger.error("M20 did not confirm Stand before the transition timeout")
                return False
        self.motion_state_cmd.publish(Int32(MOTION_RL_CONTROL))
        if not self._wait_for_motion_state(MOTION_RL_CONTROL, self.config.rl_control_timeout_s):
            logger.error("M20 did not confirm RL Control after standing")
            return False
        return True

    def _set_gait_and_wait(self, gait: int) -> bool:
        self.gait_cmd.publish(UInt32(gait))
        return self._wait_for_gait_state(gait, self.config.gait_timeout_s)

    def _enter_navigation_mode(self) -> bool:
        response = self._basic_server_request(
            message_type=_BASIC_SERVER_MODE_TYPE,
            command=_BASIC_SERVER_MODE_COMMAND,
            items={"Mode": USAGE_MODE_NAVIGATION},
        )
        try:
            error_code = int(response["PatrolDevice"]["Items"]["ErrorCode"])
        except (KeyError, TypeError, ValueError):
            logger.error("M20 basic_server returned an invalid usage-mode response")
            return False
        if error_code != 0:
            logger.error("M20 basic_server usage-mode switch failed: error %s", error_code)
            return False
        return True

    def _enable_commands(self) -> bool:
        with self._lock:
            if not self._command_ready:
                logger.warning("M20 command gate refused enable: robot control path is not ready")
                return False
            self._commands_enabled = True
        logger.info("M20 command output enabled")
        return True

    def _disable_commands(self) -> None:
        with self._lock:
            self._commands_enabled = False
        self.safe_cmd_vel.publish(Twist.zero())
        logger.info("M20 command output disabled")

    def _basic_server_request(
        self,
        *,
        message_type: int,
        command: int,
        items: dict[str, Any],
    ) -> dict[str, Any]:
        payload = json.dumps(
            {
                "PatrolDevice": {
                    "Type": message_type,
                    "Command": command,
                    "Time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "Items": items,
                }
            },
            separators=(",", ":"),
        ).encode()
        if len(payload) > 0xFFFF:
            raise ValueError("M20 basic_server payload exceeds the APDU limit")

        message_id = self._basic_server_message_ids.next() & 0xFFFF
        header = _BASIC_SERVER_HEADER.pack(
            _BASIC_SERVER_MAGIC,
            len(payload),
            message_id,
            _BASIC_SERVER_JSON,
            b"\0" * 7,
        )

        try:
            with socket.create_connection(
                (self.config.basic_server_host, self.config.basic_server_tcp_port),
                timeout=self.config.basic_server_timeout_s,
            ) as connection:
                connection.settimeout(self.config.basic_server_timeout_s)
                connection.sendall(header + payload)
                response_header = _recv_exact(connection, _BASIC_SERVER_HEADER.size)
                magic, length, response_id, encoding, _reserved = _BASIC_SERVER_HEADER.unpack(
                    response_header
                )
                if magic != _BASIC_SERVER_MAGIC:
                    raise ValueError("invalid APDU magic")
                if response_id != message_id:
                    raise ValueError("response APDU message ID does not match request")
                if encoding != _BASIC_SERVER_JSON:
                    raise ValueError("basic_server response is not JSON")
                response_payload = _recv_exact(connection, length)
        except (OSError, ValueError) as exc:
            logger.error("M20 basic_server request failed: %s", exc)
            return {}

        try:
            decoded = json.loads(response_payload)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            logger.error("M20 basic_server returned invalid JSON: %s", exc)
            return {}
        return decoded if isinstance(decoded, dict) else {}

    def _wait_for_motion_state(self, expected: int, timeout_s: float) -> bool:
        with self._state_condition:
            return self._state_condition.wait_for(
                lambda: self._motion_state == expected, timeout=timeout_s
            )

    def _wait_for_gait_state(self, expected: int, timeout_s: float) -> bool:
        with self._state_condition:
            return self._state_condition.wait_for(
                lambda: self._gait_state == expected, timeout=timeout_s
            )

    def _wait_for_control_readiness(self, timeout_s: float) -> bool:
        with self._state_condition:
            return self._state_condition.wait_for(lambda: self._command_ready, timeout=timeout_s)


def _recv_exact(connection: socket.socket, size: int) -> bytes:
    result = bytearray()
    while len(result) < size:
        chunk = connection.recv(size - len(result))
        if not chunk:
            raise OSError("basic_server closed the connection before completing its response")
        result.extend(chunk)
    return bytes(result)
