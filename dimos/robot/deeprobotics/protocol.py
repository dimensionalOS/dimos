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

"""UDP protocol utility for Deep Robotics quadrupeds (M20/Lynx series).

Binary protocol format (16-byte header + JSON payload):
    Header: 0xEB 0x91 0xEB 0x90 | data_len (u16 LE) | msg_id (u16 LE) | format (1) | padding (7)
    Payload: UTF-8 JSON with PatrolDevice wrapper
"""

import json
import logging
import socket
import struct
import threading
import time
from typing import Callable, Optional

logger = logging.getLogger(__name__)

# Binary protocol constants
HEADER_MAGIC = bytes([0xEB, 0x91, 0xEB, 0x90])
HEADER_LEN = 16
JSON_FORMAT = 0x01


class CommandType:
    HEARTBEAT = 100
    MOTION = 2
    PERIPHERAL = 1101


class Command:
    HEARTBEAT = 100
    VELOCITY = 21
    MOTION_STATE = 22
    GAIT_SWITCH = 23
    CHARGING = 24
    USAGE_MODE = 5
    SLEEP = 6
    FLASHLIGHT = 2


class MotionState:
    STAND = 1
    SIT = 4


class GaitType:
    STANDARD = 0x1001
    HIGH_OBSTACLE = 0x1002
    STAIRS = 0x1003


class UsageMode:
    REGULAR = 0
    NAVIGATION = 1
    ASSIST = 2


class M20Protocol:
    """Low-level UDP protocol for Deep Robotics M20 quadruped.

    Utility class (not a dimos Module) that handles encoding/decoding the
    binary header and JSON payload for the M20's basic_server UDP interface.
    Includes heartbeat management and status response listener.
    """

    def __init__(self, host: str = "10.21.31.103", port: int = 30000):
        self.host = host
        self.port = port
        self._sock: Optional[socket.socket] = None
        self._msg_id: int = 0
        self._listener_thread: Optional[threading.Thread] = None
        self._listener_running: bool = False
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._heartbeat_running: bool = False

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> None:
        """Create the UDP socket."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(1.0)
        logger.info(f"M20 UDP protocol ready — target {self.host}:{self.port}")

    def close(self) -> None:
        """Close the UDP socket and stop all background threads."""
        self.stop_heartbeat()
        self.stop_listener()
        if self._sock:
            self._sock.close()
            self._sock = None
            logger.info("M20 UDP protocol closed")

    def _send_command(self, cmd_type: int, command: int, items: dict) -> None:
        """Send a UDP command with 16-byte header + JSON payload."""
        if not self._sock:
            return

        payload = json.dumps({
            "PatrolDevice": {
                "Type": cmd_type,
                "Command": command,
                "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "Items": items,
            }
        }).encode("utf-8")

        data_len = len(payload)
        self._msg_id = (self._msg_id + 1) % 65536
        header = (
            HEADER_MAGIC
            + struct.pack("<H", data_len)
            + struct.pack("<H", self._msg_id)
            + bytes([JSON_FORMAT])
            + bytes(7)
        )

        self._sock.sendto(header + payload, (self.host, self.port))

    # --- Motion commands ---

    def send_velocity(
        self,
        x: float,
        y: float,
        yaw: float,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
    ) -> None:
        """Send velocity command (Type=2, Cmd=21) with normalized [-1,1] values."""
        def _clamp(v: float) -> float:
            return round(max(-1.0, min(1.0, v)), 4)

        self._send_command(CommandType.MOTION, Command.VELOCITY, {
            "X": _clamp(x),
            "Y": _clamp(y),
            "Z": _clamp(z),
            "Roll": _clamp(roll),
            "Pitch": _clamp(pitch),
            "Yaw": _clamp(yaw),
        })

    def send_motion_state(self, motion_param: int) -> None:
        """Send motion state switch (Type=2, Cmd=22). Stand=1, Sit=4."""
        self._send_command(CommandType.MOTION, Command.MOTION_STATE, {
            "MotionParam": motion_param,
        })
        logger.info(f"Motion state switch: MotionParam={motion_param}")

    def send_gait_switch(self, gait_param: int) -> None:
        """Send gait switch (Type=2, Cmd=23). Standard=0x1001, HighObs=0x1002, Stairs=0x1003."""
        self._send_command(CommandType.MOTION, Command.GAIT_SWITCH, {
            "GaitParam": gait_param,
        })
        logger.info(f"Gait switch: GaitParam=0x{gait_param:04X}")

    def send_charging_command(self, charge: int) -> None:
        """Send charging command (Type=2, Cmd=24). Start=1, Stop=0, Clear=2."""
        self._send_command(CommandType.MOTION, Command.CHARGING, {
            "Charge": charge,
        })
        logger.info(f"Charging command: Charge={charge}")

    # --- Peripheral commands ---

    def send_usage_mode(self, mode: int) -> None:
        """Send usage mode switch (Type=1101, Cmd=5). Regular=0, Navigation=1, Assist=2."""
        self._send_command(CommandType.PERIPHERAL, Command.USAGE_MODE, {
            "Mode": mode,
        })
        logger.info(f"Usage mode switch: Mode={mode}")

    def send_sleep(self, sleep: bool, auto: bool = False, timeout: int = 30) -> None:
        """Send sleep command (Type=1101, Cmd=6)."""
        self._send_command(CommandType.PERIPHERAL, Command.SLEEP, {
            "Sleep": sleep,
            "Auto": auto,
            "Time": timeout,
        })
        logger.info(f"Sleep command: Sleep={sleep}, Auto={auto}")

    def send_flashlight(self, front: int, back: int = 0) -> None:
        """Send flashlight control (Type=1101, Cmd=2). On=1, Off=0."""
        self._send_command(CommandType.PERIPHERAL, Command.FLASHLIGHT, {
            "Front": front,
            "Back": back,
        })
        logger.info(f"Flashlight: Front={front}, Back={back}")

    # --- Heartbeat ---

    def send_heartbeat(self) -> None:
        """Send heartbeat (Type=100, Cmd=100). Required at >=1Hz for status reports."""
        self._send_command(CommandType.HEARTBEAT, Command.HEARTBEAT, {})

    def start_heartbeat(self, interval: float = 0.5) -> None:
        """Start background heartbeat thread at the given interval (seconds)."""
        if self._heartbeat_thread and self._heartbeat_running:
            return
        self._heartbeat_running = True
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            args=(interval,),
            daemon=True,
        )
        self._heartbeat_thread.start()
        logger.info(f"Heartbeat started at {1 / interval:.1f}Hz")

    def stop_heartbeat(self) -> None:
        """Stop the heartbeat thread."""
        self._heartbeat_running = False
        if self._heartbeat_thread:
            self._heartbeat_thread.join(timeout=2.0)
            self._heartbeat_thread = None

    def _heartbeat_loop(self, interval: float) -> None:
        while self._heartbeat_running and self._sock:
            self.send_heartbeat()
            time.sleep(interval)

    # --- Status listener ---

    def start_listener(self, callback: Callable[[dict], None]) -> None:
        """Start background thread to receive UDP status responses from M20.

        Callback receives dicts with keys: type, command, items.
        """
        if self._listener_thread and self._listener_running:
            return
        self._listener_running = True
        self._listener_thread = threading.Thread(
            target=self._listen_loop,
            args=(callback,),
            daemon=True,
        )
        self._listener_thread.start()
        logger.info("UDP listener started")

    def stop_listener(self) -> None:
        """Stop the UDP listener thread."""
        self._listener_running = False
        if self._listener_thread:
            self._listener_thread.join(timeout=2.0)
            self._listener_thread = None

    def _listen_loop(self, callback: Callable[[dict], None]) -> None:
        while self._listener_running and self._sock:
            try:
                data, _ = self._sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(data) < HEADER_LEN:
                continue
            if data[:4] != HEADER_MAGIC:
                continue

            payload_len = struct.unpack("<H", data[4:6])[0]
            payload_bytes = data[HEADER_LEN : HEADER_LEN + payload_len]

            try:
                msg = json.loads(payload_bytes)
                device = msg.get("PatrolDevice", {})
                callback({
                    "type": device.get("Type"),
                    "command": device.get("Command"),
                    "items": device.get("Items", {}),
                })
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue
