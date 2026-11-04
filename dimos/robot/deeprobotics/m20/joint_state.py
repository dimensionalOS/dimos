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

"""The M20's measured joint angles, from basic_server's MotionStatus reports (Type 1002, Command 4, 10 Hz)."""

import math
import socket
import threading
import time
from typing import Any

from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.deeprobotics.m20 import basic_server
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

HEARTBEAT_TYPE = 100
HEARTBEAT_COMMAND = 100
REPORT_TYPE = 1002
MOTION_STATUS_COMMAND = 4

# basic_server's "Left" is the battery side and "Back" the hard e-stop side.
_LEGS = {"fl": "LeftFront", "fr": "RightFront", "hl": "LeftBack", "hr": "RightBack"}
_JOINTS = ("hipx", "hipy", "knee", "wheel")
_REPORT_JOINTS = ("HipX", "HipY", "Knee", "Wheel")
# The report holds raw motor angles; the vendor's M20_sdk_deploy (m20_interface.hpp) maps them to the URDF as
# raw * direction + zero offset, per leg in hipx/hipy/knee/wheel order.
_DIRECTION = {
    "fl": (1, 1, -1, 1),
    "fr": (1, -1, 1, -1),
    "hl": (-1, 1, -1, 1),
    "hr": (-1, -1, 1, -1),
}
_ZERO_OFFSET_DEG = {
    "fl": (-25, -131, 160),
    "fr": (25, -131, 160),
    "hl": (-25, 131, -160),
    "hr": (25, 131, -160),
}


def urdf_joints_from_motor_status(
    motor_status: dict[str, Any],
) -> tuple[dict[str, float], dict[str, float]]:
    """URDF joint angles (rad) for the hips and knees, and wheel speeds (rad/s), from a report's MotorStatus."""
    angles: dict[str, float] = {}
    wheel_speeds: dict[str, float] = {}
    for leg, side in _LEGS.items():
        for index, (joint, report_joint) in enumerate(zip(_JOINTS, _REPORT_JOINTS, strict=True)):
            value = float(motor_status[f"{side}{report_joint}"]) * _DIRECTION[leg][index]
            if joint == "wheel":
                wheel_speeds[f"{leg}_wheel_joint"] = value
            else:
                # The motor encoders are single-turn, so wrap into (-pi, pi] like the vendor's offset reset does.
                angle = value + math.radians(_ZERO_OFFSET_DEG[leg][index])
                angles[f"{leg}_{joint}_joint"] = math.remainder(angle, math.tau)
    return angles, wheel_speeds


class M20JointStateConfig(ModuleConfig):
    basic_server_host: str = "10.21.31.103"
    basic_server_udp_port: int = Field(default=30000, ge=1, le=65535)
    heartbeat_period_s: float = Field(default=0.5, gt=0.0)


class M20JointState(Module):
    """Heartbeats basic_server (the only frame it ever sends) and publishes the joint angles it reports."""

    config: M20JointStateConfig

    coordinator_joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="m20-joint-state", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        super().stop()

    def _run(self) -> None:
        target = (self.config.basic_server_host, self.config.basic_server_udp_port)
        wheel_angles = {f"{leg}_wheel_joint": 0.0 for leg in _LEGS}
        last_report_at: float | None = None
        message_id = 0
        next_heartbeat_at = 0.0
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            udp.bind(("0.0.0.0", 0))
            udp.settimeout(0.1)
            while not self._stop_event.is_set():
                if time.monotonic() >= next_heartbeat_at:
                    message_id += 1
                    try:
                        udp.sendto(
                            basic_server.encode_frame(
                                HEARTBEAT_TYPE, HEARTBEAT_COMMAND, {}, message_id
                            ),
                            target,
                        )
                    except OSError as exc:
                        logger.warning("M20 basic_server heartbeat failed: %s", exc)
                    next_heartbeat_at = time.monotonic() + self.config.heartbeat_period_s
                try:
                    frame = udp.recv(65535)
                except OSError:
                    continue
                device = basic_server.decode_frame(frame)
                if (
                    device is None
                    or device.get("Type") != REPORT_TYPE
                    or device.get("Command") != MOTION_STATUS_COMMAND
                ):
                    continue
                try:
                    angles, wheel_speeds = urdf_joints_from_motor_status(
                        device["Items"]["MotorStatus"]
                    )
                except (KeyError, TypeError, ValueError) as exc:
                    logger.warning("M20 MotionStatus report without usable MotorStatus: %s", exc)
                    continue
                now = time.monotonic()
                # The report gives wheel speed, not angle, so the wheel angle is its integral since start.
                if last_report_at is not None:
                    for joint, speed in wheel_speeds.items():
                        wheel_angles[joint] = math.remainder(
                            wheel_angles[joint] + speed * (now - last_report_at), math.tau
                        )
                last_report_at = now
                positions = {**angles, **wheel_angles}
                self.coordinator_joint_state.publish(
                    JointState(
                        ts=time.time(), name=list(positions), position=list(positions.values())
                    )
                )
