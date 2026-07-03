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

"""DeepRobotics Lynx M20 connection: video (RTSP) + teleop (cmd_vel).

The M20 exposes a high-level "Inspection / PatrolDevice" protocol over UDP
(`10.21.31.103:30000`) for control + telemetry, and RTSP for the wide-angle
cameras. This module wires just the two things we care about into dimos:

  * ``color_image: Out[Image]``  <- front RTSP camera
  * ``cmd_vel: In[Twist]``       -> normalized axis-velocity commands

It does NOT touch the onboard lidar / SLAM (those live in the robot's ROS 2
stack, not this protocol). See the M20 Software Development Manual V0.0.4.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
import json
import os
import socket
import struct
import threading
import time
from typing import TYPE_CHECKING

import cv2
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport, pSHMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.BatteryState import (
    POWER_SUPPLY_STATUS_CHARGING,
    POWER_SUPPLY_STATUS_DISCHARGING,
    BatteryState,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.deeprobotics.m20.msgs.status import M20Status
from dimos.spec.perception import Image as VideoSource
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.core.rpc_client import ModuleProxy

logger = setup_logger()

# --- Patrol protocol (manual §1.1) --------------------------------------------

# 16-byte header sync word. The spec table (§1.1.5) gives EB 91 EB 90, but the
# appendix C sample writes EB 90 EB 90. We trust the spec table; if the robot
# rejects frames, switch to _SYNC_ALT. TODO: confirm against real hardware.
_SYNC = b"\xeb\x91\xeb\x90"
_SYNC_ALT = b"\xeb\x90\xeb\x90"
_FMT_JSON = 0x01

# (Type, Command) message ids — commands we send.
_HEARTBEAT = (100, 100)
_USAGE_MODE = (1101, 5)
_MOTION_STATE = (2, 22)
_GAIT = (2, 23)
_MOVE = (2, 21)

# (Type, Command) reports the robot pushes.
_REPORT_DEVICE_STATE = (1002, 5)  # batteries, temps, GPS, DevEnable
_REPORT_BASIC_STATUS = (1002, 6)  # state-machine enums

# MotionParam / GaitParam / Mode enum values.
_STATE_STAND = 1
_STATE_STANDARD = 6
_GAIT_BASIC = 1
_MODE_REGULAR = 0


def _clip(v: float) -> float:
    return max(-1.0, min(1.0, v))


def _num(items: Mapping[str, object], key: str) -> float:
    v = items.get(key)
    return float(v) if isinstance(v, (int, float)) else 0.0


def _battery_from(items: Mapping[str, object], side: str, location: str) -> BatteryState:
    """Build a native BatteryState from one side of the M20 BatteryStatus group."""
    serial = items.get(f"serial{side}")
    return BatteryState(
        voltage=_num(items, f"Voltage{side}"),
        temperature=_num(items, f"battery_temperature{side}"),
        percentage=_num(items, f"BatteryLevel{side}") / 100.0,
        power_supply_status=(
            POWER_SUPPLY_STATUS_CHARGING
            if items.get(f"charge{side}")
            else POWER_SUPPLY_STATUS_DISCHARGING
        ),
        present=True,
        location=location,
        serial_number=serial if isinstance(serial, str) else "",
    )


def _status_from(bs: Mapping[str, object]) -> M20Status:
    """Build an M20Status from the BasicStatus group."""

    def i(key: str) -> int:
        v = bs.get(key)
        return int(v) if isinstance(v, (int, float)) else 0

    version = bs.get("Version")
    return M20Status(
        motion_state=i("MotionState"),
        gait=i("Gait"),
        control_mode=i("ControlUsageMode"),
        charge_state=i("Charge"),
        direction=i("Direction"),
        status_code=i("StatusCode"),
        hard_estop=bool(i("HES")),
        sleep=bool(bs.get("Sleep")),
        version=version if isinstance(version, str) else "",
    )


def _axes_from_twist(twist: Twist, max_linear: float, max_angular: float) -> dict[str, float]:
    """Map a velocity Twist (m/s, rad/s) to the M20's normalized [-1, 1] axis command.

    linear.x -> X (forward), linear.y -> Y (left), angular.z -> Yaw. Sign of Y/Yaw
    relative to the robot frame is unverified; flip here if it strafes/turns wrong.
    Z/Roll/Pitch are unused in Basic/Stair gait so they're held at 0.
    """
    return {
        "X": _clip(twist.linear.x / max_linear),
        "Y": _clip(twist.linear.y / max_linear),
        "Z": 0.0,
        "Roll": 0.0,
        "Pitch": 0.0,
        "Yaw": _clip(twist.angular.z / max_angular),
    }


def _apply_deadman(cmd: Twist | None, age: float, timeout: float) -> Twist:
    """Return the command to send: zero if it's missing or staler than ``timeout``."""
    if cmd is None or age > timeout:
        return Twist.zero()
    return cmd


class PatrolLink:
    """Minimal UDP client for the M20 'PatrolDevice' protocol.

    Owns the socket, the heartbeat (which is what subscribes us to telemetry),
    and frame (de)coding. Control is fire-and-forget; telemetry is drained on a
    background thread so the socket buffer can't fill, and abnormal-status
    reports are logged.
    """

    def __init__(self, ip: str, port: int = 30000, *, sync: bytes = _SYNC) -> None:
        self.ip = ip
        self.port = port
        self._sync = sync
        self._sock: socket.socket | None = None
        self._msg_id = 0
        self._lock = threading.Lock()
        self._running = False
        self._threads: list[threading.Thread] = []
        self.latest_status: dict[str, object] = {}
        # Invoked on the recv thread for every parsed report: (Type, Command, Items).
        self.on_report: Callable[[int, int, Mapping[str, object]], None] | None = None

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("", 0))  # ephemeral local port; robot reports back here
        self._sock.settimeout(0.2)  # bounds recv-loop shutdown latency
        self._running = True
        for target in (self._heartbeat_loop, self._recv_loop):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)

    def stop(self) -> None:
        self._running = False
        for t in self._threads:
            t.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._threads.clear()
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def _encode(self, payload: bytes) -> bytes:
        with self._lock:
            mid = self._msg_id
            self._msg_id = (self._msg_id + 1) & 0xFFFF
        header = bytearray(16)
        header[0:4] = self._sync
        struct.pack_into("<H", header, 4, len(payload))  # ASDU length, LE
        struct.pack_into("<H", header, 6, mid)  # message id, LE
        header[8] = _FMT_JSON  # bytes 9-15 reserved (zero)
        return bytes(header) + payload

    def send(self, type_: int, command: int, items: Mapping[str, object] | None = None) -> None:
        if self._sock is None:
            return
        asdu = {
            "PatrolDevice": {
                "Type": type_,
                "Command": command,
                "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "Items": dict(items or {}),
            }
        }
        payload = json.dumps(asdu).encode("utf-8")
        try:
            self._sock.sendto(self._encode(payload), (self.ip, self.port))
        except OSError as e:
            logger.warning("M20 send failed: %s", e)

    def _heartbeat_loop(self) -> None:
        # Manual recommends >=1 Hz; 2 Hz gives margin and keeps telemetry flowing.
        while self._running:
            self.send(*_HEARTBEAT)
            time.sleep(0.5)

    def _recv_loop(self) -> None:
        while self._running:
            try:
                data, _ = self._sock.recvfrom(65535)  # type: ignore[union-attr]
            except TimeoutError:
                continue
            except OSError:
                break
            self._handle(data)

    def _handle(self, data: bytes) -> None:
        if len(data) < 16 or data[8] != _FMT_JSON:
            return  # only JSON telemetry is parsed here
        try:
            decoded = json.loads(data[16:].decode("utf-8", "ignore"))
        except ValueError:
            return
        if not isinstance(decoded, dict):
            return
        asdu = decoded.get("PatrolDevice")
        items = asdu.get("Items") if isinstance(asdu, dict) else None
        if not isinstance(items, dict):
            return
        status = items.get("BasicStatus")
        if isinstance(status, dict):
            self.latest_status = status
        errors = items.get("ErrorList")
        if errors:
            logger.warning("M20 abnormal status: %s", errors)
        if self.on_report is not None and isinstance(asdu, dict):
            self.on_report(int(asdu.get("Type", 0)), int(asdu.get("Command", 0)), items)


# --- dimos module -------------------------------------------------------------


class M20Config(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip or "10.21.31.103")
    # Wide-angle cameras. Default to the control-channel host (front = video1,
    # rear = video2); override only to point at a different stream.
    rtsp_url: str | None = None
    rtsp_url_rear: str | None = None
    # cmd_vel -> axis command is normalized to [-1, 1] as a fraction of max speed.
    max_linear: float = 1.0  # m/s that maps to |X| = |Y| = 1
    max_angular: float = 1.5  # rad/s that maps to |Yaw| = 1
    cmd_rate_hz: float = 20.0  # manual recommends 20 Hz for translation/rotation
    cmd_timeout: float = 0.4  # deadman: zero the command if no fresh cmd_vel
    autostand: bool = True  # bring the robot up to a drivable state on start()
    # Low-latency video: skip buffered frames to the live edge + FFmpeg no-buffer
    # flags. The RTSP stack will otherwise hold ~0.5 s of frames after any stall.
    low_latency: bool = True


class M20Connection(Module, VideoSource):
    """Video + teleop for the DeepRobotics Lynx M20."""

    dedicated_worker = True

    config: M20Config
    cmd_vel: In[Twist]
    color_image: Out[Image]  # front wide-angle
    color_image_rear: Out[Image]  # rear wide-angle
    status: Out[M20Status]  # state-machine status (BasicStatus)
    battery_left: Out[BatteryState]
    battery_right: Out[BatteryState]

    _link: PatrolLink | None = None
    _latest_video_frame: Image | None = None
    _video_threads: list[threading.Thread] = []
    _control_thread: threading.Thread | None = None
    _running: bool = False
    _cmd: Twist | None = None
    _cmd_ts: float = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self._running = True
        self._cmd = Twist.zero()
        self._cmd_ts = 0.0

        self._link = PatrolLink(self.config.ip)
        self._link.on_report = self._on_report  # set before start() so we catch early reports
        self._link.start()

        if self.config.autostand:
            self._bringup()

        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))

        ip = self.config.ip
        cameras = (
            (self.config.rtsp_url or f"rtsp://{ip}:8554/video1", self.color_image, "m20_front"),
            (
                self.config.rtsp_url_rear or f"rtsp://{ip}:8554/video2",
                self.color_image_rear,
                "m20_rear",
            ),
        )
        self._video_threads = [
            threading.Thread(
                target=self._stream_camera, args=cam, daemon=True, name=f"m20-{cam[2]}"
            )
            for cam in cameras
        ]

        for t in self._video_threads:
            t.start()

        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

    @rpc
    def stop(self) -> None:
        self._running = False
        for t in (*self._video_threads, self._control_thread):
            if t and t.is_alive():
                t.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        if self._link:
            self._link.send(*_MOTION_STATE, {"MotionParam": _STATE_STAND})  # settle
            self._link.stop()
            self._link = None
        super().stop()

    def _bringup(self) -> None:
        """Regular mode -> stand -> standard pose -> basic gait (drivable)."""
        link = self._link
        assert link is not None
        link.send(*_USAGE_MODE, {"Mode": _MODE_REGULAR})
        time.sleep(0.5)
        link.send(*_MOTION_STATE, {"MotionParam": _STATE_STAND})
        time.sleep(3.0)
        link.send(*_MOTION_STATE, {"MotionParam": _STATE_STANDARD})
        time.sleep(1.0)
        link.send(*_GAIT, {"GaitParam": _GAIT_BASIC})

    def _on_report(self, type_: int, command: int, items: Mapping[str, object]) -> None:
        """Map M20 telemetry reports to typed Out streams (runs on the recv thread)."""
        if (type_, command) == _REPORT_BASIC_STATUS:
            bs = items.get("BasicStatus")
            if isinstance(bs, dict):
                self.status.publish(_status_from(bs))
        elif (type_, command) == _REPORT_DEVICE_STATE:
            bat = items.get("BatteryStatus")
            if isinstance(bat, dict):
                self.battery_left.publish(_battery_from(bat, "Left", "left"))
                self.battery_right.publish(_battery_from(bat, "Right", "right"))

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Set teleop velocity. linear.x = forward (m/s), linear.y = left (m/s),
        angular.z = yaw (rad/s). Held until the next command; the control loop
        re-sends at ``cmd_rate_hz`` and zeroes it after ``cmd_timeout``.

        ``duration`` is accepted for parity with other dimos connections but is
        unused — hold behaviour is handled by the deadman in the control loop.
        """
        self._cmd = twist
        self._cmd_ts = time.time()
        return True

    def _control_loop(self) -> None:
        # Re-send the latest command at cmd_rate_hz (manual recommends 20 Hz);
        # the deadman zeroes it if cmd_vel goes stale, so the robot stops if we do.
        period = 1.0 / self.config.cmd_rate_hz
        cfg = self.config
        while self._running:
            twist = _apply_deadman(self._cmd, time.time() - self._cmd_ts, cfg.cmd_timeout)
            axes = _axes_from_twist(twist, cfg.max_linear, cfg.max_angular)
            self._link.send(*_MOVE, axes)  # type: ignore[union-attr]
            time.sleep(period)

    def _open_capture(self, url: str) -> cv2.VideoCapture:
        if self.config.low_latency:
            # Process-global, but this is a dedicated_worker so it's just us. Don't
            # force a transport (UDP loses packets -> HEVC artifacts on wifi).
            os.environ.setdefault(
                "OPENCV_FFMPEG_CAPTURE_OPTIONS",
                "fflags;nobuffer|flags;low_delay|reorder_queue_size;0|max_delay;0",
            )
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _stream_camera(self, url: str, port: Out[Image], frame_id: str) -> None:
        cap = self._open_capture(url)
        if not cap.isOpened():
            logger.error("M20: cannot open RTSP stream %s", url)
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        live_edge = 0.5 / fps  # a grab slower than this means we've reached live video

        while self._running:
            if not cap.isOpened():
                time.sleep(1.0)
                cap = self._open_capture(url)
                continue

            # Skip any buffered backlog without decoding it: grab() returns instantly
            # for already-buffered frames and blocks (~frame period) at the live edge.
            # Decode only the newest frame -> minimal latency, auto-recovers from stalls.
            grabbed = False
            while self._running:
                t = time.time()
                if not cap.grab():
                    break
                grabbed = True
                if not self.config.low_latency or time.time() - t >= live_edge:
                    break
            if not grabbed:
                logger.warning("M20: RTSP read failed (%s); reconnecting", frame_id)
                cap.release()
                time.sleep(0.5)
                cap = self._open_capture(url)
                continue

            ok, frame = cap.retrieve()
            if not ok:
                continue
            img = Image.from_numpy(frame, format=ImageFormat.BGR, frame_id=frame_id)
            if port is self.color_image:  # front feeds observe()
                self._latest_video_frame = img
            port.publish(img)
        cap.release()

    @skill
    def observe(self) -> Image | None:
        """Return the latest front-camera frame from the M20, or None if none yet."""
        return self._latest_video_frame


def deploy(dimos: ModuleCoordinator, ip: str = "10.21.31.103", prefix: str = "") -> ModuleProxy:
    from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE

    conn = dimos.deploy(M20Connection, ip=ip)
    conn.color_image.transport = pSHMTransport(
        f"{prefix}/image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    )
    conn.color_image_rear.transport = pSHMTransport(
        f"{prefix}/image_rear", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    )
    conn.cmd_vel.transport = LCMTransport(f"{prefix}/cmd_vel", Twist)
    conn.status.transport = LCMTransport(f"{prefix}/status", M20Status)
    conn.battery_left.transport = LCMTransport(f"{prefix}/battery_left", BatteryState)
    conn.battery_right.transport = LCMTransport(f"{prefix}/battery_right", BatteryState)
    conn.start()
    return conn
