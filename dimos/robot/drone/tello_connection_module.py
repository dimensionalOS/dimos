#!/usr/bin/env python3
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

"""DimOS module wrapper for RoboMaster TT / Tello SDK control."""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import String
from reactivex.disposable import CompositeDisposable, Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Twist, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.robot.drone.tello_sdk import TelloSdkClient
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _add_disposable(composite: CompositeDisposable, item: Disposable | Any) -> None:
    if isinstance(item, Disposable):
        composite.add(item)
    elif callable(item):
        composite.add(Disposable(item))


def _to_float(value: Any, default: float = 0.0) -> float:
    if value is None:
        return default
    if isinstance(value, (float, int)):
        return float(value)
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


class TelloConnectionModule(Module):
    """Module that bridges Tello SDK to DimOS streams and skills."""

    # Inputs
    movecmd: In[Vector3]
    movecmd_twist: In[Twist]
    extcmd: In[String]
    tracking_status: In[Any]

    # Outputs
    odom: Out[PoseStamped]
    status: Out[Any]
    telemetry: Out[Any]
    video: Out[Image]
    follow_object_cmd: Out[Any]

    # Parameters
    tello_ip: str
    local_ip: str
    local_command_port: int
    state_port: int
    video_port: int
    command_timeout: float
    video_uri: str | None

    # Internal state
    _latest_odom: PoseStamped | None = None
    _latest_video_frame: Image | None = None
    _latest_status: dict[str, Any] | None = None
    _latest_telemetry: dict[str, Any] | None = None
    _latest_tracking_status: dict[str, Any] | None = None
    _state_lock: threading.RLock

    def __init__(
        self,
        tello_ip: str = "192.168.10.1",
        local_ip: str = "",
        local_command_port: int = 9000,
        state_port: int = 8890,
        video_port: int = 11111,
        command_timeout: float = 7.0,
        video_uri: str | None = None,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        """Initialize Tello module.

        Args:
            tello_ip: Drone IP address.
            local_ip: Local bind host. Empty binds all interfaces.
            local_command_port: Local UDP port for command responses.
            state_port: Local UDP port receiving state packets.
            video_port: Local UDP port receiving H264 video.
            command_timeout: Timeout for SDK commands.
            video_uri: Optional OpenCV URI override for the video stream.
        """
        self.tello_ip = tello_ip
        self.local_ip = local_ip
        self.local_command_port = local_command_port
        self.state_port = state_port
        self.video_port = video_port
        self.command_timeout = command_timeout
        self.video_uri = video_uri

        self.connection: TelloSdkClient | None = None
        self._latest_odom = None
        self._latest_video_frame = None
        self._latest_status = None
        self._latest_telemetry = None
        self._latest_tracking_status = None
        self._state_lock = threading.RLock()
        self._manual_override_until = 0.0
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self) -> None:
        """Start the module and subscribe to Tello streams."""
        self.connection = TelloSdkClient(
            tello_ip=self.tello_ip,
            local_ip=self.local_ip,
            local_command_port=self.local_command_port,
            state_port=self.state_port,
            video_port=self.video_port,
            command_timeout=self.command_timeout,
            video_uri=self.video_uri,
        )

        if not self.connection.connect():
            logger.error("Failed to connect to Tello TT")
            return

        _add_disposable(self._disposables, self.connection.odom_stream().subscribe(self._publish_tf))
        _add_disposable(
            self._disposables, self.connection.status_stream().subscribe(self._publish_status)
        )
        _add_disposable(
            self._disposables, self.connection.telemetry_stream().subscribe(self._publish_telemetry)
        )
        _add_disposable(
            self._disposables, self.connection.video_stream().subscribe(self._store_and_publish_frame)
        )

        _add_disposable(self._disposables, self.movecmd.subscribe(self._on_move))

        if self.movecmd_twist.transport:
            _add_disposable(self._disposables, self.movecmd_twist.subscribe(self._on_move_twist))
        if self.extcmd.transport:
            _add_disposable(self._disposables, self.extcmd.subscribe(self._on_ext_cmd))
        if self.tracking_status.transport:
            _add_disposable(self._disposables, self.tracking_status.subscribe(self._on_tracking_status))

        logger.info("TelloConnectionModule started")
        return

    def _store_and_publish_frame(self, frame: Image) -> None:
        self._latest_video_frame = frame
        self.video.publish(frame)

    def _publish_tf(self, msg: PoseStamped) -> None:
        self._latest_odom = msg
        self.odom.publish(msg)

        base_link = Transform(
            translation=msg.position,
            rotation=msg.orientation,
            frame_id="world",
            child_frame_id="base_link",
            ts=msg.ts if hasattr(msg, "ts") else time.time(),
        )
        self.tf.publish(base_link)

        camera_link = Transform(
            translation=Vector3(0.05, 0.0, -0.02),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=time.time(),
        )
        self.tf.publish(camera_link)

    def _publish_status(self, status: dict[str, Any]) -> None:
        with self._state_lock:
            self._latest_status = dict(status)
        self.status.publish(String(json.dumps(status)))

    def _publish_telemetry(self, telemetry: dict[str, Any]) -> None:
        with self._state_lock:
            self._latest_telemetry = dict(telemetry)
        self.telemetry.publish(String(json.dumps(telemetry)))

    def _on_move(self, cmd: Vector3) -> None:
        if self.connection:
            self.connection.move(cmd, duration=0.0)

    def _on_move_twist(self, msg: Twist) -> None:
        if time.time() < self._manual_override_until:
            return
        if self.connection:
            self.connection.move_twist(msg, duration=0.0, lock_altitude=True)

    def _on_ext_cmd(self, msg: String) -> None:
        if self.connection:
            self.connection.send_ext(msg.data)

    def _on_tracking_status(self, status: Any) -> None:
        data: dict[str, Any] | None = None
        if isinstance(status, String):
            try:
                data = json.loads(status.data)
            except json.JSONDecodeError:
                data = None
        elif isinstance(status, dict):
            data = status

        if data is not None:
            with self._state_lock:
                self._latest_tracking_status = data

    def _wait_for_tracking_status(
        self, timeout: float
    ) -> tuple[str | None, dict[str, Any] | None]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._state_lock:
                status = dict(self._latest_tracking_status or {}) or None
            if status is not None:
                value = status.get("status")
                if isinstance(value, str) and value in {"tracking", "not_found", "failed", "lost"}:
                    return value, status
            time.sleep(0.05)
        with self._state_lock:
            status = dict(self._latest_tracking_status or {}) or None
        return None, status

    def _wait_until_airborne(self, min_height_cm: float = 20.0, timeout: float = 12.0) -> bool:
        if not self.connection:
            return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            state = self.connection.get_state()
            if _to_float(state.get("h")) >= min_height_cm:
                return True
            time.sleep(0.1)
        state = self.connection.get_state()
        return _to_float(state.get("h")) >= min_height_cm

    @rpc
    def get_odom(self) -> PoseStamped | None:
        """Get latest odometry estimate."""
        return self._latest_odom

    @rpc
    def get_status(self) -> dict[str, Any]:
        """Get latest compact status dict."""
        with self._state_lock:
            return dict(self._latest_status or {})

    @rpc
    def get_state(self) -> dict[str, Any]:
        """Get latest raw Tello state packet as dict."""
        with self._state_lock:
            return dict(self._latest_telemetry or {})

    @skill
    def move(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, duration: float = 0.0) -> str:
        """Move with body-frame velocity.

        Args:
            x: Forward velocity in m/s.
            y: Left velocity in m/s.
            z: Up velocity in m/s.
            duration: Optional duration in seconds. 0 means single update.
        """
        if not self.connection:
            return "Failed: no Tello connection"
        ok = self.connection.move(Vector3(x, y, z), duration=duration)
        return "Move command sent" if ok else "Failed: move command rejected"

    @skill
    def rc(
        self,
        left_right: int = 0,
        forward_back: int = 0,
        up_down: int = 0,
        yaw: int = 0,
        duration: float = 0.0,
    ) -> str:
        """Send low-level rc command in SDK units.

        Args:
            left_right: Left/right channel in [-100, 100]. Positive is right.
            forward_back: Forward/back channel in [-100, 100]. Positive is forward.
            up_down: Vertical channel in [-100, 100]. Positive is up.
            yaw: Yaw channel in [-100, 100]. Positive is clockwise.
            duration: Optional duration before auto-stop.
        """
        if not self.connection:
            return "Failed: no Tello connection"
        ok = self.connection.rc(left_right, forward_back, up_down, yaw)
        if duration > 0:
            time.sleep(duration)
            self.connection.rc(0, 0, 0, 0)
        return "RC command sent" if ok else "Failed: rc command rejected"

    @skill
    def move_relative(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, speed: float = 0.4) -> str:
        """Move by a relative offset using the SDK go command.

        Args:
            x: Forward distance in meters.
            y: Left distance in meters.
            z: Up distance in meters.
            speed: Translation speed in m/s.
        """
        if not self.connection:
            return "Failed: no Tello connection"
        ok = self.connection.move_relative(x=x, y=y, z=z, speed=speed)
        return "Relative move command sent" if ok else "Failed: move_relative rejected"

    @skill
    def yaw(self, degrees: float) -> str:
        """Rotate in place.

        Args:
            degrees: Positive rotates counter-clockwise, negative clockwise.
        """
        if not self.connection:
            return "Failed: no Tello connection"
        ok = self.connection.yaw(degrees)
        return "Yaw command sent" if ok else "Failed: yaw command rejected"

    @skill
    def takeoff(self, altitude: float = 1.0) -> str:
        """Take off and hover.

        Args:
            altitude: Desired altitude hint in meters.
        """
        if not self.connection:
            return "Failed: no Tello connection"
        ok = self.connection.takeoff()
        if not ok:
            return "Failed: takeoff rejected"
        if altitude > 1.0:
            return "Takeoff command sent (TT hovers at fixed initial altitude; use move_relative z for more)"
        return "Takeoff command sent"

    @skill
    def land(self) -> str:
        """Land immediately."""
        if not self.connection:
            return "Failed: no Tello connection"
        ok = self.connection.land()
        return "Land command sent" if ok else "Failed: land rejected"

    @skill
    def emergency_stop(self) -> str:
        """Emergency stop motors immediately."""
        if not self.connection:
            return "Failed: no Tello connection"
        ok = self.connection.emergency()
        return "Emergency command sent" if ok else "Failed: emergency rejected"

    @skill
    def send_ext(self, ext_command: str) -> str:
        """Send a TT expansion-board EXT command.

        Args:
            ext_command: Raw EXT payload, with or without the EXT prefix.
        """
        if not self.connection:
            return "Failed: no Tello connection"
        ok = self.connection.send_ext(ext_command)
        return "EXT command sent" if ok else "Failed: EXT command rejected"

    @skill
    def follow_object(
        self,
        object_description: str = "person",
        distance_m: float = 1.0,
        duration: float = 120.0,
        scan_step_deg: float = 30.0,
        max_scan_steps: int = 12,
        control_mode: str = "full",
    ) -> str:
        """Scan for and follow an object using visual tracking.

        Args:
            object_description: Object to search for (for example "person").
            distance_m: Desired following distance in meters (currently a hint only).
            duration: Maximum follow duration in seconds once acquired. Also used as search budget.
            scan_step_deg: Yaw step used between search attempts.
            max_scan_steps: Number of scan attempts before giving up.
            control_mode: "full" for translation+tracking, or "yaw_only" to rotate in place only.
                Use "yaw_only" when the user asks to hover and only rotate to center a person.
        """
        if not self.connection:
            return "Failed: no Tello connection"
        if not self.follow_object_cmd.transport:
            return "Failed: follow_object stream is not connected"
        if not self._wait_until_airborne(min_height_cm=20.0, timeout=12.0):
            logger.warning("follow_object: drone not confirmed airborne; refusing to track")
            return "Failed: drone is not airborne yet"

        steps = max(1, max_scan_steps)
        scan_budget_s = max(2.0, min(duration, 30.0))
        per_step_budget_s = max(0.8, scan_budget_s / steps)
        follow_duration_s = max(duration, 60.0)

        for step in range(steps):
            step_deadline = time.time() + per_step_budget_s
            while time.time() < step_deadline:
                with self._state_lock:
                    self._latest_tracking_status = None

                msg = {
                    "object_description": object_description,
                    "duration": follow_duration_s,
                    "distance_m": distance_m,
                    "control_mode": control_mode,
                }
                self.follow_object_cmd.publish(String(json.dumps(msg)))

                time_left = max(0.2, step_deadline - time.time())
                status, detail = self._wait_for_tracking_status(timeout=min(1.5, time_left))
                if status == "tracking":
                    return (
                        f"Following {object_description} for up to {follow_duration_s:.0f}s. "
                        "Using fixed forward approach speed with yaw centering. "
                        f"Mode={control_mode}."
                    )
                if status == "lost":
                    return f"Started tracking {object_description} but target was lost"
                if status == "failed":
                    reason = detail.get("error", "tracker failed") if detail else "tracker failed"
                    # Hard backend/config errors should return immediately.
                    if "no supported" in str(reason).lower() or "stream is not connected" in str(
                        reason
                    ).lower():
                        return f"Failed to start tracking {object_description}: {reason}"

                time.sleep(0.15)

            if step < steps - 1 and abs(scan_step_deg) > 0.0:
                self.connection.yaw(scan_step_deg)
                time.sleep(0.5)

        return (
            f"Could not find {object_description} after scanning for about {scan_budget_s:.0f}s"
        )

    @skill
    def center_person_by_yaw(
        self,
        duration: float = 120.0,
        scan_step_deg: float = 0.0,
        max_scan_steps: int = 1,
    ) -> str:
        """Hover and rotate to keep a detected person centered in view (no translation).

        Args:
            duration: Maximum tracking duration in seconds once acquired.
            scan_step_deg: Optional yaw scan step between search attempts.
            max_scan_steps: Number of search orientations before giving up.
        """
        return self.follow_object(
            object_description="person",
            distance_m=0.0,
            duration=duration,
            scan_step_deg=scan_step_deg,
            max_scan_steps=max_scan_steps,
            control_mode="yaw_only",
        )

    @skill
    def orbit_object(
        self,
        radius_m: float = 1.0,
        speed_mps: float = 0.35,
        revolutions: float = 1.0,
        clockwise: bool = False,
    ) -> str:
        """Circle around the currently followed target using velocity+yaw control.

        Args:
            radius_m: Desired orbit radius in meters.
            speed_mps: Tangential orbit speed in m/s.
            revolutions: Number of revolutions to complete.
            clockwise: If True orbit clockwise, otherwise counter-clockwise.
        """
        if not self.connection:
            return "Failed: no Tello connection"
        if radius_m <= 0.1:
            return "Failed: radius_m must be greater than 0.1"
        if speed_mps <= 0.05:
            return "Failed: speed_mps must be greater than 0.05"
        if revolutions <= 0.0:
            return "Failed: revolutions must be positive"

        duration = (2.0 * math.pi * radius_m * revolutions) / speed_mps
        direction = -1.0 if clockwise else 1.0

        self._manual_override_until = time.time() + duration + 0.5
        try:
            yaw_rate = direction * (speed_mps / radius_m)
            ok = self.connection.move(
                Vector3(0.0, direction * speed_mps, 0.0),
                yaw_rate=yaw_rate,
                duration=duration,
            )
        finally:
            self._manual_override_until = 0.0

        if ok:
            orbit_dir = "clockwise" if clockwise else "counter-clockwise"
            return (
                f"Completed {revolutions:.2f} {orbit_dir} orbit(s) at radius {radius_m:.2f}m "
                f"and speed {speed_mps:.2f}m/s"
            )
        return "Failed: orbit command rejected"

    @skill
    def observe(self) -> Image | None:
        """Return latest camera frame for perception tasks."""
        return self._latest_video_frame

    @rpc
    def stop(self) -> None:
        """Stop module and close the Tello connection."""
        if self.connection:
            self.connection.disconnect()
            self.connection = None
        logger.info("TelloConnectionModule stopped")
        super().stop()


tello_connection_module = TelloConnectionModule.blueprint

__all__ = ["TelloConnectionModule", "tello_connection_module"]
