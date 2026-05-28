from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Callable, Protocol

import cv2
import numpy as np
from reactivex.subject import Subject

from demo_app.types import Pose

logger = logging.getLogger(__name__)


class _ConnectionProtocol(Protocol):
    def lidar_stream(self): ...
    def video_stream(self): ...
    def odom_stream(self): ...
    def move(self, twist, duration: float = 0.0) -> bool: ...
    def standup(self) -> bool: ...
    def liedown(self) -> bool: ...
    def balance_stand(self) -> bool: ...
    def set_obstacle_avoidance(self, enabled: bool = True) -> None: ...
    def stop(self) -> None: ...


def _default_connection_factory(ip: str, mode: str = "ai"):
    from dimos.robot.unitree.connection import UnitreeWebRTCConnection
    return UnitreeWebRTCConnection(ip=ip, mode=mode)


class _Twist:
    class _Vec:
        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
            self.x, self.y, self.z = x, y, z

    def __init__(self, vx: float, vy: float, vyaw: float) -> None:
        self.linear = self._Vec(vx, vy, 0.0)
        self.angular = self._Vec(0.0, 0.0, vyaw)


class Go2Runner:
    MAX_VX = 0.8
    MAX_VY = 0.3
    MAX_VYAW = 0.8
    ARRIVE_DIST = 0.25
    ARRIVE_YAW = 5.0

    def __init__(
        self,
        robot_ip: str,
        obstacle_avoidance: bool = True,
        camera_resize: tuple[int, int] = (640, 480),
        connection_factory: Callable[..., _ConnectionProtocol] = _default_connection_factory,
        standup_delay_sec: float = 2.0,
        connect_timeout_sec: float = 15.0,
    ) -> None:
        self._ip = robot_ip
        self._obstacle_avoidance = obstacle_avoidance
        self._resize_w, self._resize_h = camera_resize
        self._connection_factory = connection_factory
        self._standup_delay = standup_delay_sec
        self._connect_timeout = connect_timeout_sec

        self._conn: _ConnectionProtocol | None = None
        self._video_subject: Subject = Subject()
        self._lidar_subject: Subject = Subject()
        self._pose = Pose(0.0, 0.0, 0.0)
        self._pose_lock = threading.Lock()
        self._video_sub = None
        self._lidar_sub = None
        self._odom_sub = None
        self._audio_hub: Any = None
        self._alert_uuid: str | None = None

    def start(self) -> None:
        if self._conn is not None:
            return
        logger.info("Connecting to Go2 at %s", self._ip)
        result: dict[str, Any] = {}
        error: dict[str, BaseException] = {}

        def _build_connection() -> None:
            try:
                result["conn"] = self._connection_factory(self._ip, mode="ai")
            except BaseException as exc:  # noqa: BLE001 - propagate exact startup failure
                error["exc"] = exc

        connect_thread = threading.Thread(target=_build_connection, daemon=True)
        connect_thread.start()
        connect_thread.join(self._connect_timeout)
        if connect_thread.is_alive():
            raise RuntimeError(
                f"Timed out after {self._connect_timeout:.0f}s connecting to Go2 at "
                f"{self._ip!r}. Check Wi-Fi, IP, and whether the robot is powered on."
            )
        if "exc" in error:
            exc = error["exc"]
            raise RuntimeError(
                f"Cannot reach Go2 at {self._ip!r}. Check Wi-Fi and IP. "
                f"({type(exc).__name__}: {exc})"
            ) from exc
        self._conn = result["conn"]

        logger.info("Subscribing to video and odometry streams")
        self._video_sub = self._conn.video_stream().subscribe(self._on_video)
        self._lidar_sub = self._conn.lidar_stream().subscribe(self._on_lidar)
        self._odom_sub = self._conn.odom_stream().subscribe(self._on_odom)
        logger.info("Sending standup and balance commands")
        self._conn.standup()
        time.sleep(self._standup_delay)
        self._conn.balance_stand()
        if hasattr(self._conn, "free_walk"):
            try:
                logger.info("Attempting to activate FreeWalk mode")
                self._conn.free_walk()
                logger.info("FreeWalk mode activated")
            except Exception as e:
                logger.warning("FreeWalk activation failed: %s", e)
        self._conn.set_obstacle_avoidance(self._obstacle_avoidance)
        logger.info("Connected to Go2 at %s", self._ip)

        try:
            from unitree_webrtc_connect.webrtc_audiohub import WebRTCAudioHub

            legion_conn = getattr(self._conn, "conn", None)
            if legion_conn is not None:
                self._audio_hub = WebRTCAudioHub(connection=legion_conn, logger=logger)
                logger.info("WebRTCAudioHub initialized")
        except Exception as e:
            logger.warning("Audio hub init failed: %s", e)
            self._audio_hub = None

    def stop(self) -> None:
        if self._conn is None:
            return
        try:
            if self._video_sub is not None:
                self._video_sub.dispose()
        except Exception:
            pass
        try:
            if self._lidar_sub is not None:
                self._lidar_sub.dispose()
        except Exception:
            pass
        try:
            if self._odom_sub is not None:
                self._odom_sub.dispose()
        except Exception:
            pass
        try:
            self._conn.liedown()
        except Exception as e:
            logger.warning("Go2 lie down failed: %s", e)
        try:
            self._conn.stop()
        except Exception:
            pass
        self._conn = None

    def video_stream(self) -> Subject:
        return self._video_subject

    def lidar_stream(self) -> Subject:
        return self._lidar_subject

    def _on_video(self, frame) -> None:
        try:
            if hasattr(frame, "data") and isinstance(getattr(frame, "data", None), np.ndarray):
                arr = frame.data
            elif hasattr(frame, "to_ndarray"):
                arr = frame.to_ndarray(format="rgb24")
            elif isinstance(frame, np.ndarray):
                arr = frame
            else:
                arr = np.asarray(frame)

            if arr.ndim != 3:
                return

            if arr.shape[:2] != (self._resize_h, self._resize_w):
                arr = cv2.resize(arr, (self._resize_w, self._resize_h), interpolation=cv2.INTER_AREA)
            self._video_subject.on_next(arr)
        except Exception as e:
            logger.warning("Video frame handler error: %s", e)

    def _on_lidar(self, msg) -> None:
        try:
            self._lidar_subject.on_next(msg)
        except Exception as e:
            logger.warning("Lidar handler error: %s", e)

    def _on_odom(self, msg) -> None:
        try:
            x = getattr(msg, "x", None)
            y = getattr(msg, "y", None)
            if x is None and hasattr(msg, "position"):
                x = msg.position.x
                y = msg.position.y
            qw = getattr(msg, "qw", None)
            qx = getattr(msg, "qx", 0.0)
            qy = getattr(msg, "qy", 0.0)
            qz = getattr(msg, "qz", None)
            if qw is None and hasattr(msg, "orientation"):
                qw, qx, qy, qz = (
                    msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
                )
            yaw_rad = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            with self._pose_lock:
                self._pose = Pose(float(x), float(y), math.degrees(yaw_rad))
        except Exception as e:
            logger.warning("Odom handler error: %s", e)

    def get_pose(self) -> Pose:
        with self._pose_lock:
            return self._pose

    def move_to(
        self,
        x: float,
        y: float,
        yaw_deg: float,
        duration_sec: float = 5.0,
        should_stop: Callable[[], bool] | None = None,
    ) -> bool:
        if self._conn is None:
            return False
        deadline = time.monotonic() + duration_sec
        tick = 0.05
        kp_lin = 0.8
        kp_yaw = 1.2

        try:
            interrupted = False
            while time.monotonic() < deadline:
                conn = self._conn
                if conn is None:
                    return False
                if should_stop is not None and should_stop():
                    logger.info("move_to interrupted before reaching target")
                    interrupted = True
                    break

                pose = self.get_pose()
                dx, dy = x - pose.x, y - pose.y
                dist = math.hypot(dx, dy)
                yaw_err = ((yaw_deg - pose.yaw_deg + 180.0) % 360.0) - 180.0
                if dist < self.ARRIVE_DIST and abs(yaw_err) < self.ARRIVE_YAW:
                    break

                if dist > self.ARRIVE_DIST:
                    yaw_rad = math.radians(pose.yaw_deg)
                    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
                    fwd = cy * dx + sy * dy
                    lat = -sy * dx + cy * dy
                    vx = max(-self.MAX_VX, min(self.MAX_VX, kp_lin * fwd))
                    vy = max(-self.MAX_VY, min(self.MAX_VY, kp_lin * lat))
                else:
                    vx, vy = 0.0, 0.0

                vyaw = max(-self.MAX_VYAW, min(self.MAX_VYAW, kp_yaw * math.radians(yaw_err)))
                conn.move(_Twist(vx, vy, vyaw))
                time.sleep(tick)
        finally:
            try:
                conn = self._conn
                if conn is not None:
                    conn.move(_Twist(0.0, 0.0, 0.0))
            except Exception:
                pass
        return not interrupted

    def drive_for_duration(
        self,
        forward_mps: float = 0.0,
        left_mps: float = 0.0,
        yaw_radps: float = 0.0,
        duration_sec: float = 0.75,
    ) -> bool:
        if self._conn is None:
            return False
        logger.info(
            "Manual drive: forward=%.2f left=%.2f yaw=%.2f duration=%.2f",
            forward_mps,
            left_mps,
            yaw_radps,
            duration_sec,
        )
        try:
            return bool(self._conn.move(_Twist(forward_mps, left_mps, yaw_radps), duration_sec))
        except Exception as e:
            logger.warning("Manual drive failed: %s", e)
            return False

    async def play_alert_on_robot(self, audio_file: str) -> bool:
        if self._audio_hub is None:
            return False
        path = str(audio_file)
        try:
            if self._alert_uuid is None:
                await self._audio_hub.upload_audio_file(path)
                listing = await self._audio_hub.get_audio_list()
                records = self._extract_audio_records(listing)
                target_name = path.rsplit("/", 1)[-1].rsplit(".", 1)[0]
                for rec in records:
                    if rec.get("file_name") == target_name or rec.get("name") == target_name:
                        self._alert_uuid = rec.get("unique_id") or rec.get("uuid")
                        break
                if self._alert_uuid is None and records:
                    self._alert_uuid = records[-1].get("unique_id") or records[-1].get("uuid")

            if self._alert_uuid:
                await self._audio_hub.play_by_uuid(self._alert_uuid)
                return True
            return await self._megaphone_play(path)
        except Exception as e:
            logger.warning("Robot alert playback failed: %s", e)
            return False

    async def _megaphone_play(self, path: str) -> bool:
        try:
            await self._audio_hub.enter_megaphone()
            await self._audio_hub.upload_megaphone(path)
            await self._audio_hub.exit_megaphone()
            return True
        except Exception as e:
            logger.warning("Megaphone playback failed: %s", e)
            return False

    @staticmethod
    def _extract_audio_records(listing) -> list[dict]:
        if listing is None:
            return []
        if isinstance(listing, list):
            return listing
        if isinstance(listing, dict):
            for key in ("data", "audio_list", "list", "records", "items"):
                value = listing.get(key)
                if isinstance(value, list):
                    return value
                if isinstance(value, dict):
                    for nested in ("audio_list", "list", "records", "items"):
                        nested_value = value.get(nested)
                        if isinstance(nested_value, list):
                            return nested_value
        return []
