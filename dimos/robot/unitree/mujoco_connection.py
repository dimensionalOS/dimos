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

"""In-process MuJoCo backend for Unitree connection modules.

The owning ``GO2Connection`` or ``G1SimConnection`` module exposes all process
boundaries as typed streams. This backend only owns physics state and a small,
thread-safe latest-value handoff to that module; it creates no transport,
subprocess, or shared-memory channel.
"""

from collections.abc import Callable
import functools
import threading
from typing import Any, TypeVar

import numpy as np
from numpy.typing import NDArray
from reactivex import Observable, empty
from reactivex.abc import ObserverBase, SchedulerBase
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.type.odometry import Odometry
from dimos.simulation.mujoco.constants import (
    LIDAR_FPS,
    VIDEO_CAMERA_FOV,
    VIDEO_FPS,
    VIDEO_HEIGHT,
    VIDEO_WIDTH,
)
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

ODOM_FREQUENCY = 50
_START_TIMEOUT = 300.0

logger = setup_logger()

T = TypeVar("T")


class MujocoConnection:
    """Policy-controlled MuJoCo backend owned by a normal connection module."""

    camera_info_static: CameraInfo = CameraInfo.from_fov(
        fov_deg=VIDEO_CAMERA_FOV,
        width=VIDEO_WIDTH,
        height=VIDEO_HEIGHT,
        axis="vertical",
        frame_id="camera_optical",
    )

    def __init__(self, global_config: GlobalConfig) -> None:
        try:
            import mujoco  # noqa: F401
            from mujoco_playground._src import mjx_env
        except ImportError as exc:
            raise ImportError(
                "Simulation dependencies are not installed. "
                "Run `uv sync --extra sim --inexact` to install them."
            ) from exc

        get_data("mujoco_sim")
        mjx_env.ensure_menagerie_exists()

        self.global_config = global_config
        self._lock = threading.Lock()
        self._command = np.zeros(3, dtype=np.float32)
        self._latest_video: tuple[NDArray[np.uint8], int] | None = None
        self._latest_odom: tuple[NDArray[Any], NDArray[Any], float, int] | None = None
        self._latest_lidar: tuple[PointCloud2, int] | None = None
        self._video_seq = 0
        self._odom_seq = 0
        self._lidar_seq = 0
        self._last_video_seq = 0
        self._last_odom_seq = 0
        self._last_lidar_seq = 0

        self._ready_event = threading.Event()
        self._simulation_stop_event = threading.Event()
        self._simulation_thread: threading.Thread | None = None
        self._simulation_error: BaseException | None = None
        self._stop_timer: threading.Timer | None = None
        self._stream_threads: list[threading.Thread] = []
        self._stream_stop_events: list[threading.Event] = []
        self._is_cleaned_up = False

    def start(self) -> None:
        if self._simulation_thread is not None and self._simulation_thread.is_alive():
            return

        self._is_cleaned_up = False
        self._simulation_error = None
        self._ready_event.clear()
        self._simulation_stop_event.clear()

        def run() -> None:
            try:
                from dimos.simulation.mujoco.locomotion_sim import run_locomotion_sim

                run_locomotion_sim(self.global_config, self)
            except BaseException as exc:
                self._simulation_error = exc
                logger.exception("MuJoCo simulation failed")
            finally:
                self._ready_event.set()

        self._simulation_thread = threading.Thread(
            target=run,
            name="UnitreeMujocoSim",
            daemon=True,
        )
        self._simulation_thread.start()

        if not self._ready_event.wait(timeout=_START_TIMEOUT):
            self.stop()
            raise RuntimeError("MuJoCo simulation failed to start (timeout)")
        if self._simulation_error is not None:
            error = self._simulation_error
            self.stop()
            raise RuntimeError("MuJoCo simulation failed to start") from error
        logger.info("MuJoCo simulation started")

    def stop(self) -> None:
        if self._is_cleaned_up:
            return
        self._is_cleaned_up = True

        if self._stop_timer is not None:
            self._stop_timer.cancel()
            self._stop_timer = None

        for stop_event in self._stream_stop_events:
            stop_event.set()
        self._simulation_stop_event.set()

        for thread in self._stream_threads:
            if thread.is_alive():
                thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
                if thread.is_alive():
                    logger.warning(f"Stream thread {thread.name} did not stop gracefully")

        simulation_thread = self._simulation_thread
        if simulation_thread is not None and simulation_thread.is_alive():
            simulation_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            if simulation_thread.is_alive():
                logger.warning("MuJoCo simulation thread did not stop gracefully")
        self._simulation_thread = None

        self._stream_threads.clear()
        self._stream_stop_events.clear()
        self.lidar_stream.cache_clear()
        self.odom_stream.cache_clear()
        self.video_stream.cache_clear()

    def signal_ready(self) -> None:
        self._ready_event.set()

    def should_stop(self) -> bool:
        return self._simulation_stop_event.is_set()

    def get_command(self) -> NDArray[Any]:
        """Return the latest held ``[forward, lateral, yaw]`` command."""
        with self._lock:
            return self._command.copy()

    def publish_video(self, frame: NDArray[np.uint8]) -> None:
        with self._lock:
            self._video_seq += 1
            self._latest_video = (frame.copy(), self._video_seq)

    def publish_odom(
        self,
        position: NDArray[np.float64],
        quaternion_wxyz: NDArray[np.float64],
        timestamp: float,
    ) -> None:
        with self._lock:
            self._odom_seq += 1
            self._latest_odom = (
                position.copy(),
                quaternion_wxyz.copy(),
                timestamp,
                self._odom_seq,
            )

    def publish_lidar(self, message: PointCloud2) -> None:
        with self._lock:
            self._lidar_seq += 1
            self._latest_lidar = (message, self._lidar_seq)

    def get_video_frame(self) -> NDArray[np.uint8] | None:
        with self._lock:
            latest = self._latest_video
            if latest is None or latest[1] <= self._last_video_seq:
                return None
            frame, self._last_video_seq = latest
            return frame.copy()

    def get_odom_message(self) -> Odometry | None:
        with self._lock:
            latest = self._latest_odom
            if latest is None or latest[3] <= self._last_odom_seq:
                return None
            position, quaternion, timestamp, self._last_odom_seq = latest

        return Odometry(
            position=Vector3(float(position[0]), float(position[1]), float(position[2])),
            orientation=Quaternion(
                float(quaternion[1]),
                float(quaternion[2]),
                float(quaternion[3]),
                float(quaternion[0]),
            ),
            ts=timestamp,
            frame_id="world",
        )

    def get_lidar_message(self) -> PointCloud2 | None:
        with self._lock:
            latest = self._latest_lidar
            if latest is None or latest[1] <= self._last_lidar_seq:
                return None
            message, self._last_lidar_seq = latest
            return message

    def _create_stream(
        self,
        getter: Callable[[], T | None],
        frequency: float,
        stream_name: str,
    ) -> Observable[T]:
        def on_subscribe(observer: ObserverBase[T], _scheduler: SchedulerBase | None) -> Disposable:
            if self._is_cleaned_up:
                observer.on_completed()
                return Disposable()

            stop_event = threading.Event()
            self._stream_stop_events.append(stop_event)

            def run() -> None:
                try:
                    while not stop_event.is_set() and not self._is_cleaned_up:
                        data = getter()
                        if data is not None:
                            observer.on_next(data)
                        stop_event.wait(1.0 / frequency)
                except Exception as exc:
                    logger.error(f"{stream_name} stream error: {exc}")
                    observer.on_error(exc)
                else:
                    observer.on_completed()

            thread = threading.Thread(
                target=run,
                name=f"Mujoco{stream_name}Stream",
                daemon=True,
            )
            self._stream_threads.append(thread)
            thread.start()
            return Disposable(stop_event.set)

        return Observable(on_subscribe)

    @functools.cache
    def lidar_stream(self) -> Observable[PointCloud2]:
        return self._create_stream(self.get_lidar_message, LIDAR_FPS, "Lidar")

    @functools.cache
    def odom_stream(self) -> Observable[Odometry]:
        return self._create_stream(self.get_odom_message, ODOM_FREQUENCY, "Odom")

    @functools.cache
    def video_stream(self) -> Observable[Image]:
        def get_video_as_image() -> Image | None:
            frame = self.get_video_frame()
            return Image.from_numpy(frame, format=ImageFormat.RGB) if frame is not None else None

        return self._create_stream(get_video_as_image, VIDEO_FPS, "Video")

    @functools.cache
    def lowstate_stream(self) -> Observable[Any]:
        return empty()

    def _hold_command(self, forward: float, lateral: float, yaw: float) -> None:
        with self._lock:
            self._command[:] = (forward, lateral, yaw)

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        if self._is_cleaned_up:
            return True
        self._hold_command(twist.linear.x, twist.linear.y, twist.angular.z)

        if duration > 0:
            if self._stop_timer is not None:
                self._stop_timer.cancel()

            def stop_later() -> None:
                self.stop_movement()
                self._stop_timer = None

            self._stop_timer = threading.Timer(duration, stop_later)
            self._stop_timer.daemon = True
            self._stop_timer.start()
        return True

    def stop_movement(self) -> None:
        self._hold_command(0.0, 0.0, 0.0)

    def standup(self) -> bool:
        return True

    def liedown(self) -> bool:
        return True

    def balance_stand(self) -> bool:
        return True

    def sport_command(self, api_id: int) -> bool:
        return True

    def set_obstacle_avoidance(self, enabled: bool = True) -> bool:
        return True

    def set_rage_mode(self, enable: bool) -> bool:
        return True

    def set_light(self, level: int) -> bool:
        return True

    def switch_joystick(self, enable: bool = True) -> bool:
        return True

    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        logger.info("Ignoring simulator publish request", topic=topic, data=data)
        return {}
