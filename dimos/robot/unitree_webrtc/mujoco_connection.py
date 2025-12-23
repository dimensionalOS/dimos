#!/usr/bin/env python3

# Copyright 2025 Dimensional Inc.
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


import base64
from collections.abc import Callable
import functools
import json
import pickle
import subprocess
import sys
import threading
import time
from typing import Any, TypeVar

import numpy as np
from numpy.typing import NDArray
from reactivex import Observable
from reactivex.abc import ObserverBase, SchedulerBase
from reactivex.disposable import Disposable

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import Quaternion, Twist, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.simulation.mujoco.constants import LAUNCHER_PATH, LIDAR_FPS, VIDEO_FPS
from dimos.simulation.mujoco.shared_memory import ShmWriter
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

ODOM_FREQUENCY = 50

logger = setup_logger()

T = TypeVar("T")


class MujocoConnection:
    """MuJoCo simulator connection that runs in a separate subprocess."""

    def __init__(self, global_config: GlobalConfig) -> None:
        try:
            import mujoco  # type: ignore[import-untyped]
        except ImportError:
            raise ImportError("'mujoco' is not installed. Use `pip install -e .[sim]`")

        # Pre-download the mujoco_sim data.
        get_data("mujoco_sim")

        # Trigger the download of the mujoco_menajerie package. This is so it
        # doesn't trigger in the mujoco process where it can time out.
        import mujoco_playground

        self.global_config = global_config
        self.process: subprocess.Popen[bytes] | None = None
        self.shm_data: ShmWriter | None = None
        self._last_video_seq = 0
        self._last_odom_seq = 0
        self._last_lidar_seq = 0
        self._stop_timer: threading.Timer | None = None

        self._stream_threads: list[threading.Thread] = []
        self._stop_events: list[threading.Event] = []
        self._is_cleaned_up = False

    def start(self) -> None:
        self.shm_data = ShmWriter()

        config_pickle = base64.b64encode(pickle.dumps(self.global_config)).decode("ascii")
        shm_names_json = json.dumps(self.shm_data.shm.to_names())

        # Launch the subprocess
        try:
            # mjpython must be used macOS (because of launch_passive inside mujoco_process.py)
            executable = sys.executable if sys.platform != "darwin" else "mjpython"
            self.process = subprocess.Popen(
                [executable, str(LAUNCHER_PATH), config_pickle, shm_names_json],
            )

        except Exception as e:
            self.shm_data.cleanup()
            raise RuntimeError(f"Failed to start MuJoCo subprocess: {e}") from e

        # Wait for process to be ready
        ready_timeout = 300.0
        start_time = time.time()
        assert self.process is not None
        while time.time() - start_time < ready_timeout:
            if self.process.poll() is not None:
                exit_code = self.process.returncode
                self.stop()
                raise RuntimeError(f"MuJoCo process failed to start (exit code {exit_code})")
            if self.shm_data.is_ready():
                logger.info("MuJoCo process started successfully")
                return
            time.sleep(0.1)

        # Timeout
        self.stop()
        raise RuntimeError("MuJoCo process failed to start (timeout)")

    def stop(self) -> None:
        if self._is_cleaned_up:
            return

        self._is_cleaned_up = True

        # clean up open file descriptors
        if self.process:
            if self.process.stderr:
                self.process.stderr.close()
            if self.process.stdout:
                self.process.stdout.close()

        # Cancel any pending timers
        if self._stop_timer:
            self._stop_timer.cancel()
            self._stop_timer = None

        # Stop all stream threads
        for stop_event in self._stop_events:
            stop_event.set()

        # Wait for threads to finish
        for thread in self._stream_threads:
            if thread.is_alive():
                thread.join(timeout=2.0)
                if thread.is_alive():
                    logger.warning(f"Stream thread {thread.name} did not stop gracefully")

        # Signal subprocess to stop
        if self.shm_data:
            self.shm_data.signal_stop()

        # Wait for process to finish
        if self.process:
            try:
                self.process.terminate()
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    logger.warning("MuJoCo process did not stop gracefully, killing")
                    self.process.kill()
                    self.process.wait(timeout=2)
            except Exception as e:
                logger.error(f"Error stopping MuJoCo process: {e}")

            self.process = None

        # Clean up shared memory
        if self.shm_data:
            self.shm_data.cleanup()
            self.shm_data = None

        # Clear references
        self._stream_threads.clear()
        self._stop_events.clear()

        self.lidar_stream.cache_clear()
        self.odom_stream.cache_clear()
        self.video_stream.cache_clear()

    def standup(self) -> bool:
        return True

    def liedown(self) -> bool:
        return True

    def get_video_frame(self) -> NDArray[Any] | None:
        if self.shm_data is None:
            return None

        frame, seq = self.shm_data.read_video()
        if seq > self._last_video_seq:
            self._last_video_seq = seq
            return frame

        return None

    def get_odom_message(self) -> Odometry | None:
        if self.shm_data is None:
            return None

        odom_data, seq = self.shm_data.read_odom()
        if seq > self._last_odom_seq and odom_data is not None:
            self._last_odom_seq = seq
            pos, quat_wxyz, timestamp = odom_data

            # Convert quaternion from (w,x,y,z) to (x,y,z,w) for ROS/Dimos
            orientation = Quaternion(quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0])

            return Odometry(
                position=Vector3(pos[0], pos[1], pos[2]),
                orientation=orientation,
                ts=timestamp,
                frame_id="world",
            )

        return None

    def get_lidar_message(self) -> LidarMessage | None:
        if self.shm_data is None:
            return None

        lidar_msg, seq = self.shm_data.read_lidar()
        if seq > self._last_lidar_seq and lidar_msg is not None:
            self._last_lidar_seq = seq
            return lidar_msg

        return None

    def _create_stream(
        self,
        getter: Callable[[], T | None],
        frequency: float,
        stream_name: str,
    ) -> Observable[T]:
        def on_subscribe(observer: ObserverBase[T], _scheduler: SchedulerBase | None) -> Disposable:
            if self._is_cleaned_up:
                observer.on_completed()
                return Disposable(lambda: None)

            stop_event = threading.Event()
            self._stop_events.append(stop_event)

            def run() -> None:
                try:
                    while not stop_event.is_set() and not self._is_cleaned_up:
                        data = getter()
                        if data is not None:
                            observer.on_next(data)
                        time.sleep(1 / frequency)
                except Exception as e:
                    logger.error(f"{stream_name} stream error: {e}")
                finally:
                    observer.on_completed()

            thread = threading.Thread(target=run, daemon=True)
            self._stream_threads.append(thread)
            thread.start()

            def dispose() -> None:
                stop_event.set()

            return Disposable(dispose)

        return Observable(on_subscribe)

    @functools.cache
    def lidar_stream(self) -> Observable[LidarMessage]:
        return self._create_stream(self.get_lidar_message, LIDAR_FPS, "Lidar")

    @functools.cache
    def odom_stream(self) -> Observable[Odometry]:
        return self._create_stream(self.get_odom_message, ODOM_FREQUENCY, "Odom")

    @functools.cache
    def video_stream(self) -> Observable[Image]:
        def get_video_as_image() -> Image | None:
            frame = self.get_video_frame()
            return Image.from_numpy(frame) if frame is not None else None

        return self._create_stream(get_video_as_image, VIDEO_FPS, "Video")

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        if self._is_cleaned_up or self.shm_data is None:
            return True

        linear = np.array([twist.linear.x, twist.linear.y, twist.linear.z], dtype=np.float32)
        angular = np.array([twist.angular.x, twist.angular.y, twist.angular.z], dtype=np.float32)
        self.shm_data.write_command(linear, angular)

        if duration > 0:
            if self._stop_timer:
                self._stop_timer.cancel()

            def stop_movement() -> None:
                if self.shm_data:
                    self.shm_data.write_command(
                        np.zeros(3, dtype=np.float32), np.zeros(3, dtype=np.float32)
                    )
                self._stop_timer = None

            self._stop_timer = threading.Timer(duration, stop_movement)
            self._stop_timer.daemon = True
            self._stop_timer.start()
        return True

    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        print(f"publishing request, topic={topic}, data={data}")
        return {}
