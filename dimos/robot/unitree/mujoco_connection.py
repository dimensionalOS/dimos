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

"""Shared base for MuJoCo-simulated Unitree connections.

Holds the MuJoCo subprocess + shared-memory IPC, stream wiring, and command
dispatch that every sim backend needs. Robot-specific subclasses customize
the config type, extra ports, start/stop sequencing, camera mounting offset,
and any extra TF transforms.
"""

from __future__ import annotations

import atexit
import base64
from collections.abc import Callable
import functools
import json
import os
from pathlib import Path
import pickle
import subprocess
import sys
import sysconfig
from threading import Event, Thread, Timer
import time
from typing import Any, TypeVar
import weakref

import numpy as np
from reactivex import Observable
from reactivex.abc import ObserverBase, SchedulerBase
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.tf_utils import odom_to_tf
from dimos.robot.unitree.mujoco_camera_constants import MUJOCO_CAMERA_INFO_STATIC
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

T = TypeVar("T")

_ODOM_FREQUENCY = 50


class MujocoConnectionBase(Module):
    """Shared MuJoCo subprocess + stream plumbing for Unitree sim connections."""

    cmd_vel: In[Twist]
    lidar: Out[PointCloud2]
    odom: Out[PoseStamped]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    camera_info_static: CameraInfo = MUJOCO_CAMERA_INFO_STATIC
    _camera_info_thread: Thread | None = None
    _latest_video_frame: Image | None = None

    # Translation from base_link to camera_link. Subclasses override to match
    # their robot's camera mounting position.
    _camera_link_offset: Vector3 = Vector3(0.0, 0.0, 0.0)

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        try:
            import mujoco  # noqa: F401
        except ImportError:
            raise ImportError("'mujoco' is not installed. Use `pip install -e .[sim]`")

        get_data("mujoco_sim")

        # Trigger menagerie download outside the subprocess to avoid timeout there.
        from mujoco_playground._src import mjx_env

        mjx_env.ensure_menagerie_exists()

        self.process: subprocess.Popen[bytes] | None = None
        self.shm_data: Any = None  # ShmWriter, lazily imported
        self._last_video_seq = 0
        self._last_odom_seq = 0
        self._last_lidar_seq = 0
        self._cmd_stop_timer: Timer | None = None

        self._stream_threads: list[Thread] = []
        self._stop_events: list[Event] = []
        self._is_cleaned_up = False
        self._stop_event = Event()

    @rpc
    def start(self) -> None:
        super().start()
        self._start_subprocess()

        def onimage(image: Image) -> None:
            self.color_image.publish(image)
            self._latest_video_frame = image

        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))
        self.register_disposable(self._odom_stream().subscribe(self._publish_tf))
        self.register_disposable(self._lidar_stream().subscribe(self.lidar.publish))
        self.register_disposable(self._video_stream().subscribe(onimage))

        self._camera_info_thread = Thread(
            target=self._publish_camera_info_loop,
            daemon=True,
        )
        self._camera_info_thread.start()

        self._on_start()

    @rpc
    def stop(self) -> None:
        self._on_stop()
        self._stop_event.set()
        self._teardown_subprocess()

        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        super().stop()

    def _on_start(self) -> None:
        """Subclass hook: extra setup after streams are wired (e.g., standup)."""

    def _on_stop(self) -> None:
        """Subclass hook: cleanup before subprocess teardown (e.g., liedown)."""

    def _extra_transforms(self, msg: PoseStamped) -> list[Transform]:
        """Subclass hook: extra TF transforms to publish alongside the standard ones."""
        return []

    def _start_subprocess(self) -> None:
        from dimos.simulation.mujoco.constants import LAUNCHER_PATH
        from dimos.simulation.mujoco.shared_memory import ShmWriter

        self.shm_data = ShmWriter()

        config_pickle = base64.b64encode(pickle.dumps(self.config.g)).decode("ascii")
        shm_names_json = json.dumps(self.shm_data.shm.to_names())

        try:
            # mjpython must be used on macOS (because of launch_passive inside the subprocess).
            # It needs libpython on the dylib search path; uv-installed Pythons
            # use @rpath which doesn't always resolve inside venvs, so we
            # point DYLD_LIBRARY_PATH at the real libpython directory.
            executable = sys.executable if sys.platform != "darwin" else "mjpython"
            env = os.environ.copy()
            if sys.platform == "darwin":
                libdir = Path(sysconfig.get_config_var("LIBDIR") or "")
                if libdir.is_dir():
                    existing = env.get("DYLD_LIBRARY_PATH", "")
                    env["DYLD_LIBRARY_PATH"] = f"{libdir}:{existing}" if existing else str(libdir)

            self.process = subprocess.Popen(
                [executable, str(LAUNCHER_PATH), config_pickle, shm_names_json],
                stderr=subprocess.PIPE,
                env=env,
            )
        except Exception as e:
            self.shm_data.cleanup()
            raise RuntimeError(f"Failed to start MuJoCo subprocess: {e}") from e

        ready_timeout = 300.0
        start_time = time.time()
        assert self.process is not None
        while time.time() - start_time < ready_timeout:
            if self.process.poll() is not None:
                exit_code = self.process.returncode
                self._teardown_subprocess()
                raise RuntimeError(f"MuJoCo process failed to start (exit code {exit_code})")
            if self.shm_data.is_ready():
                logger.info("MuJoCo process started successfully")
                weak_self = weakref.ref(self)

                def cleanup_on_exit(
                    weak_self: weakref.ReferenceType[MujocoConnectionBase] = weak_self,
                ) -> None:
                    instance = weak_self()
                    if instance is not None:
                        instance._teardown_subprocess()

                atexit.register(cleanup_on_exit)
                return
            time.sleep(0.1)

        self._teardown_subprocess()
        raise RuntimeError("MuJoCo process failed to start (timeout)")

    def _teardown_subprocess(self) -> None:
        if self._is_cleaned_up:
            return

        self._is_cleaned_up = True

        if self.process:
            if self.process.stderr:
                self.process.stderr.close()
            if self.process.stdout:
                self.process.stdout.close()

        if self._cmd_stop_timer:
            self._cmd_stop_timer.cancel()
            self._cmd_stop_timer = None

        for stop_event in self._stop_events:
            stop_event.set()

        for thread in self._stream_threads:
            if thread.is_alive():
                thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
                if thread.is_alive():
                    logger.warning(f"Stream thread {thread.name} did not stop gracefully")

        if self.shm_data:
            self.shm_data.signal_stop()

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

        if self.shm_data:
            self.shm_data.cleanup()
            self.shm_data = None

        self._stream_threads.clear()
        self._stop_events.clear()

        self._lidar_stream.cache_clear()
        self._odom_stream.cache_clear()
        self._video_stream.cache_clear()

    def _get_video_frame(self) -> np.ndarray | None:  # type: ignore[type-arg]
        if self.shm_data is None:
            return None

        frame, seq = self.shm_data.read_video()
        if seq > self._last_video_seq:
            self._last_video_seq = seq
            return frame  # type: ignore[no-any-return]

        return None

    def _get_odom_message(self) -> PoseStamped | None:
        if self.shm_data is None:
            return None

        odom_data, seq = self.shm_data.read_odom()
        if seq > self._last_odom_seq and odom_data is not None:
            self._last_odom_seq = seq
            pos, quat_wxyz, timestamp = odom_data

            # Convert quaternion from (w,x,y,z) to (x,y,z,w) for ROS/Dimos.
            from dimos.robot.unitree.type.odometry import Odometry as OdometryMsg

            orientation = Quaternion(quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0])

            return OdometryMsg(
                position=Vector3(pos[0], pos[1], pos[2]),
                orientation=orientation,
                ts=timestamp,
                frame_id="world",
            )

        return None

    def _get_lidar_message(self) -> PointCloud2 | None:
        if self.shm_data is None:
            return None

        lidar_msg, seq = self.shm_data.read_lidar()
        if seq > self._last_lidar_seq and lidar_msg is not None:
            self._last_lidar_seq = seq
            return lidar_msg  # type: ignore[no-any-return]

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

            stop_event = Event()
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

            thread = Thread(target=run, daemon=True)
            self._stream_threads.append(thread)
            thread.start()

            def dispose() -> None:
                stop_event.set()

            return Disposable(dispose)

        return Observable(on_subscribe)

    @functools.cache
    def _lidar_stream(self) -> Observable[PointCloud2]:
        from dimos.simulation.mujoco.constants import LIDAR_FPS

        return self._create_stream(self._get_lidar_message, LIDAR_FPS, "Lidar")

    @functools.cache
    def _odom_stream(self) -> Observable[PoseStamped]:
        return self._create_stream(self._get_odom_message, _ODOM_FREQUENCY, "Odom")

    @functools.cache
    def _video_stream(self) -> Observable[Image]:
        from dimos.simulation.mujoco.constants import VIDEO_FPS

        def get_video_as_image() -> Image | None:
            frame = self._get_video_frame()
            # MuJoCo renderer returns RGB uint8 frames; Image.from_numpy defaults to BGR.
            return Image.from_numpy(frame, format=ImageFormat.RGB) if frame is not None else None

        return self._create_stream(get_video_as_image, VIDEO_FPS, "Video")

    def _publish_camera_info_loop(self) -> None:
        while not self._stop_event.is_set():
            self.camera_info.publish(self.camera_info_static)
            self._stop_event.wait(1.0)

    def _publish_tf(self, msg: PoseStamped) -> None:
        self.odom.publish(msg)
        self.tf.publish(
            *odom_to_tf(
                msg,
                camera_link_offset=self._camera_link_offset,
                extras=self._extra_transforms(msg),
            )
        )

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send movement command to the sim via shared memory."""
        if self._is_cleaned_up or self.shm_data is None:
            return True

        linear = np.array([twist.linear.x, twist.linear.y, twist.linear.z], dtype=np.float32)
        angular = np.array([twist.angular.x, twist.angular.y, twist.angular.z], dtype=np.float32)
        self.shm_data.write_command(linear, angular)

        if duration > 0:
            if self._cmd_stop_timer:
                self._cmd_stop_timer.cancel()

            def stop_movement() -> None:
                if self.shm_data:
                    self.shm_data.write_command(
                        np.zeros(3, dtype=np.float32), np.zeros(3, dtype=np.float32)
                    )
                self._cmd_stop_timer = None

            self._cmd_stop_timer = Timer(duration, stop_movement)
            self._cmd_stop_timer.daemon = True
            self._cmd_stop_timer.start()
        return True

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        logger.info(f"sim publish_request stub: topic={topic} data={data}")
        return {}
