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


import atexit
import functools
import logging
import threading
import time
from typing import List

from reactivex import Observable

from dimos.msgs.geometry_msgs import Twist
from dimos.msgs.sensor_msgs import Image
from dimos.utils.data import get_data

try:
    from dimos.simulation.mujoco.mujoco import MujocoThread
except ImportError:
    MujocoThread = None

LIDAR_FREQUENCY = 10
ODOM_FREQUENCY = 50
VIDEO_FREQUENCY = 30

logger = logging.getLogger(__name__)


class MujocoConnection:
    def __init__(self, *args, **kwargs):
        if MujocoThread is None:
            raise ImportError("'mujoco' is not installed. Use `pip install -e .[sim]`")
        get_data("mujoco_sim")
        self.mujoco_thread = MujocoThread()
        self._stream_threads: List[threading.Thread] = []
        self._stop_events: List[threading.Event] = []
        self._is_cleaned_up = False

        # Register cleanup on exit
        atexit.register(self.cleanup)

    def start(self):
        self.mujoco_thread.start()

    def standup(self):
        print("standup supressed")

    def liedown(self):
        print("liedown supressed")

    @functools.cache
    def lidar_stream(self):
        def on_subscribe(observer, scheduler):
            if self._is_cleaned_up:
                observer.on_completed()
                return lambda: None

            stop_event = threading.Event()
            self._stop_events.append(stop_event)

            def run():
                try:
                    while not stop_event.is_set() and not self._is_cleaned_up:
                        lidar_to_publish = self.mujoco_thread.get_lidar_message()

                        if lidar_to_publish:
                            observer.on_next(lidar_to_publish)

                        time.sleep(1 / LIDAR_FREQUENCY)
                except Exception as e:
                    logger.error(f"Lidar stream error: {e}")
                finally:
                    observer.on_completed()

            thread = threading.Thread(target=run, daemon=True)
            self._stream_threads.append(thread)
            thread.start()

            def dispose():
                stop_event.set()

            return dispose

        return Observable(on_subscribe)

    @functools.cache
    def odom_stream(self):
        print("odom stream start")

        def on_subscribe(observer, scheduler):
            if self._is_cleaned_up:
                observer.on_completed()
                return lambda: None

            stop_event = threading.Event()
            self._stop_events.append(stop_event)

            def run():
                try:
                    while not stop_event.is_set() and not self._is_cleaned_up:
                        odom_to_publish = self.mujoco_thread.get_odom_message()
                        if odom_to_publish:
                            observer.on_next(odom_to_publish)

                        time.sleep(1 / ODOM_FREQUENCY)
                except Exception as e:
                    logger.error(f"Odom stream error: {e}")
                finally:
                    observer.on_completed()

            thread = threading.Thread(target=run, daemon=True)
            self._stream_threads.append(thread)
            thread.start()

            def dispose():
                stop_event.set()

            return dispose

        return Observable(on_subscribe)

    @functools.cache
    def video_stream(self):
        print("video stream start")

        def on_subscribe(observer, scheduler):
            if self._is_cleaned_up:
                observer.on_completed()
                return lambda: None

            stop_event = threading.Event()
            self._stop_events.append(stop_event)

            def run():
                try:
                    while not stop_event.is_set() and not self._is_cleaned_up:
                        with self.mujoco_thread.pixels_lock:
                            if self.mujoco_thread.shared_pixels is not None:
                                img = Image.from_numpy(self.mujoco_thread.shared_pixels.copy())
                                observer.on_next(img)
                        time.sleep(1 / VIDEO_FREQUENCY)
                except Exception as e:
                    logger.error(f"Video stream error: {e}")
                finally:
                    observer.on_completed()

            thread = threading.Thread(target=run, daemon=True)
            self._stream_threads.append(thread)
            thread.start()

            def dispose():
                stop_event.set()

            return dispose

        return Observable(on_subscribe)

    def move(self, twist: Twist, duration: float = 0.0):
        if not self._is_cleaned_up:
            self.mujoco_thread.move(twist, duration)

    def stop(self):
        """Stop the MuJoCo connection gracefully."""
        self.cleanup()

    def cleanup(self):
        """Clean up all resources. Can be called multiple times safely."""
        if self._is_cleaned_up:
            return

        logger.debug("Cleaning up MuJoCo connection resources")
        self._is_cleaned_up = True

        # Stop all stream threads
        for stop_event in self._stop_events:
            stop_event.set()

        # Wait for threads to finish
        for thread in self._stream_threads:
            if thread.is_alive():
                thread.join(timeout=2.0)
                if thread.is_alive():
                    logger.warning(f"Stream thread {thread.name} did not stop gracefully")

        # Clean up the MuJoCo thread
        if hasattr(self, "mujoco_thread") and self.mujoco_thread:
            self.mujoco_thread.cleanup()

        # Clear references
        self._stream_threads.clear()
        self._stop_events.clear()

        # Clear cached methods to prevent memory leaks
        if hasattr(self, "lidar_stream"):
            self.lidar_stream.cache_clear()
        if hasattr(self, "odom_stream"):
            self.odom_stream.cache_clear()
        if hasattr(self, "video_stream"):
            self.video_stream.cache_clear()

    def __del__(self):
        """Destructor to ensure cleanup on object deletion."""
        try:
            self.cleanup()
        except Exception:
            pass
