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


import functools
import threading
import time

from reactivex import Observable

from dimos.msgs.geometry_msgs import Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.simulation.mujoco.mujoco import MujocoThread

LIDAR_FREQUENCY = 10
ODOM_FREQUENCY = 50
VIDEO_FREQUENCY = 30


class MujocoConnection:
    def __init__(self, *args, **kwargs):
        self.mujoco_thread = MujocoThread()

    def start(self):
        self.mujoco_thread.start()

    def standup(self):
        print("standup supressed")

    def liedown(self):
        print("liedown supressed")

    @functools.cache
    def lidar_stream(self):
        def on_subscribe(observer, scheduler):
            stop_event = threading.Event()

            def run():
                while not stop_event.is_set():
                    lidar_to_publish = self.mujoco_thread.get_lidar_message()

                    if lidar_to_publish:
                        observer.on_next(lidar_to_publish)

                    time.sleep(1 / LIDAR_FREQUENCY)

                observer.on_completed()

            thread = threading.Thread(target=run, daemon=True)
            thread.start()

            def dispose():
                stop_event.set()

            return dispose

        return Observable(on_subscribe)

    @functools.cache
    def odom_stream(self):
        print("odom stream start")

        def on_subscribe(observer, scheduler):
            stop_event = threading.Event()

            def run():
                while not stop_event.is_set():
                    odom_to_publish = self.mujoco_thread.get_odom_message()
                    if odom_to_publish:
                        observer.on_next(odom_to_publish)

                    time.sleep(1 / ODOM_FREQUENCY)
                observer.on_completed()

            thread = threading.Thread(target=run, daemon=True)
            thread.start()

            def dispose():
                stop_event.set()

            return dispose

        return Observable(on_subscribe)

    @functools.cache
    def video_stream(self):
        print("video stream start")

        def on_subscribe(observer, scheduler):
            stop_event = threading.Event()

            def run():
                while not stop_event.is_set():
                    with self.mujoco_thread.pixels_lock:
                        if self.mujoco_thread.shared_pixels is not None:
                            img = Image.from_numpy(self.mujoco_thread.shared_pixels.copy())
                            observer.on_next(img)
                    time.sleep(1 / VIDEO_FREQUENCY)
                observer.on_completed()

            thread = threading.Thread(target=run, daemon=True)
            thread.start()

            def dispose():
                stop_event.set()

            return dispose

        return Observable(on_subscribe)

    def move(self, vector: Vector3, duration: float = 0.0):
        self.mujoco_thread.move(vector, duration)
