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

import time
from threading import Event, Thread

from dimos.core import Module, Out, initialize, module, rpc
from dimos.robot.unitree_multiprocess.unitree_go2 import Robot
from dimos.types.vector import Vector


class Mover(Module):
    mov: Out[Vector]
    _stop_event: Event

    def __init__(self):
        self.mov = Out(Vector, "mov", self)
        self._stop_event = Event()

    @rpc
    def start(self):
        self._thread = Thread(target=self.movloop)
        self._thread.start()

    def movloop(self):
        self._stop_event.clear()
        while not self._stop_event.is_set():
            self.mov.publish(Vector(0.0, 0.0, 0.2))
            time.sleep(0.1)  # Add a small delay to prevent excessive publishing

    @rpc
    def stop(self):
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)


def test_mover():
    dimos = initialize()
    mover = dimos.deploy(Mover)

    robot = dimos.deploy(Robot, "192.168.1.1")

    robot.mov.connect(mover.mov)

    robot.start().result()
    mover.start().result()
    time.sleep(3)


if __name__ == "__main__":
    test_mover()
