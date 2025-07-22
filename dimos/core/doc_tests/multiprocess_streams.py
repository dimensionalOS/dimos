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

import threading
import time
from typing import Callable

from dimos.core import In, LCMTransport, Module, Out, rpc, start
from dimos.msgs.geometry_msgs import Vector3


class ThreadedCtrlModule(Module):
    # make sure to initialize as None
    control_stream: Out[Vector3] = None

    speed: float
    counter: int = 0

    # we can feed arguments at init to configure the module
    def __init__(self, speed: float = 1.0):
        self.speed = speed
        super().__init__()

    @rpc
    def start(self):
        def loop():
            while True:
                time.sleep(0.125)
                self.control_stream.publish(
                    Vector3(
                        self.counter + self.speed,
                        self.counter + self.speed,
                        self.counter + self.speed,
                    )
                )
                self.counter += 1

        thread = threading.Thread(target=loop, daemon=True)
        thread.start()


class DriveModule(Module):
    driving_direction: In[Vector3] = None
    received_messages: list[Vector3] = []

    def local_function(self): ...

    @rpc
    def remote_function(self):
        return "hi"

    @rpc
    def start(self):
        self.driving_direction.subscribe(self.receive_msg)

    def receive_msg(self, msg: Vector3):
        self.received_messages.append(msg)
        print(f"Received message: {msg}")

    @rpc
    def get_received_messages(self):
        return self.received_messages


if __name__ == "__main__":
    dimos = start(2)

    drive_module = dimos.deploy(DriveModule, speed=1.0)
    ctrl_module = dimos.deploy(ThreadedCtrlModule)

    ctrl_module.control_stream.transport = LCMTransport("/move", Vector3)
    drive_module.driving_direction.connect(ctrl_module.control_stream)

    print(ctrl_module.io())
    print(drive_module.io())

    try:
        drive_module.local_function()
    except AttributeError:
        print("DriveModule has no local_function, callable remotely, as expected.")

    # remote function is callable
    assert drive_module.remote_function() == "hi"

    # Here we can see the details of connections:
    #
    # driving_direction[Vector3] @ DriveModule ◀─ RemoteOut control_stream[Vector3]
    # @ ThreadedCtrlModule-c3e9e5c6-3fec-45b4-945c-93b98e63584f
    # via LCMTransport(/move#geometry_msgs.Vector3

    drive_module.start()
    ctrl_module.start()

    time.sleep(0.4)

    assert drive_module.get_received_messages() == [
        Vector3([1, 1, 1]),
        Vector3([2, 2, 2]),
        Vector3([3, 3, 3]),
    ]

    dimos.shutdown()
