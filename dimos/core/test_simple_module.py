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

import random
import threading
import time

import pytest

import dimos.core as core
import dimos.protocol.service.lcmservice as lcmservice
from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import Pose, Vector3


class MyComponent(Module):
    ctrl: In[Vector3] = None
    current_pose: Out[Vector3] = None

    @rpc
    def start(self):
        # at start you have self.ctrl and self.current_pose available
        self.ctrl.subscribe(self.handle_ctrl)

    def handle_ctrl(self, target: Vector3):
        print("handling control command:", target)
        self.current_pose.publish(target)


class Controller(Module):
    cmd: Out[Vector3] = None

    def __init__(self, period=1, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.period = period

    @rpc
    def start(self):
        def sendCmd():
            while True:
                time.sleep(self.period)
                vector = Vector3([0, 0, random.uniform(-1, 1)])
                print("sending", vector)
                self.cmd.publish(Vector3([0, 0, 0]))

        thread = threading.Thread(target=sendCmd, daemon=True)
        thread.start()


@pytest.mark.tool
def test_my_component():
    # configures underlying system
    lcmservice.autoconf()
    dimos = core.start(2)

    controller = dimos.deploy(Controller, period=2)
    component = dimos.deploy(MyComponent)

    controller.cmd.transport = core.LCMTransport("/cmd", Vector3)
    component.current_pose.transport = core.LCMTransport("/pos", Vector3)

    controller.cmd.connect(component.ctrl)
    controller.start()
    component.start()

    while True:
        time.sleep(1)


if __name__ == "__main__":
    test_my_component()
