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

import lcm
import pytest

from dimos.msgs.geometry_msgs import Vector3


@pytest.mark.tool
def test_runpublish() -> None:
    for i in range(10):
        msg = Vector3(-5 + i, -5 + i, i)
        lc = lcm.LCM()
        lc.publish("thing1_vector3#geometry_msgs.Vector3", msg.encode())
        time.sleep(0.1)
        print(f"Published: {msg}")


@pytest.mark.tool
def test_receive() -> None:
    lc = lcm.LCM()

    def receive(bla, msg) -> None:
        # print("receive", bla, msg)
        print(Vector3.decode(msg))

    lc.subscribe("thing1_vector3#geometry_msgs.Vector3", receive)

    def _loop() -> None:
        while True:
            """LCM message handling loop"""
            try:
                lc.handle()
                # loop 10000 times
                for _ in range(10000000):
                    3 + 3  # noqa: B018
            except Exception as e:
                print(f"Error in LCM handling: {e}")

    _loop()
