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
import serial

import pytest

import dimos.core as core
from dimos.core import Module, In, Out, rpc
from dimos.protocol.service.lcmservice import autoconf
from dimos.msgs.geometry_msgs import Pose, Vector3, Twist
import dimos.protocol.service.lcmservice as lcmservice

dev_serial = "/dev/ttyUSB0"


class TorsoBrige(Module):
    twist_cmd: In[Twist] = None
    pose_state: Out[Pose] = None

    def __init__(
        self, port: str = dev_serial, baud: int = 9600, timeout: float = 0.1, *args, **kwargs
    ):
        super().__init__(*args, **kwargs)
        self.ser = serial.Serial(port, baud, timeout=timeout)

    @rpc
    def start(self):
        # subscribe to incoming LCM Twist messages
        self.twist_cmd.subscribe(self._on_twist)

    def _on_twist(self, msg: Twist):
        print(f"[TorsoBrige] Received twist: {msg}")
        # take the six floats, format: <lx,ly,lz,ax,ay,az>
        cmd = (
            f"<{msg.linear.x:.3f},"
            f"{msg.linear.y:.3f},"
            f"{msg.linear.z:.3f},"
            f"{msg.angular.x:.3f},"
            f"{msg.angular.y:.3f},"
            f"{msg.angular.z:.3f}>"
        )
        self.ser.write(cmd.encode("ascii"))
        print(f"[TorsoBrige] Sent to serial: {cmd}")
        # mirror your pc_echo.py’s sleep to let Arduino respond
        time.sleep(0.05)

    def _reader(self):
        while True:
            line = self.ser.readline()
            if not line:
                continue
            text = line.decode("ascii", "ignore").strip()
            # expect “Pose,0.100,0.200,…”
            if not text.startswith("POSE"):
                continue
            parts = text.split(",")
            if len(parts) != 7:
                continue
            # parse six floats
            lx, ly, lz, ax, ay, az = map(float, parts[1:])
            cp = Pose()
            cp.position.x, cp.position.y, cp.position.z = lx, ly, lz
            cp.orientation.x, cp.orientation.y, cp.orientation.z, cp.orientation.w = ax, ay, az, 1.0
            self.pose_state.publish(cp)


def TestTorsoBridge():
    lcmservice.autoconf()
    dimos = core.start(2)

    torso = dimos.deploy(TorsoBrige, port=dev_serial, baud=9600)

    torso.pose_state.transport = core.LCMTransport("/pose", Pose)
    torso.twist_cmd.transport = core.LCMTransport("/cmd_vel", Twist)

    torso.start()
    print("TorsoBridge started")

    while True:
        time.sleep(1)


if __name__ == "__main__":
    TestTorsoBridge()
