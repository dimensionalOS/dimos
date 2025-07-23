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
import argparse

import pytest

import dimos.core as core
from dimos.core import Module, In, Out, rpc
from dimos.protocol.service.lcmservice import autoconf
from dimos.msgs.geometry_msgs import Pose, Vector3, Twist
import dimos.protocol.service.lcmservice as lcmservice
from dimos.msgs.sensor_msgs.JointState import JointState

dev_serial = "/dev/ttyUSB0"


class SerialBrige(Module):
    joint_state: In[JointState] = None
    pose_state: Out[Pose] = None

    def __init__(
        self,
        port: str = dev_serial,
        baud: int = 115200,
        timeout: float = 0.1,
        joint_name: str = None,
        *args,
        **kwargs,
    ):
        print(f"Initializing SerialBrige with port {port} and baud {baud} for joint {joint_name}")
        super().__init__(*args, **kwargs)
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.last_position = None
        self.joint_name = joint_name

    @rpc
    def start(self):
        # subscribe to incoming LCM Twist messages
        self.joint_state.subscribe(self._on_joint_state)
        print(f"Subscribed to {self.joint_state}")

    def _on_joint_state(self, msg: JointState):
        print(f"[SerialBrige] Received joint state: {msg}")
        if self.joint_name not in msg.name:
            print(
                f"[SerialBrige] Joint name {self.joint_name} not found in message names: {msg.name}"
            )
            return
        idx = msg.name.index(self.joint_name)
        position = msg.position[idx]
        # Format: <sec,joint_name,position>
        cmd = f"<{msg.header.stamp.sec},{self.joint_name},{position}>"
        # Check if the position has changed and if so, send the command
        if position != self.last_position:
            self.ser.write(cmd.encode("ascii"))
            print(f"[SerialBrige] Sent to serial: {cmd}")
            self.last_position = position
        # mirror your pc_echo.py’s sleep to let Arduino respond
        time.sleep(0.05)

    def _reader(self):
        while True:
            print("Reading from serial")
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


def TestSerialBridge(port, joint_name):
    lcmservice.autoconf()
    dimos = core.start(2)

    torso = dimos.deploy(SerialBrige, port=port, baud=115200, joint_name=joint_name)

    torso.pose_state.transport = core.LCMTransport("/pose", Pose)
    torso.joint_state.transport = core.LCMTransport("/joint_states", JointState)

    torso.start()
    print("SerialBridge started")

    while True:
        time.sleep(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Serial bridge for robot joint control.")
    parser.add_argument(
        "--port", type=str, default=dev_serial, help="Serial device path (e.g. /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--joint",
        type=str,
        required=True,
        help="Joint name to control (must match JointState name)",
    )
    args = parser.parse_args()
    TestSerialBridge(args.port, args.joint)
