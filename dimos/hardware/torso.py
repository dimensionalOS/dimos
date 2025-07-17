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
from dimos.msgs.geometry_msgs import Twist, Pose

class TorsoBrige(Module):
 
    twist_cmd: In[Twist] = None
    pose_state: Out[Pose] = None

    def __init__(self, port: str = '/dev/ttyUSB1',
                 baud: int = 9600, timeout: float = 0.1,
                 *args, **kwargs):
        super().__init__(*args, **kwargs)
        # self.ser = serial.Serial(port, baud, timeout=timeout)

    @rpc
    def start(self):
        # subscribe to incoming LCM Twist messages
        self.twist_cmd.subscribe(self._on_twist)

    def _on_twist(self, msg: Twist):
        # take the six floats, format: <lx,ly,lz,ax,ay,az>
        cmd = f"<{msg.linear.x:.3f}," \
              f"{msg.linear.y:.3f}," \
              f"{msg.linear.z:.3f}," \
              f"{msg.angular.x:.3f}," \
              f"{msg.angular.y:.3f}," \
              f"{msg.angular.z:.3f}>"
        self.ser.write(cmd.encode('ascii'))
        # mirror your pc_echo.py’s sleep to let Arduino respond
        time.sleep(0.05)

    def _reader(self):
        while True:
            line = self.ser.readline()
            if not line:
                continue
            text = line.decode('ascii', 'ignore').strip()
            # expect “Pose,0.100,0.200,…”
            if not text.startswith("POSE"):
                continue
            parts = text.split(',')
            if len(parts) != 7:
                continue
            # parse six floats
            lx, ly, lz, ax, ay, az = map(float, parts[1:])
            cp = Pose()
            cp.position.x, cp.position.y, cp.position.z = lx, ly, lz
            cp.orientation.x, cp.orientation.y, cp.orientation.z, cp.orientation.w = ax, ay, az, 1.0
            self.pose_state.publish(cp)
