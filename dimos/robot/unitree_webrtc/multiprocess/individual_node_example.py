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
import time

from reactivex import operators as ops

from dimos import core
from dimos.core import In, Module, Out
from dimos.msgs.geometry_msgs import Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.protocol import pubsub
from dimos.robot.global_planner import AstarPlanner
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.types.vector import Vector
from dimos.utils.reactive import backpressure, getter_streaming
from dimos.utils.testing import TimedSensorReplay


class DebugModule(Module):
    target: In[Vector] = None

    def start(self):
        self.target.subscribe(lambda x: print("TARGET", x))


if __name__ == "__main__":
    dimos = core.start(1)
    debugModule = dimos.deploy(DebugModule)
    debugModule.target.transport = core.LCMTransport("/clicked_point", Vector3)
    debugModule.start()
    time.sleep(1000)
