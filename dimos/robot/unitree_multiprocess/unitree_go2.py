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

from __future__ import annotations

from typing import Literal, TypeAlias

import numpy as np

from dimos.core import In, Module, Out, RemoteIn, RemoteOut, module, rpc
from dimos.robot.unitree_webrtc.connection import WebRTCRobot
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.types.vector import Vector

VideoMessage: TypeAlias = np.ndarray[tuple[int, int, Literal[3]], np.uint8]


class Robot(Module, WebRTCRobot):
    mov: In[Vector]
    lidar: Out[LidarMessage]
    odometry: Out[Odometry]
    video: Out[VideoMessage]

    def __init__(self, ip: str):
        super().__init__(ip, mode="ai")
        self.lidar = Out(LidarMessage, "lidar", self)
        self.odometry = Out(Odometry, "odometry", self)
        self.mov = In(Vector, "mov", self)

    def start(self):
        self.connect()

        self.odom_stream().subscribe(self.odometry.publish)
        self.lidar_stream().subscribe(self.lidar.publish)
        self.video_stream().subscribe(self.video.publish)
        self.mov.subscribe(self.move)
