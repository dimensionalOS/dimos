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

import pytest
from dimos_lcm.foxglove_msgs import ImageAnnotations, SceneUpdate
from dimos_lcm.sensor_msgs import Image, PointCloud2

from dimos.core import LCMTransport
from dimos.msgs.geometry_msgs import PoseStamped, Transform, Vector3
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2 as PointCloud2Msg
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection2d import testing
from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.perception.detection2d.moduleDB import ObjectDBModule
from dimos.perception.detection2d.type import (
    Detection2D,
    Detection3D,
    ImageDetections2D,
    ImageDetections3D,
)
from dimos.protocol.service import lcmservice as lcm
from dimos.robot.unitree_webrtc.modular import deploy_connection, deploy_navigation
from dimos.robot.unitree_webrtc.modular.connection_module import ConnectionModule
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map


# def test_module3d():
#     lcm.autoconf()

#     for i in range(120):
#         seek_value = 10.0 + (i / 2)
#         moment = testing.detections3d(seek=seek_value)

#         testing.publish_moment(moment)
#         testing.publish_moment(moment)

#         time.sleep(0.1)
