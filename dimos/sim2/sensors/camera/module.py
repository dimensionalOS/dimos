# Copyright 2026 Dimensional Inc.
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

"""Typed stream/lifecycle wrapper for the camera measurement implementation."""

from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.sensors.camera.device import CameraDevice
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.module import SensorModule, SensorModuleConfig


class CameraModuleConfig(SensorModuleConfig):
    sensor: Camera


class SimCameraModule(SensorModule):
    config: CameraModuleConfig
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    def open(self) -> None:
        self.device = CameraDevice(self.reader, self.config.sensor, self.config.robot_id)


class SimRGBDCameraModule(SimCameraModule):
    depth_image: Out[Image]
    depth_camera_info: Out[CameraInfo]
    pointcloud: Out[PointCloud2]
