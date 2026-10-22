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

"""Ideal instantaneous RGB-D acquisition, usable without a DimOS module."""

import math

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoCamera
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.device import SensorDevice


class CameraDevice(SensorDevice[Camera]):
    def open(self) -> None:
        sensor = self.sensor
        self._renderer = MujocoCamera(
            self.reader.model, sensor.width, sensor.height, shadows=sensor.shadows
        )
        self._camera = self.reader.model.camera(self.model_prefix + sensor.model_name).id
        self._frame = f"{self.frame_prefix}{sensor.name}_optical"
        fovy = float(self.reader.model.cam_fovy[self._camera])
        self._focal = sensor.height / (2 * math.tan(math.radians(fovy) / 2))
        self._status["fidelity"] = "ideal_instantaneous"

    def capture(self) -> None:
        if self.sample_time(self.sensor.rate_hz) is None:
            return
        sensor = self.sensor
        rgb, depth = self._renderer.capture(self.reader.data, self._camera, sensor.depth)
        ts = self.reader.timestamp
        position = self.reader.data.cam_xpos[self._camera]
        # MuJoCo looks along -Z; optical frames look along +Z with Y down.
        rotation = self.reader.data.cam_xmat[self._camera].reshape(3, 3) @ np.diag([1, -1, -1])
        self.emit(
            "tf",
            TFMessage(
                Transform(
                    translation=Vector3(*position),
                    rotation=Quaternion(*Rotation.from_matrix(rotation).as_quat()),
                    frame_id="world",
                    child_frame_id=self._frame,
                    ts=ts,
                )
            ),
        )
        info = CameraInfo.from_intrinsics(
            self._focal,
            self._focal,
            (sensor.width - 1) / 2,
            (sensor.height - 1) / 2,
            sensor.width,
            sensor.height,
            frame_id=self._frame,
        )
        info.ts = ts
        color = Image(data=rgb, format=ImageFormat.RGB, frame_id=self._frame, ts=ts)
        self.emit("camera_info", info)
        self.emit("color_image", color)
        if sensor.depth:
            assert depth is not None
            image = Image(data=depth, format=ImageFormat.DEPTH, frame_id=self._frame, ts=ts)
            self.emit("depth_image", image)
            self.emit("depth_camera_info", info)
            if sensor.pointcloud:
                cloud = PointCloud2.from_rgbd(color, image, info, depth_scale=1.0)
                self.emit("pointcloud", cloud.voxel_downsample(0.005))

    def close(self) -> None:
        self._renderer.close()
        super().close()
