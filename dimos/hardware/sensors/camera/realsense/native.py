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

"""The RealSense camera as a native module: rust/ does the capture, this wires it.

Config and ports mirror :class:`RealSenseCamera` one for one, so the two are
interchangeable in a blueprint.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCameraConfig
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.ImuInfo import ImuInfo
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.spec import perception


class NativeRealSenseCameraConfig(RealSenseCameraConfig):
    fps: int = 30
    enable_pointcloud: bool = True
    enable_depth: bool = True
    imu_hz: int = 400
    pointcloud_fps: float = 30


class RealSenseCameraNativeConfig(NativeModuleConfig, NativeRealSenseCameraConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/realsense_native"
    # Own flake: librealsense2 isn't in the root shell.
    build_command: str | None = "nix develop path:. -c cargo build --release"
    stdin_config: bool = True
    # The frame stem and its namespace cross to rust like any other field.
    base_fields: frozenset[str] = frozenset({"frame_id", "frame_id_prefix"})
    # NativeModuleConfig's None would shadow the camera's stem.
    frame_id: str | None = "camera"

    def to_config_dict(self) -> dict[str, Any]:
        config = super().to_config_dict()
        # The rust struct has every key; None crosses as an explicit null.
        config["serial_number"] = self.serial_number
        config["imu_info"] = (
            None
            if self.imu_info is None
            else {
                "gyro_noise_density": self.imu_info.gyro_noise_density,
                "gyro_random_walk": self.imu_info.gyro_random_walk,
                "accel_noise_density": self.imu_info.accel_noise_density,
                "accel_random_walk": self.imu_info.accel_random_walk,
            }
        )
        return config


class RealSenseCameraNative(NativeModule, perception.DepthCamera):
    config: RealSenseCameraNativeConfig
    color_image: Out[Image]
    depth_image: Out[Image]
    infrared_left: Out[Image]
    infrared_right: Out[Image]
    imu: Out[Imu]
    imu_info: Out[ImuInfo]
    pointcloud: Out[PointCloud2]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    infrared_left_camera_info: Out[CameraInfo]
    infrared_right_camera_info: Out[CameraInfo]
    tf: Out[TFMessage]


if TYPE_CHECKING:
    RealSenseCameraNative()
