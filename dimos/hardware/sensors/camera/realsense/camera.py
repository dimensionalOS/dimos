# Copyright 2025-2026 Dimensional Inc.
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


"""The RealSense D4xx camera: rust/ does the capture, this wires it."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.camera.spec import DepthCameraConfig
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.ImuInfo import ImuInfo
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.spec import perception


class RealSenseCameraConfig(NativeModuleConfig, DepthCameraConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/realsense_native"
    # Own flake: librealsense2 isn't in the root shell.
    build_command: str | None = "nix develop path:. -c cargo build --release"
    stdin_config: bool = True
    # The frame stem and its namespace cross to rust like any other field.
    base_fields: frozenset[str] = frozenset({"frame_id", "frame_id_prefix"})

    width: int = 848
    height: int = 480
    fps: int = 15
    # The camera's tf link, what a mount edge points at. Imager frames hang off
    # it with `_link` dropped: d435_link -> d435_color_optical_frame.
    frame_id: str | None = "camera_link"
    align_depth_to_color: bool = True
    enable_depth: bool = True
    enable_color: bool = True
    # On, auto-exposure halves the color frame rate to lengthen exposure.
    color_auto_exposure_priority: bool = False
    enable_pointcloud: bool = False
    enable_infrared: bool = False
    # Dots make depth much better and the IR pair useless for feature tracking.
    emitter_enabled: bool = True
    enable_imu: bool = False
    # Gyro rate, and so the Imu output rate.
    imu_hz: int = 400
    imu_info: ImuInfo | None = Field(
        # default value based on a d455, but this should be measured for every physical camera (every d455,d435,etc)
        default_factory=lambda: ImuInfo(
            gyro_noise_density=2.0e-4,
            gyro_random_walk=1.0e-5,
            accel_noise_density=1.8e-3,
            accel_random_walk=1.0e-4,
        )
    )
    pointcloud_fps: float = 5.0
    # Every n-th pixel goes into the cloud.
    pointcloud_decimation: int = 2
    # Cloud points further than this are dropped; D4xx range varies by model and calibration.
    depth_trunc_m: float = 5.0
    camera_info_fps: float = 1.0
    serial_number: str | None = None

    def to_config_dict(self) -> dict[str, Any]:
        config = super().to_config_dict()
        # Module.frame_id falls back to the class name; rust gets the same stem.
        config["frame_id"] = self.frame_id or "RealSenseCamera"
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


class RealSenseCamera(NativeModule, perception.DepthCamera):
    config: RealSenseCameraConfig
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
    RealSenseCamera()
