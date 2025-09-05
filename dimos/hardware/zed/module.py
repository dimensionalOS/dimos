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

import logging
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

import cv2
import numpy as np
import open3d as o3d
from reactivex import interval
from reactivex import operators as ops

try:
    import pyzed.sl as sl
except ImportError:
    sl = None
    logging.warning("ZED SDK not found. Please install pyzed to use ZED camera functionality.")

from abc import abstractmethod, abstractproperty
from typing import Protocol, TypeVar

from dimos_lcm.sensor_msgs import CameraInfo
from reactivex.observable import Observable

from dimos.core import Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import Image, ImageFormat, PointCloud2
from dimos.protocol.service.spec import Service
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.logging_config import setup_logger
from dimos.utils.testing import TimedSensorStorage


class CameraConfig(Protocol):
    frame_id_prefix: str


CameraConfigT = TypeVar("CameraConfigT", bound=CameraConfig)

logger = setup_logger(__name__)


class StereoCamera(Service[CameraConfigT]):
    @abstractmethod
    def pose_stream(self) -> Observable[PoseStamped]:
        pass

    @abstractmethod
    def color_stream(self) -> Observable[Image]:
        pass

    @abstractmethod
    def depth_stream(self) -> Observable[Image]:
        pass

    @abstractmethod
    def pointcloud_stream(self) -> Observable[PointCloud2]:
        pass

    @abstractproperty
    def camera_info(self) -> CameraInfo:
        pass


class MappingStereoCamera(StereoCamera[CameraConfigT]):
    @abstractmethod
    def global_map_stream(self) -> Observable[PointCloud2]:
        pass


class StereoCameraModule(Module):
    color_image: Out[Image] = None
    depth_image: Out[Image] = None
    pointcloud: Out[LidarMessage] = None

    def __init__(
        self,
        camera: Callable[[CameraConfigT], StereoCamera[CameraConfigT]],
        frame_id_prefix: str = "stereo_",
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.frame_id_prefix = frame_id_prefix

        self.storages = None
        if self.recording_path:
            logger.info(f"Recording enabled - saving to {self.recording_path}")

    def _frame(self, name: str) -> str:
        return self.frame_id_prefix + name

    @property
    def camera_info(self) -> CameraInfo:
        return self.connection.camera_info

    @rpc
    def start(self):
        if self.connection is not None:
            raise RuntimeError("Camera already started")
        self.connection = self.camera()
        self.connection.start()

        def maybe_store(name: str, observable: Observable[Any]):
            if not self.recording_path:
                return observable

            store = TimedSensorStorage(f"{self.recording_path}/{name}")
            return store.save_stream(observable)

        maybe_store("pose", self.connection.pose_stream()).subscribe(self._publish_tf)
        maybe_store("color", self.connection.color_stream()).subscribe(self.color_image.publish)
        maybe_store("depth", self.connection.depth_stream()).subscribe(self.depth_image.publish)
        maybe_store("pointcloud", self.connection.pointcloud_stream()).subscribe(
            self.pointcloud.publish
        )

    @rpc
    def stop(self):
        if self.connection is not None:
            self.connection.stop()
            self.connection = None

    def _publish_tf(self, pose: PoseStamped):
        base_tf = Transform.from_pose(self.frame("camera_link"), pose)

        camera_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
            frame_id=self._frame("camera_link"),
            child_frame_id=self._frame("camera_optical"),
            ts=base_tf.ts,
        )

        self.tf.publish(base_tf, camera_optical)

    def cleanup(self):
        """Clean up resources on module destruction."""
        self.stop()
