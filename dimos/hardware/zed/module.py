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

from abc import abstractmethod, abstractproperty
from typing import Any, Callable, Protocol, TypeVar

from dimos_lcm.sensor_msgs import CameraInfo
from reactivex.observable import Observable

from dimos.core import Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.protocol.service.spec import Service
from dimos.utils.logging_config import setup_logger
from dimos.utils.testing import TimedSensorStorage


# we insist on cameras to take a frame_id_prefix argument in their config
# so that multiple cameras can be instantiated without frame conflicts
class CameraConfig(Protocol):
    frame_id_prefix: str


CameraConfigT = TypeVar("CameraConfigT", bound=CameraConfig)

logger = setup_logger(__name__)


# StereoCamera interface, for cameras that provide standard
# color, depth, pointcloud, and pose messages
class StereoCameraHardware(Service[CameraConfigT]):
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


# MappingStereoCamera interface, for cameras that also provide a global map stream
# (e.g. ZED with Spatial Mapping)
class MappingStereoCameraHardware(StereoCameraHardware[CameraConfigT]):
    @abstractmethod
    def global_map_stream(self) -> Observable[PointCloud2]:
        pass


# StereoCameraModule - standard dimos module for any StereoCamera implementation
# Handles TF publishing and optional recording of streams
#
# Optionally standard dimos mapper can consume the pointcloud stream
class StereoCamera(Module):
    color_image: Out[Image] = None
    depth_image: Out[Image] = None
    pointcloud: Out[PointCloud2] = None

    def __init__(
        self,
        camera: Callable[[CameraConfigT], StereoCameraHardware[CameraConfigT]],
        frame_id_prefix: str = "stereo_",
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.frame_id_prefix = frame_id_prefix
        self.camera_config: CameraConfigT = CameraConfigT(frame_id_prefix=frame_id_prefix, **kwargs)
        if self.recording_path:
            logger.info(f"Recording enabled - saving to {self.recording_path}")

    def _frame(self, name: str) -> str:
        return self.frame_id_prefix + name

    @property
    def camera_info(self) -> CameraInfo:
        return self.connection.camera_info

    def _maybe_store(self, name: str, observable: Observable[Any]):
        if not self.recording_path:
            return observable

        store = TimedSensorStorage(f"{self.recording_path}/{name}")
        return store.save_stream(observable)

    @rpc
    def start(self):
        if self.connection is not None:
            raise RuntimeError("Camera already started")

        self.connection = self.camera(**self.camera_config)
        self.connection.start()

        self.maybe_store("pose", self.connection.pose_stream()).subscribe(self._publish_tf)
        self.maybe_store("color", self.connection.color_stream()).subscribe(
            self.color_image.publish
        )
        self.maybe_store("depth", self.connection.depth_stream()).subscribe(
            self.depth_image.publish
        )
        self.maybe_store("pointcloud", self.connection.pointcloud_stream()).subscribe(
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


class MappingStereoCamera(StereoCamera):
    global_map: Out[PointCloud2] = None

    @rpc
    def start(self):
        super().start()

        self.maybe_store("global_map", self.connection.global_map_stream()).subscribe(
            self.global_map.publish
        )
