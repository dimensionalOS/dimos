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

import time

from pydantic import Field
import reactivex as rx

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.stream import Out
from dimos.core.tf_module import TfModule, TfModuleConfig
from dimos.hardware.sensors.camera.webcam import Webcam, WebcamConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, sharpness_barrier
from dimos.spec import perception
from dimos.visualization.vis_module import vis_module


def default_transform() -> Transform:
    return Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="base_link",
        child_frame_id="camera_link",
    )


class WebcamModuleConfig(TfModuleConfig):
    frame_id: str = "camera_link"
    # "parent" is what the camera is mounted on; local even when it's world, so a namespaced camera stays on its robot.
    tf_frames_local: dict[str, str] = {
        "parent": "base_link",
        "camera_link": "camera_link",
        "camera_optical": "camera_optical",
    }
    tf_edges: list[tuple[str, str]] = [("parent", "camera_link"), ("camera_link", "camera_optical")]
    # The mount pose, parent -> camera_link; only its translation and rotation are used.
    transform: Transform | None = Field(default_factory=default_transform)
    hardware: WebcamConfig = Field(default_factory=WebcamConfig)
    frequency: float = 0.0  # Hz, 0 means no limit


class WebcamModule(TfModule, perception.Camera):
    config: WebcamModuleConfig
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    _camera: Webcam | None = None

    @rpc
    def start(self) -> None:
        super().start()

        camera = Webcam(**self.config.hardware.model_dump())
        self._camera = camera

        stream = camera.image_stream()

        if self.config.frequency > 0:
            stream = stream.pipe(sharpness_barrier(self.config.frequency))

        self.register_disposable(
            stream.subscribe(self._publish_image),
        )

        self.register_disposable(
            rx.interval(1.0).subscribe(lambda _: self.publish_metadata()),
        )

    def _publish_image(self, image: Image) -> None:
        image.frame_id = self.frame.camera_optical
        self.color_image.publish(image)

    def publish_metadata(self) -> None:
        camera = self._camera
        if camera is None:
            return
        camera_info = camera.camera_info.with_ts(time.time())
        camera_info.frame_id = self.frame.camera_optical
        self.camera_info.publish(camera_info)

        if not self.config.transform:
            return

        camera_link = Transform(
            translation=self.config.transform.translation,
            rotation=self.config.transform.rotation,
            frame_id=self.frame.parent,
            child_frame_id=self.frame.camera_link,
            ts=camera_info.ts,
        )

        camera_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
            frame_id=self.frame.camera_link,
            child_frame_id=self.frame.camera_optical,
            ts=camera_link.ts,
        )

        self.tfbuffer.publish(camera_link, camera_optical)

    @rpc
    def stop(self) -> None:
        camera = self._camera
        self._camera = None
        if camera is not None:
            camera.stop()
        super().stop()


demo_camera = autoconnect(
    WebcamModule.blueprint(),
    vis_module(viewer_backend=global_config.viewer),
)
