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

from copy import deepcopy
import time

from pydantic import Field, InstanceOf
import reactivex as rx

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.camera.webcam import Webcam, WebcamConfig
from dimos.message_utils import image, time as message_time
from dimos_messages.builtin_interfaces.msg import Time
from dimos_messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_messages.sensor_msgs.msg import CameraInfo, Image
from dimos_messages.std_msgs.msg import Header
from dimos_messages.tf2_msgs.msg import TFMessage
from dimos.spec import perception
from dimos.utils.reactive import quality_barrier
from dimos.visualization.vis_module import vis_module


def default_transform() -> TransformStamped:
    return TransformStamped(
        header=Header(frame_id="base_link", stamp=Time(sec=0, nanosec=0)),
        child_frame_id="camera_link",
        transform=Transform(
            rotation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
            translation=Vector3(x=0.0, y=0.0, z=0.0),
        ),
    )


class WebcamModuleConfig(ModuleConfig):
    frame_id: str = "camera_link"
    transform: InstanceOf[TransformStamped] | None = Field(default_factory=default_transform)
    hardware: WebcamConfig = Field(default_factory=WebcamConfig)
    frequency: float = 0.0  # Hz, 0 means no limit


class WebcamModule(Module, perception.Camera):
    config: WebcamModuleConfig
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    _camera: Webcam | None = None

    @rpc
    def start(self) -> None:
        super().start()

        camera = Webcam(**self.config.hardware.model_dump())
        self._camera = camera

        stream = camera.image_stream()

        if self.config.frequency > 0:
            stream = stream.pipe(quality_barrier(image.sharpness, self.config.frequency))

        self.register_disposable(
            stream.subscribe(self.color_image.publish),
        )

        self.register_disposable(
            rx.interval(1.0).subscribe(lambda _: self.publish_metadata()),
        )

    def publish_metadata(self) -> None:
        camera = self._camera
        if camera is None:
            return
        camera_info = deepcopy(camera.camera_info)
        camera_info.header.stamp = message_time.time_from_nanoseconds(time.time_ns())
        self.camera_info.publish(camera_info)

        if not self.config.transform:
            return

        camera_link = deepcopy(self.config.transform)
        camera_link.header.stamp = camera_info.header.stamp
        camera_optical = TransformStamped(
            header=Header(frame_id="camera_link", stamp=camera_info.header.stamp),
            child_frame_id="camera_optical",
            transform=Transform(
                rotation=Quaternion(x=-0.5, y=0.5, z=-0.5, w=0.5),
                translation=Vector3(x=0.0, y=0.0, z=0.0),
            ),
        )
        self.tf.publish(TFMessage(transforms=[camera_link, camera_optical]))

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
