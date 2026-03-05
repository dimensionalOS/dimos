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

from collections.abc import Callable
from dataclasses import dataclass, field
import time
from typing import Any

import reactivex as rx

from dimos.agents.annotation import skill
from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.camera.spec import CameraHardware
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, sharpness_barrier
from dimos.spec import perception
from dimos.visualization.rerun.bridge import rerun_bridge


def default_transform() -> Transform:
    return Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="base_link",
        child_frame_id="camera_link",
    )


@dataclass
class CameraModuleConfig(ModuleConfig):
    frame_id: str = "camera_link"
    transform: Transform | None = field(default_factory=default_transform)
    hardware: Callable[[], CameraHardware[Any]] | CameraHardware[Any] = Webcam
    frequency: float = 0.0  # Hz, 0 means no limit


class CameraModule(Module[CameraModuleConfig], perception.Camera):
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    hardware: CameraHardware[Any]

    config: CameraModuleConfig
    default_config = CameraModuleConfig
    _global_config: GlobalConfig

    def __init__(self, *args: Any, cfg: GlobalConfig = global_config, **kwargs: Any) -> None:
        self._global_config = cfg
        self._latest_image: Image | None = None
        super().__init__(*args, **kwargs)

    @rpc
    def start(self) -> None:
        super().start()

        if callable(self.config.hardware):
            self.hardware = self.config.hardware()
        else:
            self.hardware = self.config.hardware

        stream = self.hardware.image_stream()

        if self.config.frequency > 0:
            stream = stream.pipe(sharpness_barrier(self.config.frequency))

        def on_image(image: Image) -> None:
            self.color_image.publish(image)
            self._latest_image = image

        self._disposables.add(
            stream.subscribe(on_image),
        )

        self._disposables.add(
            rx.interval(1.0).subscribe(lambda _: self.publish_metadata()),
        )

    def publish_metadata(self) -> None:
        camera_info = self.hardware.camera_info.with_ts(time.time())
        self.camera_info.publish(camera_info)

        if not self.config.transform:
            return

        camera_link = self.config.transform
        camera_link.ts = camera_info.ts

        camera_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
            frame_id="camera_link",
            child_frame_id="camera_optical",
            ts=camera_link.ts,
        )

        self.tf.publish(camera_link, camera_optical)

    @skill
    def take_a_picture(self) -> Image:
        """Grabs and returns the latest image from the camera."""
        if self._latest_image is None:
            raise RuntimeError("No image received from camera yet.")
        return self._latest_image

    def stop(self) -> None:
        if self.hardware and hasattr(self.hardware, "stop"):
            self.hardware.stop()
        super().stop()


camera_module = CameraModule.blueprint

demo_camera = autoconnect(
    camera_module(),
    rerun_bridge(),
)

__all__ = ["CameraModule", "camera_module"]
