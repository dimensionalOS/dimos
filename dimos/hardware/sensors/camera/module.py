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
import queue
import time

from dimos_lcm.sensor_msgs import CameraInfo
import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import Disposable
from reactivex.observable import Observable

from dimos import spec
from dimos.agents import Output, Reducer, Stream, skill  # type: ignore[attr-defined]
from dimos.core import Module, ModuleConfig, Out, rpc
from dimos.hardware.sensors.camera.spec import CameraHardware
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.Image import Image, sharpness_barrier
from dimos.spec import perception as spec  # type: ignore[no-redef]


def default_transform():  # type: ignore[no-untyped-def]
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
    hardware: Callable[[], CameraHardware] | CameraHardware = Webcam  # type: ignore[type-arg]


class CameraModule(Module[CameraModuleConfig], spec.Camera):
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    hardware: CameraHardware = None  # type: ignore[assignment, type-arg]
    _module_subscription: Disposable | None = None
    _camera_info_subscription: Disposable | None = None
    _skill_stream: Observable[Image] | None = None

    config: CameraModuleConfig
    default_config = CameraModuleConfig

    def __init__(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)

    @property
    def hardware_camera_info(self) -> CameraInfo:
        return self.hardware.camera_info

    @rpc
    def start(self):  # type: ignore[no-untyped-def]
        if callable(self.config.hardware):
            self.hardware = self.config.hardware()
        else:
            self.hardware = self.config.hardware

        if self._module_subscription:
            return "already started"

        stream = self.hardware.image_stream().pipe(sharpness_barrier(self.config.frequency))  # type: ignore[attr-defined]
        self._disposables.add(stream.subscribe(self.color_image.publish))

        # camera_info_stream = self.camera_info_stream(frequency=5.0)

        def publish_info(camera_info: CameraInfo):  # type: ignore[no-untyped-def]
            self.camera_info.publish(camera_info)

            if self.config.transform is None:
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

        self._camera_info_subscription = self.camera_info_stream().subscribe(publish_info)  # type: ignore[assignment]
        self._module_subscription = stream.subscribe(self.image.publish)  # type: ignore[attr-defined]

    @skill(stream=Stream.passive, output=Output.image, reducer=Reducer.latest)  # type: ignore[arg-type]
    def video_stream(self) -> Image:  # type: ignore[misc]
        """implicit video stream skill"""
        _queue = queue.Queue(maxsize=1)  # type: ignore[var-annotated]
        self.hardware.image_stream().subscribe(_queue.put)

        yield from iter(_queue.get, None)

    def publish_info(self, camera_info: CameraInfo) -> None:
        self.camera_info.publish(camera_info)

        if self.config.transform is None:
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

    def camera_info_stream(self, frequency: float = 1.0) -> Observable[CameraInfo]:
        def camera_info(_) -> CameraInfo:  # type: ignore[no-untyped-def]
            self.hardware.camera_info.ts = time.time()
            return self.hardware.camera_info

        return rx.interval(1.0 / frequency).pipe(ops.map(camera_info))

    def stop(self):  # type: ignore[no-untyped-def]
        if self._module_subscription:
            self._module_subscription.dispose()
            self._module_subscription = None
        if self._camera_info_subscription:
            self._camera_info_subscription.dispose()
            self._camera_info_subscription = None
        # Also stop the hardware if it has a stop method
        if self.hardware and hasattr(self.hardware, "stop"):
            self.hardware.stop()
        super().stop()
