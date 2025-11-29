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

from collections.abc import Callable
from dataclasses import dataclass, field
import queue
import time

from dimos_lcm.sensor_msgs import CameraInfo  # type: ignore[import-untyped]
import reactivex as rx
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos import spec
from dimos.agents2 import Output, Reducer, Stream, skill  # type: ignore[attr-defined]
from dimos.core import Module, ModuleConfig, Out, rpc
from dimos.hardware.camera.spec import CameraHardware
from dimos.hardware.camera.webcam import Webcam
from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.Image import Image, sharpness_barrier


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
    frequency: float = 5.0


class CameraModule(Module, spec.Camera):
    image: Out[Image] = None  # type: ignore[assignment]
    camera_info: Out[CameraInfo] = None  # type: ignore[assignment]

    hardware: Callable[[], CameraHardware] | CameraHardware = None  # type: ignore[assignment, type-arg]
    _skill_stream: Observable[Image] | None = None

    default_config = CameraModuleConfig

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)

    @property
    def hardware_camera_info(self) -> CameraInfo:
        return self.hardware.camera_info  # type: ignore[union-attr]

    @rpc
    def start(self) -> str:  # type: ignore[return]
        if callable(self.config.hardware):  # type: ignore[attr-defined]
            self.hardware = self.config.hardware()  # type: ignore[attr-defined]
        else:
            self.hardware = self.config.hardware  # type: ignore[attr-defined]

        self._disposables.add(self.camera_info_stream().subscribe(self.publish_info))

        stream = self.hardware.image_stream().pipe(sharpness_barrier(self.config.frequency))  # type: ignore[attr-defined, union-attr]
        self._disposables.add(stream.subscribe(self.image.publish))

    @rpc
    def stop(self) -> None:
        if self.hardware and hasattr(self.hardware, "stop"):
            self.hardware.stop()
        super().stop()

    @skill(stream=Stream.passive, output=Output.image, reducer=Reducer.latest)  # type: ignore[arg-type]
    def video_stream(self) -> Image:  # type: ignore[misc]
        """implicit video stream skill"""
        _queue = queue.Queue(maxsize=1)  # type: ignore[var-annotated]
        self.hardware.image_stream().subscribe(_queue.put)  # type: ignore[union-attr]

        yield from iter(_queue.get, None)

    def publish_info(self, camera_info: CameraInfo) -> None:
        self.camera_info.publish(camera_info)  # type: ignore[no-untyped-call]

        if self.config.transform is None:  # type: ignore[attr-defined]
            return

        camera_link = self.config.transform  # type: ignore[attr-defined]
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
            self.hardware.camera_info.ts = time.time()  # type: ignore[union-attr]
            return self.hardware.camera_info  # type: ignore[union-attr]

        return rx.interval(1.0 / frequency).pipe(ops.map(camera_info))


camera_module = CameraModule.blueprint
