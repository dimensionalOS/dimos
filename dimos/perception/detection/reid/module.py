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

from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.vision_msgs.msg import Detection2DArray
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.time import to_seconds
from dimos.perception.detection.reid.embedding_id_system import EmbeddingIDSystem
from dimos.perception.detection.reid.type import IDSystem
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.types.timestamped import TimestampedData, align_timestamped
from dimos.utils.reactive import backpressure


def _timed_image(image: Image) -> TimestampedData[Image]:
    return TimestampedData(image, to_seconds(image.header.stamp))


def _timed_detections(detections: Detection2DArray) -> TimestampedData[Detection2DArray]:
    return TimestampedData(detections, to_seconds(detections.header.stamp))


def _has_detections(detections: Detection2DArray) -> bool:
    return bool(detections.detections)


def _image_detections(
    pair: tuple[TimestampedData[Image], TimestampedData[Detection2DArray]],
) -> ImageDetections2D:
    return ImageDetections2D.from_ros_detection2d_array(pair[0].value, pair[1].value)


class Config(ModuleConfig):
    idsystem: IDSystem


class ReidModule(Module):
    config: Config
    detections: In[Detection2DArray]
    image: In[Image]

    def __init__(self, idsystem: IDSystem | None = None, **kwargs) -> None:  # type: ignore[no-untyped-def]
        if idsystem is None:
            try:
                from dimos.models.embedding.treid import TorchReIDModel

                idsystem = EmbeddingIDSystem(model=TorchReIDModel, padding=0)
            except Exception as e:
                raise RuntimeError(
                    "TorchReIDModel not available. Please install with: pip install dimos[torchreid]"
                ) from e

        super().__init__(idsystem=idsystem, **kwargs)
        self.idsystem = idsystem

    def detections_stream(self) -> Observable[ImageDetections2D]:
        return backpressure(
            align_timestamped(
                self.image.pure_observable().pipe(ops.map(_timed_image)),
                self.detections.pure_observable().pipe(
                    ops.filter(_has_detections), ops.map(_timed_detections)
                ),
                match_tolerance=0.0,
                buffer_size=2.0,
            ).pipe(ops.map(_image_detections))
        )

    @rpc
    def start(self) -> None:
        self.detections_stream().subscribe(self.ingress)

    @rpc
    def stop(self) -> None:
        super().stop()

    def ingress(self, imageDetections: ImageDetections2D) -> None:
        for detection in imageDetections:
            self.idsystem.register_detection(detection)
