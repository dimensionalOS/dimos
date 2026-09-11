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

from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.detection.reid.embedding_id_system import EmbeddingIDSystem
from dimos.perception.detection.reid.type import IDSystem
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.types.timestamped import align_timestamped
from dimos.utils.reactive import backpressure


class Config(ModuleConfig):
    idsystem: IDSystem


class ReidModule(Module):
    config: Config
    detections: In[Detection2DArray]
    image: In[Image]

    def __init__(self, idsystem: IDSystem | None = None, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        if idsystem is None:
            try:
                from dimos.models.embedding.treid import TorchReIDModel

                idsystem = EmbeddingIDSystem(model=TorchReIDModel, padding=0)
            except Exception as e:
                raise RuntimeError(
                    "TorchReIDModel not available. Please install with: pip install dimos[torchreid]"
                ) from e

        self.idsystem = idsystem

    def detections_stream(self) -> Observable[ImageDetections2D]:
        return backpressure(
            align_timestamped(
                self.image.pure_observable(),
                self.detections.pure_observable().pipe(
                    ops.filter(lambda d: d.detections_length > 0)  # type: ignore[attr-defined]
                ),
                match_tolerance=0.0,
                buffer_size=2.0,
            ).pipe(ops.map(lambda pair: ImageDetections2D.from_ros_detection2d_array(*pair)))  # type: ignore[arg-type, misc]
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
