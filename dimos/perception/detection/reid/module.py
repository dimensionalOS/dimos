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

from dimos_lcm.foxglove_msgs.ImageAnnotations import (  # type: ignore[import-untyped]
    ImageAnnotations,
    TextAnnotation,
)
from dimos_lcm.foxglove_msgs.Point2 import Point2  # type: ignore[import-untyped]
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.core import In, Module, ModuleConfig, Out, rpc
from dimos.msgs.foxglove_msgs.Color import Color
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.reid.embedding_id_system import EmbeddingIDSystem
from dimos.perception.detection.reid.type import IDSystem
from dimos.perception.detection.type import ImageDetections2D
from dimos.types.timestamped import align_timestamped, to_ros_stamp
from dimos.utils.reactive import backpressure


class Config(ModuleConfig):
    idsystem: IDSystem


class ReidModule(Module):
    default_config = Config

    detections: In[Detection2DArray]
    image: In[Image]
    annotations: Out[ImageAnnotations]

    def __init__(self, idsystem: IDSystem | None = None, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        if idsystem is None:
            try:
                from dimos.models.embedding import TorchReIDModel

                idsystem = EmbeddingIDSystem(model=TorchReIDModel, padding=0)  # type: ignore[arg-type]
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
            ).pipe(ops.map(lambda pair: ImageDetections2D.from_ros_detection2d_array(*pair)))  # type: ignore[misc]
        )

    @rpc
    def start(self) -> None:
        self.detections_stream().subscribe(self.ingress)

    @rpc
    def stop(self) -> None:
        super().stop()

    def ingress(self, imageDetections: ImageDetections2D) -> None:
        text_annotations = []

        for detection in imageDetections:
            # Register detection and get long-term ID
            long_term_id = self.idsystem.register_detection(detection)

            # Skip annotation if not ready yet (long_term_id == -1)
            if long_term_id == -1:
                continue

            # Create text annotation for long_term_id above the detection
            x1, y1, _, _ = detection.bbox
            font_size = imageDetections.image.width / 60

            text_annotations.append(
                TextAnnotation(
                    timestamp=to_ros_stamp(detection.ts),
                    position=Point2(x=x1, y=y1 - font_size * 1.5),
                    text=f"PERSON: {long_term_id}",
                    font_size=font_size,
                    text_color=Color(r=0.0, g=1.0, b=1.0, a=1.0),  # Cyan
                    background_color=Color(r=0.0, g=0.0, b=0.0, a=0.8),
                )
            )

        # Publish annotations (even if empty to clear previous annotations)
        annotations = ImageAnnotations(
            texts=text_annotations,
            texts_length=len(text_annotations),
            points=[],
            points_length=0,
        )
        self.annotations.publish(annotations)
