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

from typing import Callable, Optional

from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,
    TextAnnotation,
)
from dimos_lcm.foxglove_msgs.Point2 import Point2
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.core import In, Module, ModuleConfig, Out, rpc
from dimos.msgs.foxglove_msgs.Color import Color
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.reid.base import EmbeddingModel
from dimos.perception.detection.reid.mobileclip import MobileCLIPModel
from dimos.perception.detection.reid.trackAssociator import TrackAssociator
from dimos.perception.detection.type import ImageDetections2D
from dimos.types.timestamped import align_timestamped, to_ros_stamp
from dimos.utils.reactive import backpressure


class Config(ModuleConfig):
    embedding_model: Optional[Callable[..., "EmbeddingModel"]] = None
    similarity_threshold: float = 0.99


class ReidModule(Module):
    default_config = Config

    detections: In[Detection2DArray] = None  # type: ignore
    image: In[Image] = None  # type: ignore
    annotations: Out[ImageAnnotations] = None  # type: ignore

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.config = Config(**kwargs)
        self.embedding_model = (
            self.config.embedding_model() if self.config.embedding_model else MobileCLIPModel()
        )
        self.associator = (
            TrackAssociator(
                model=self.embedding_model, similarity_threshold=self.config.similarity_threshold
            )
            if self.embedding_model
            else None
        )

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
    def start(self):
        self.detections_stream().subscribe(self.ingress)

    def ingress(self, imageDetections: ImageDetections2D):
        if not self.associator or not self.embedding_model:
            print("No embedding model or associator configured")
            return

        track_ids = []

        # Update embeddings for all detections
        for detection in imageDetections:
            embedding = self.embedding_model.embed(detection.cropped_image(padding=0))
            # embed() with single image returns single Embedding
            assert not isinstance(embedding, list), "Expected single embedding"
            self.associator.update_embedding(detection.track_id, embedding)
            track_ids.append(detection.track_id)

        # Record negative constraints (co-occurrence = different objects)
        self.associator.add_negative_constraints(track_ids)

        # Associate and create annotations
        text_annotations = []
        for detection in imageDetections:
            long_term_id = self.associator.associate(detection.track_id)
            print(
                f"track_id={detection.track_id} -> long_term_id={long_term_id} "
                f"({detection.name}, conf={detection.confidence:.2f})"
            )

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

        # Publish annotations
        if text_annotations:
            annotations = ImageAnnotations(
                texts=text_annotations,
                texts_length=len(text_annotations),
                points=[],
                points_length=0,
            )
            self.annotations.publish(annotations)
