# Copyright 2026 Dimensional Inc.
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

"""D-FINE closed-set detection: the 80 COCO classes to boxes with per-box scores."""

from __future__ import annotations

from functools import cached_property

import torch

from dimos.models.base import HuggingFaceModel, HuggingFaceModelConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.detectors.base import Detector
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

# The checkpoints spell six classes the Pascal VOC way. These are their COCO names,
# which the YOLO detectors use too.
_COCO_NAMES = {
    "motorbike": "motorcycle",
    "aeroplane": "airplane",
    "sofa": "couch",
    "pottedplant": "potted plant",
    "diningtable": "dining table",
    "tvmonitor": "tv",
}


class DFineConfig(HuggingFaceModelConfig):
    model_name: str = "ustc-community/dfine-small-coco"
    dtype: torch.dtype = torch.float32
    threshold: float = 0.6


class DFineDetector(HuggingFaceModel, Detector):
    """Real-time closed-set detector (DETR style, no NMS).

    The default of the detection modules. The code and the checkpoints have an
    Apache-2.0 licence. There is no tracker, so every ``track_id`` is -1. Weights
    come from the Hugging Face hub cache: a miss downloads once.

    A frame takes 9 ms on an RTX 4070 and 83 ms on a Ryzen 9950X. Without a GPU,
    ``ustc-community/dfine-nano-coco`` takes 42 ms and finds fewer objects.
    """

    config: DFineConfig

    @cached_property
    def _model(self):  # type: ignore[no-untyped-def]
        # Not at the top of the file: importing transformers takes seconds.
        from transformers import DFineForObjectDetection

        self._ensure_cuda_initialized()
        return (
            DFineForObjectDetection.from_pretrained(self.config.model_name)
            .eval()
            .to(self.config.device)
        )

    @cached_property
    def _processor(self):  # type: ignore[no-untyped-def]
        # Not at the top of the file: importing transformers takes seconds.
        from transformers import AutoImageProcessor

        # The torch processor resizes on the model's device. The PIL one costs 5 ms more.
        return AutoImageProcessor.from_pretrained(self.config.model_name, use_fast=True)

    def process_image(self, image: Image) -> ImageDetections2D:
        """Detect the COCO classes in the image. Boxes below the threshold are dropped."""
        with torch.inference_mode():
            inputs = self._processor(
                images=image.to_rgb().data, return_tensors="pt", device=self.config.device
            )
            outputs = self._model(**inputs)
            results = self._processor.post_process_object_detection(
                outputs,
                threshold=self.config.threshold,
                target_sizes=[(image.height, image.width)],
            )[0]

        names = self._model.config.id2label
        detections: list[Detection2DBBox] = []
        w, h = float(image.width), float(image.height)
        for box, score, label in zip(
            results["boxes"].tolist(),
            results["scores"].tolist(),
            results["labels"].tolist(),
            strict=True,
        ):
            x1, y1, x2, y2 = box
            name = names[label]
            det = Detection2DBBox(
                bbox=(max(0.0, x1), max(0.0, y1), min(w, x2), min(h, y2)),
                track_id=-1,
                class_id=label,
                confidence=score,
                name=_COCO_NAMES.get(name, name),
                ts=image.ts,
                image=image,
            )
            if det.is_valid():
                detections.append(det)

        return ImageDetections2D(image=image, detections=detections)

    def stop(self) -> None:
        if "_processor" in self.__dict__:
            del self.__dict__["_processor"]
        super().stop()
