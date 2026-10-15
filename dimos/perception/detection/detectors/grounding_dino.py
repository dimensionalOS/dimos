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

"""Grounding DINO open-vocabulary detection: text prompts to boxes with per-box scores."""

from __future__ import annotations

from functools import cached_property

from dimos_generated.sensor_msgs.msg import Image
import numpy as np
from PIL import Image as PILImage
import torch

from dimos.models.base import HuggingFaceModel, HuggingFaceModelConfig
from dimos.msgs.image import image_to_rgb
from dimos.msgs.time import to_seconds
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


class GroundingDinoConfig(HuggingFaceModelConfig):
    model_name: str = "IDEA-Research/grounding-dino-tiny"
    dtype: torch.dtype = torch.float32


class GroundingDinoDetector(HuggingFaceModel):
    """Text-grounded open-vocabulary detector (DETR decoder over a BERT caption).

    Same query contract as ``OmDetDetector``. The queries become one caption
    of period-separated phrases, and a box's score for a query is the best
    token probability inside that query's phrase. Weights come from the
    Hugging Face hub cache.
    """

    config: GroundingDinoConfig

    @cached_property
    def _model(self):  # type: ignore[no-untyped-def]
        from transformers import GroundingDinoForObjectDetection

        self._ensure_cuda_initialized()
        return (
            GroundingDinoForObjectDetection.from_pretrained(self.config.model_name)
            .eval()
            .to(self.config.device)
        )

    @cached_property
    def _processor(self):  # type: ignore[no-untyped-def]
        from transformers import AutoProcessor

        return AutoProcessor.from_pretrained(self.config.model_name)

    def query_detections(
        self,
        image: Image,
        queries: list[str],
        threshold: float = 0.35,
    ) -> ImageDetections2D:
        """Detect every query string in the image; boxes below threshold are dropped.

        Each detection's ``name`` is its best-scoring query and its
        ``confidence`` that query's score. ``class_id`` indexes into
        ``queries``.
        """
        boxes, rows = self.query_score_rows(image, queries, threshold)
        detections: list[Detection2DBBox] = []
        for box, row in zip(boxes, rows, strict=True):
            label = int(row.argmax())
            det = Detection2DBBox(
                bbox=(float(box[0]), float(box[1]), float(box[2]), float(box[3])),
                track_id=-1,
                class_id=label,
                confidence=float(row[label]),
                name=queries[label],
                ts=to_seconds(image.header.stamp),
                image=image,
            )
            if det.is_valid():
                detections.append(det)
        return ImageDetections2D(image=image, detections=detections)

    def query_score_rows(
        self,
        image: Image,
        queries: list[str],
        threshold: float = 0.35,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Score every query against every kept box; no argmax, no label.

        Selection is ``OmDetDetector.query_score_rows``': top-k over the
        (box, query) score matrix, the per-box threshold, then class-wise
        NMS, each surviving box once with its whole score row. Returns pixel
        ``(x1, y1, x2, y2)`` boxes and their ``(n_boxes, n_queries)`` score
        rows.
        """
        from torchvision.ops.boxes import batched_nms  # type: ignore[import-untyped]
        from transformers.image_transforms import center_to_corners_format

        pil = PILImage.fromarray(image_to_rgb(image))
        phrases = [" ".join(query.replace(".", " ").lower().split()) for query in queries]
        caption = " ".join(f"{phrase}." for phrase in phrases)
        with torch.inference_mode():
            inputs = self._processor(images=pil, text=caption, return_tensors="pt").to(
                self.config.device
            )
            outputs = self._model(**inputs)
            tokens = torch.sigmoid(outputs.logits[0])
            period = self._processor.tokenizer.convert_tokens_to_ids(".")
            ends = torch.nonzero(inputs.input_ids[0] == period).flatten().tolist()
            rows = torch.zeros(tokens.shape[0], len(queries), device=tokens.device)
            for j, (start, end) in enumerate(zip([1, *(e + 1 for e in ends)], ends, strict=False)):
                if end > start:
                    rows[:, j] = tokens[:, start:end].max(dim=-1).values
            n_proposals, n_queries = rows.shape
            scores, pairs = rows.flatten().topk(n_proposals, sorted=False)
            passing = scores > threshold
            scores, pairs = scores[passing], pairs[passing]
            scale = torch.tensor([pil.width, pil.height, pil.width, pil.height], device=rows.device)
            boxes = center_to_corners_format(outputs.pred_boxes[0]) * scale
            survivors = batched_nms(boxes[pairs // n_queries], scores, pairs % n_queries, 0.5)
            proposals = (pairs[survivors] // n_queries).unique()
            kept_boxes = boxes[proposals].float().cpu().numpy()
            kept_rows = rows[proposals].float().cpu().numpy()

        kept_boxes[:, 0::2] = kept_boxes[:, 0::2].clip(0.0, float(pil.width))
        kept_boxes[:, 1::2] = kept_boxes[:, 1::2].clip(0.0, float(pil.height))
        return kept_boxes, kept_rows

    def stop(self) -> None:
        if "_processor" in self.__dict__:
            del self.__dict__["_processor"]
        super().stop()
