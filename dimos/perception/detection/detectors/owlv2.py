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

"""OWLv2 open-vocabulary detection: text prompts to boxes with calibrated scores."""

from __future__ import annotations

from functools import cached_property

import numpy as np
from PIL import Image as PILImage
import torch

from dimos.models.base import HuggingFaceModel, HuggingFaceModelConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


class Owlv2Config(HuggingFaceModelConfig):
    model_name: str = "google/owlv2-base-patch16-ensemble"
    # float16 runs the forward under autocast at roughly half the latency;
    # scores jitter by a few thousandths, so threshold-edge boxes may flip.
    dtype: torch.dtype = torch.float32


class Owlv2Detector(HuggingFaceModel):
    """Text-conditioned open-vocabulary detector with per-box scores.

    Unlike VLM-based proposers, the per-box score is a usable acceptance
    signal: real matches on this rig score well above text-only
    hallucinations, so a threshold plus refusal is meaningful. Weights come
    from the Hugging Face hub cache: a miss downloads once, a hit reuses the
    cache.
    """

    config: Owlv2Config

    @cached_property
    def _model(self):  # type: ignore[no-untyped-def]
        from transformers import Owlv2ForObjectDetection

        self._ensure_cuda_initialized()
        return (
            Owlv2ForObjectDetection.from_pretrained(self.config.model_name)
            .eval()
            .to(self.config.device)
        )

    @cached_property
    def _processor(self):  # type: ignore[no-untyped-def]
        from transformers import Owlv2Processor

        return Owlv2Processor.from_pretrained(self.config.model_name)

    def _autocast(self) -> torch.autocast:
        return torch.autocast(
            device_type="cuda",
            dtype=self.config.dtype,
            enabled=self.config.dtype is not torch.float32 and "cuda" in str(self.config.device),
        )

    def query_detections(
        self,
        image: Image,
        queries: list[str],
        threshold: float = 0.1,
    ) -> ImageDetections2D:
        """Detect every query string in the image; boxes below threshold are dropped.

        Each detection's ``name`` is the query text it matched and its
        ``confidence`` is the calibrated per-box score. ``class_id`` indexes
        into ``queries``.
        """
        return self.query_detections_batch([image], queries, threshold)[0]

    def query_detections_batch(
        self,
        images: list[Image],
        queries: list[str],
        threshold: float = 0.1,
    ) -> list[ImageDetections2D]:
        """``query_detections`` over several images in one forward pass.

        Per-call preprocessing, text encoding and kernel launches amortize
        across the batch, which is what makes many-frame sweeps affordable;
        results are per-image, in input order.
        """
        pils = [PILImage.fromarray(image.to_rgb().data) for image in images]
        with torch.inference_mode(), self._autocast():
            inputs = self._processor(
                text=[queries] * len(pils), images=pils, return_tensors="pt"
            ).to(self.config.device)
            outputs = self._model(**inputs)
            results = self._processor.post_process_grounded_object_detection(
                outputs=outputs,
                target_sizes=torch.tensor([(pil.height, pil.width) for pil in pils]),
                threshold=threshold,
            )

        batch: list[ImageDetections2D] = []
        for image, pil, result in zip(images, pils, results, strict=True):
            detections: list[Detection2DBBox] = []
            w, h = float(pil.width), float(pil.height)
            for box, score, label in zip(
                result["boxes"], result["scores"], result["labels"], strict=False
            ):
                x1, y1, x2, y2 = (float(v) for v in box)
                bbox = (max(0.0, x1), max(0.0, y1), min(w, x2), min(h, y2))
                det = Detection2DBBox(
                    bbox=bbox,
                    track_id=-1,
                    class_id=int(label),
                    confidence=float(score),
                    name=queries[int(label)],
                    ts=image.ts,
                    image=image,
                )
                if det.is_valid():
                    detections.append(det)
            batch.append(ImageDetections2D(image=image, detections=detections))
        return batch

    def query_score_rows(
        self,
        image: Image,
        queries: list[str],
        threshold: float = 0.1,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Score every query against every kept box; no argmax, no label.

        Same forward pass and same per-box threshold as
        ``query_detections()``, which reports one label per box because
        post-processing maxes over the query axis. Here the whole
        ``(n_boxes, n_queries)`` block survives, so a caller can rank
        queries and refuse. Returns pixel ``(x1, y1, x2, y2)`` boxes and
        their score rows.
        """
        pil = PILImage.fromarray(image.to_rgb().data)
        with torch.inference_mode(), self._autocast():
            inputs = self._processor(text=[queries], images=pil, return_tensors="pt").to(
                self.config.device
            )
            outputs = self._model(**inputs)
            results = self._processor.post_process_grounded_object_detection(
                outputs=outputs,
                target_sizes=torch.tensor([(pil.height, pil.width)]),
                threshold=threshold,
            )[0]
            # sigmoid is monotonic, so this mask is the one post-processing
            # applied to its max-over-queries scores: the same boxes, in order.
            scores = torch.sigmoid(outputs.logits[0])
            kept = scores[scores.max(dim=-1).values > threshold].float().cpu().numpy()

        boxes = results["boxes"].float().cpu().numpy()
        boxes[:, 0::2] = boxes[:, 0::2].clip(0.0, float(pil.width))
        boxes[:, 1::2] = boxes[:, 1::2].clip(0.0, float(pil.height))
        return boxes, kept

    def stop(self) -> None:
        if "_processor" in self.__dict__:
            del self.__dict__["_processor"]
        super().stop()
