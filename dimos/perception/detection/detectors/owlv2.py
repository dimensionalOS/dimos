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

from collections import OrderedDict
from functools import cached_property

import numpy as np
from PIL import Image as PILImage
import torch

from dimos.models.base import HuggingFaceModel, HuggingFaceModelConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

# ~3.8 MB of GPU memory per cached frame
_FEATURE_CACHE_MAX = 128


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

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        # image-tower forwards run so far; instrumentation reads deltas
        self.forwards = 0
        # (frame ts, label, floor) to that label's floor-filtered
        # (boxes_kx4, scores_k); filled and read by localize
        self.score_cache: OrderedDict[tuple[float, str, float], tuple[np.ndarray, np.ndarray]] = (
            OrderedDict()
        )
        # frame ts to text-independent tower outputs; any query set scores
        # against a cached frame without rerunning the tower
        self._features: OrderedDict[
            float, tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]
        ] = OrderedDict()
        self._text_embeds: dict[str, torch.Tensor] = {}

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
        self.forwards += len(images)
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
        self.forwards += 1
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

    def _frame_features(
        self, image: Image
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Text-independent tower outputs for one frame, cached by frame ts.

        Returns unit-normalized per-box class embeddings, the class head's
        logit shift and scale, and clipped pixel ``(x1, y1, x2, y2)`` boxes.
        A miss runs the image tower once; a hit runs nothing.
        """
        key = image.ts
        if key in self._features:
            self._features.move_to_end(key)
            return self._features[key]

        from transformers.image_transforms import center_to_corners_format

        self.forwards += 1
        pil = PILImage.fromarray(image.to_rgb().data)
        model = self._model
        head = model.class_head
        with torch.inference_mode(), self._autocast():
            pixel_values = self._processor(images=pil, return_tensors="pt").pixel_values.to(
                self.config.device
            )
            feature_map = model.image_embedder(pixel_values)[0]
            batch, height, width, dim = feature_map.shape
            image_feats = feature_map.reshape(batch, height * width, dim)
            pred_boxes = model.box_predictor(image_feats, feature_map)
            class_embeds = head.dense0(image_feats)
            class_embeds = class_embeds / (
                torch.linalg.norm(class_embeds, dim=-1, keepdim=True) + 1e-6
            )
            logit_shift = head.logit_shift(image_feats)
            logit_scale = head.elu(head.logit_scale(image_feats)) + 1

            # the same conversion post_process_grounded_object_detection runs:
            # corners in model dtype, scaled to the padded square in float32
            boxes = center_to_corners_format(pred_boxes)[0].float() * float(
                max(pil.width, pil.height)
            )
            boxes[:, 0::2] = boxes[:, 0::2].clip(0.0, float(pil.width))
            boxes[:, 1::2] = boxes[:, 1::2].clip(0.0, float(pil.height))

        entry = (class_embeds[0], logit_shift[0], logit_scale[0], boxes)
        self._features[key] = entry
        if len(self._features) > _FEATURE_CACHE_MAX:
            self._features.popitem(last=False)
        return entry

    def _query_embeds(self, queries: list[str]) -> torch.Tensor:
        """Unit-normalized text embeddings, one row per query, cached per string."""
        missing = [q for q in queries if q not in self._text_embeds]
        if missing:
            with torch.inference_mode(), self._autocast():
                inputs = self._processor(text=[missing], return_tensors="pt").to(self.config.device)
                embeds = self._model.owlv2.get_text_features(**inputs)
                embeds = embeds / torch.linalg.norm(embeds, ord=2, dim=-1, keepdim=True)
            for query, embed in zip(missing, embeds, strict=True):
                self._text_embeds[query] = embed
        return torch.stack([self._text_embeds[q] for q in queries])

    def query_score_rows_batch(
        self,
        images: list[Image],
        queries: list[str],
        threshold: float = 0.1,
    ) -> list[tuple[np.ndarray, np.ndarray]]:
        """``query_score_rows`` over several images: one ``(boxes, rows)`` each.

        The image tower runs only for frames missing from the feature cache;
        scoring any query set against a cached frame is a matmul against its
        stored class embeddings, so repeated windows and new labels on seen
        frames cost no model forwards.
        """
        with torch.inference_mode():
            query_embeds = self._query_embeds(queries)
            query_embeds = query_embeds / (
                torch.linalg.norm(query_embeds, dim=-1, keepdim=True) + 1e-6
            )
            out: list[tuple[np.ndarray, np.ndarray]] = []
            for image in images:
                class_embeds, logit_shift, logit_scale, boxes_px = self._frame_features(image)
                logits = (class_embeds @ query_embeds.T + logit_shift) * logit_scale
                scores = torch.sigmoid(logits.to(torch.float32))
                keep = scores.max(dim=-1).values > threshold
                out.append((boxes_px[keep].cpu().numpy(), scores[keep].cpu().numpy()))
        return out

    def stop(self) -> None:
        self.score_cache.clear()
        self._features.clear()
        self._text_embeds.clear()
        if "_processor" in self.__dict__:
            del self.__dict__["_processor"]
        super().stop()
