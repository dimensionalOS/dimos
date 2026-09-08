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

"""OmDet-Turbo open-vocabulary detection: text prompts to boxes with per-box scores."""

from __future__ import annotations

from functools import cached_property

import numpy as np
from PIL import Image as PILImage
import torch

from dimos.models.base import HuggingFaceModel, HuggingFaceModelConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


class OmDetConfig(HuggingFaceModelConfig):
    model_name: str = "omlab/omdet-turbo-swin-tiny-hf"
    dtype: torch.dtype = torch.float32


class OmDetDetector(HuggingFaceModel):
    """Real-time text-conditioned open-vocabulary detector (RT-DETR style).

    Same query contract as ``Owlv2Detector.query_detections`` at a fraction
    of the forward cost: Swin-T backbone at 640px against OWLv2's ensemble
    at 960px. Post-processing runs NMS, so returned boxes are already
    deduplicated. Weights come from the Hugging Face hub cache.
    """

    config: OmDetConfig

    @cached_property
    def _model(self):  # type: ignore[no-untyped-def]
        from huggingface_hub import hf_hub_download
        from safetensors.torch import load_file
        from transformers import OmDetTurboConfig, OmDetTurboForObjectDetection

        self._ensure_cuda_initialized()
        # from_pretrained instantiates on the meta device, which leaves the
        # timm Swin backbone's computed non-persistent buffers uninitialized;
        # construct for real and load the checkpoint directly.
        config = OmDetTurboConfig.from_pretrained(self.config.model_name)
        model = OmDetTurboForObjectDetection(config)
        model.load_state_dict(
            load_file(hf_hub_download(self.config.model_name, "model.safetensors"))
        )
        return model.eval().to(self.config.device)

    @cached_property
    def _processor(self):  # type: ignore[no-untyped-def]
        from transformers import AutoProcessor

        return AutoProcessor.from_pretrained(self.config.model_name)

    def query_detections(
        self,
        image: Image,
        queries: list[str],
        threshold: float = 0.3,
    ) -> ImageDetections2D:
        """Detect every query string in the image; boxes below threshold are dropped.

        Each detection's ``name`` is the query text it matched and its
        ``confidence`` is the per-box score. ``class_id`` indexes into
        ``queries``.
        """
        pil = PILImage.fromarray(image.to_rgb().data)
        with torch.inference_mode():
            inputs = self._processor(images=pil, text=queries, return_tensors="pt").to(
                self.config.device
            )
            outputs = self._model(**inputs)
            results = self._processor.post_process_grounded_object_detection(
                outputs,
                text_labels=queries,
                threshold=threshold,
                target_sizes=[(pil.height, pil.width)],
            )[0]

        detections: list[Detection2DBBox] = []
        w, h = float(pil.width), float(pil.height)
        for box, score, label in zip(
            results["boxes"], results["scores"], results["labels"], strict=False
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

        return ImageDetections2D(image=image, detections=detections)

    def query_score_rows(
        self,
        image: Image,
        queries: list[str],
        threshold: float = 0.3,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Score every query against every kept box; no argmax, no label.

        Same selection as the stock post-processing behind
        ``query_detections()``: top-k over the (proposal, query) score
        matrix, the per-box threshold, then class-wise NMS. A proposal
        surviving under several queries returns once with its whole
        ``n_queries`` score row, so a caller can rank queries and refuse.
        The task prompt is fixed instead of the processor default, which
        lists every query in one sentence and truncates it at 77 tokens,
        silently conditioning the decoder on a prefix of a large
        vocabulary. Returns pixel ``(x1, y1, x2, y2)`` boxes and their
        score rows.
        """
        from torchvision.ops.boxes import batched_nms  # type: ignore[import-untyped]
        from transformers.image_transforms import center_to_corners_format

        pil = PILImage.fromarray(image.to_rgb().data)
        with torch.inference_mode():
            inputs = self._processor(
                images=pil, text=queries, task="Detect all objects.", return_tensors="pt"
            ).to(self.config.device)
            outputs = self._model(**inputs)
            rows = torch.sigmoid(outputs.decoder_class_logits[0])
            n_proposals, n_queries = rows.shape
            scores, pairs = rows.flatten().topk(n_proposals, sorted=False)
            passing = scores > threshold
            scores, pairs = scores[passing], pairs[passing]
            scale = torch.tensor([pil.width, pil.height, pil.width, pil.height], device=rows.device)
            boxes = center_to_corners_format(outputs.decoder_coord_logits[0]) * scale
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
