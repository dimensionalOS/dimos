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
    dtype: torch.dtype = torch.float32
    # Do the processor's image preparation in torch on the model's device instead of in
    # numpy on the CPU. OFF by default: it is not bit-identical, and every caller of this
    # detector should opt in deliberately rather than find its scores moved underneath it.
    #
    # WHY IT IS WORTH OPTING IN, measured on an RTX 5070 with 1280x720 frames:
    # `Owlv2ImageProcessor` costs 347 ms a frame against the model's own 439 ms, so more
    # than a third of a detection is numpy. It is slow because the resize is not a plain
    # bilinear -- it is skimage's recipe, a scipy `gaussian_filter` with
    # sigma = (factor - 1) / 2 followed by an order-1 `ndi.zoom`. The same four steps in
    # torch cost **3.5 ms**, a hundredfold.
    #
    # THE COST, stated rather than buried: torch's antialiased bilinear is a different
    # kernel from that gaussian-then-zoom, so the pixels differ (mean 0.001, max 0.17 on a
    # 0-1 scale) and the scores move a little with them -- worst 0.022 over four frames,
    # the same order as float16. Anything with a threshold should be re-checked against
    # its own answers before turning this on, not just against a few frames.
    gpu_preprocess: bool = False
    # Where those four steps run. "" follows the model. The point of naming it separately
    # is that THE WIN WAS NEVER THE GPU -- it is torch instead of skimage, and torch on a
    # CPU is already most of it. MEASURED on an M-series Mac, one 848x480 frame:
    #
    #   the processor's preprocessing, on the cpu   155 ms
    #   these four steps in torch, on the cpu       4.0 ms
    #   the forward pass, on mps                    141 ms
    #
    # Which matters because MPS cannot run them at all: torch has no
    # `aten::_upsample_bilinear2d_aa` there and asking raises mid-run. Preparing on the
    # cpu and moving the result is 39x on a machine that could not use this feature.
    preprocess_device: str = ""


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

    def _pixel_values(self, pils: list) -> torch.Tensor:
        """`Owlv2ImageProcessor`'s four steps, in torch, on the model's device.

        Rescale to 0-1, pad to a square with grey on the bottom and right, resize to the
        checkpoint's side, normalize by the CLIP mean and std -- in that order, which is
        the order the processor uses and the only order that gives the same picture. The
        padding value is 0.5 because the rescale has already happened by then.

        The resize is where this diverges: the processor uses a gaussian pre-filter plus
        an order-1 zoom, and `antialias=True` here is a different kernel with the same
        purpose. See `Owlv2Config.gpu_preprocess` for what that costs in score.
        """
        processor = self._processor.image_processor
        side = processor.size["height"]
        # Where the arithmetic happens, which need not be where the model lives.
        device = self.config.preprocess_device or self.config.device
        mean = torch.tensor(processor.image_mean, device=device).view(1, 3, 1, 1)
        deviation = torch.tensor(processor.image_std, device=device).view(1, 3, 1, 1)

        squares = []
        for pil in pils:
            pixels = torch.from_numpy(np.asarray(pil, dtype=np.uint8)).to(device)
            pixels = pixels.permute(2, 0, 1).unsqueeze(0).float().div_(255.0)
            height, width = pixels.shape[-2:]
            square = max(height, width)
            pixels = torch.nn.functional.pad(
                pixels, (0, square - width, 0, square - height), mode="constant", value=0.5
            )
            squares.append(
                torch.nn.functional.interpolate(
                    pixels,
                    size=(side, side),
                    mode="bilinear",
                    align_corners=False,
                    antialias=True,
                )
            )
        prepared = (torch.cat(squares, dim=0) - mean) / deviation
        return prepared if device == self.config.device else prepared.to(self.config.device)

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
            if self.config.gpu_preprocess:
                # The text side stays with the processor -- it is a tokenizer, it is
                # cheap, and it is the images that cost 347 ms apiece.
                inputs = self._processor(
                    text=[queries] * len(pils), return_tensors="pt", padding=True
                ).to(self.config.device)
                inputs["pixel_values"] = self._pixel_values(pils)
            else:
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
        with torch.inference_mode():
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
