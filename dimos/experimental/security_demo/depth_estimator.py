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

from __future__ import annotations

from collections.abc import Callable
import threading

import numpy as np
from PIL import Image as PILImage
import torch
from transformers import AutoImageProcessor, AutoModelForDepthEstimation

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

_DEPTH_MODEL_NAME = "depth-anything/Depth-Anything-V2-Small-hf"
_DEPTH_MAX_WIDTH = 640


class DepthEstimator:
    """
    Runs depth estimation in a background thread, always processing only the latest image.

    Only intended for visualization (human consumption only).
    """

    def __init__(self, publish: Callable[[Image], None], device: str | None = None) -> None:
        self._publish = publish
        if device is None:
            if torch.cuda.is_available():
                device = "cuda"
            elif torch.backends.mps.is_available():
                device = "mps"
            else:
                device = "cpu"
        self._device = torch.device(device)
        self._processor = AutoImageProcessor.from_pretrained(_DEPTH_MODEL_NAME)
        self._model = AutoModelForDepthEstimation.from_pretrained(_DEPTH_MODEL_NAME).to(
            self._device
        )
        self._latest: Image | None = None
        self._event = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="DepthEstimator")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._event.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    def submit(self, image: Image) -> None:
        """Submit a new image; any unprocessed previous image is discarded."""
        self._latest = image
        self._event.set()

    def _loop(self) -> None:
        while not self._stop.is_set():
            self._event.wait()
            self._event.clear()
            if self._stop.is_set():
                break
            image = self._latest
            if image is not None:
                self._process(image)

    def _process(self, image: Image) -> None:
        rgb = image.to_rgb()
        pil_image = PILImage.fromarray(rgb.data)
        if pil_image.width > _DEPTH_MAX_WIDTH:
            scale = _DEPTH_MAX_WIDTH / pil_image.width
            new_h = int(pil_image.height * scale)
            pil_image = pil_image.resize((_DEPTH_MAX_WIDTH, new_h), PILImage.Resampling.BILINEAR)
        inputs = self._processor(images=pil_image, return_tensors="pt").to(self._device)

        with torch.no_grad():
            outputs = self._model(**inputs)

        depth = torch.nn.functional.interpolate(
            outputs.predicted_depth.unsqueeze(1),
            size=(image.height, image.width),
            mode="bicubic",
            align_corners=False,
        ).squeeze()

        depth_np = depth.cpu().numpy().astype(np.float32)
        self._publish(
            Image.from_numpy(
                depth_np, format=ImageFormat.DEPTH, frame_id=image.frame_id, ts=image.ts
            )
        )
