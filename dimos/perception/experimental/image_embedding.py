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

"""CLIP image and text embeddings from the ONNX model in ``data/models_clip``."""

from __future__ import annotations

import base64
import io
import os
import sys
from typing import TYPE_CHECKING

import numpy as np
from PIL import Image
from tokenizers import Tokenizer  # type: ignore[import-untyped]

from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import onnxruntime as ort

logger = setup_logger()

# Preprocessing of openai/clip-vit-base-patch32, from data/models_clip/preprocessor_config.json.
_SIZE = 224
_MEAN = np.array([0.48145466, 0.4578275, 0.40821073], dtype=np.float32)
_STD = np.array([0.26862954, 0.26130258, 0.27577711], dtype=np.float32)
_MAX_TOKENS = 77  # the text encoder's context length


class ImageEmbeddingProvider:
    """Unit-normalised CLIP embeddings of images and text, for similarity search."""

    _session: ort.InferenceSession

    def __init__(self) -> None:
        # Imported here so that importing the spatial memory module (and the
        # blueprints that include it) stays cheap.
        import onnxruntime as ort

        model_dir = get_data("models_clip")
        # Loads cuDNN and cuBLAS from the nvidia wheels. Without it the CUDA
        # provider fails to load unless torch was imported first. A no-op on
        # CPU-only builds and on macOS.
        ort.preload_dlls()  # type: ignore[attr-defined]
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        if sys.platform == "darwin":
            providers = ["CoreMLExecutionProvider", "CPUExecutionProvider"]
        self._session = ort.InferenceSession(str(model_dir / "model.onnx"), providers=providers)
        logger.info("CLIP session ready", providers=self._session.get_providers())
        self._tokenizer = Tokenizer.from_file(str(model_dir / "tokenizer.json"))
        self._tokenizer.enable_truncation(_MAX_TOKENS)

    def get_embedding(self, image: np.ndarray | str | bytes) -> np.ndarray:
        """Embed an image given as a BGR array, a file path, a base64 string or encoded bytes."""
        # The fused model wants all three inputs; the text ones are dummies here.
        outputs = self._session.run(
            ["image_embeds"],
            {
                "pixel_values": _pixel_values(_prepare_image(image)),
                "input_ids": np.zeros((1, 1), dtype=np.int64),
                "attention_mask": np.ones((1, 1), dtype=np.int64),
            },
        )
        return _unit(outputs[0][0])

    def get_text_embedding(self, text: str) -> np.ndarray:
        """Embed a text query into the same space as the images."""
        encoding = self._tokenizer.encode(text)
        outputs = self._session.run(
            ["text_embeds"],
            {
                "pixel_values": np.zeros((1, 3, _SIZE, _SIZE), dtype=np.float32),
                "input_ids": np.array([encoding.ids], dtype=np.int64),
                "attention_mask": np.array([encoding.attention_mask], dtype=np.int64),
            },
        )
        return _unit(outputs[0][0])


def _unit(vector: np.ndarray) -> np.ndarray:
    normalised: np.ndarray = vector / np.linalg.norm(vector)
    return normalised


def _pixel_values(image: Image.Image) -> np.ndarray:
    """What CLIPImageProcessor does: shortest side to 224 bicubic, centre crop, scale, normalise, CHW."""
    image = image.convert("RGB")
    width, height = image.size
    short, long = (width, height) if width <= height else (height, width)
    new_long = int(_SIZE * long / short)
    size = (_SIZE, new_long) if width <= height else (new_long, _SIZE)
    image = image.resize(size, Image.Resampling.BICUBIC)
    left = (size[0] - _SIZE) // 2
    top = (size[1] - _SIZE) // 2
    image = image.crop((left, top, left + _SIZE, top + _SIZE))
    pixels = (np.asarray(image, dtype=np.float32) / 255 - _MEAN) / _STD
    return pixels.transpose(2, 0, 1)[np.newaxis]


def _prepare_image(image: np.ndarray | str | bytes) -> Image.Image:
    if isinstance(image, np.ndarray):
        if image.ndim == 3 and image.shape[2] == 3:
            image = image[:, :, ::-1]  # BGR (OpenCV) to RGB
        return Image.fromarray(np.ascontiguousarray(image))
    if isinstance(image, bytes):
        return Image.open(io.BytesIO(image))
    if os.path.isfile(image):
        return Image.open(image)
    return Image.open(io.BytesIO(base64.b64decode(image)))
