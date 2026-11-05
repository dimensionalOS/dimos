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

"""The transformers-free CLIP pipeline must match the reference transformers one."""

import json

import numpy as np
import pytest

from dimos.perception.experimental.image_embedding import (
    ImageEmbeddingProvider,
    _pixel_values,
    _prepare_image,
)
from dimos.utils.data import get_data

pytestmark = pytest.mark.self_hosted


@pytest.fixture(scope="module")
def provider() -> ImageEmbeddingProvider:
    return ImageEmbeddingProvider()


def test_embeddings_are_unit_vectors(provider: ImageEmbeddingProvider) -> None:
    square = np.zeros((480, 640, 3), dtype=np.uint8)
    square[100:300, 200:400] = (255, 0, 0)  # blue, in BGR
    for embedding in (provider.get_embedding(square), provider.get_text_embedding("a blue square")):
        assert embedding.shape == (512,)
        assert np.isclose(np.linalg.norm(embedding), 1.0, atol=1e-5)


def test_text_ranks_the_matching_image(provider: ImageEmbeddingProvider) -> None:
    cafe = provider.get_embedding(str(get_data("cafe.jpg")))
    scores = {
        text: float(cafe @ provider.get_text_embedding(text))
        for text in ("a photo of a cafe", "a photo of a submarine")
    }
    assert scores["a photo of a cafe"] > scores["a photo of a submarine"]


def test_preprocessing_matches_transformers(provider: ImageEmbeddingProvider) -> None:
    # transformers takes seconds to import; keep it out of test collection.
    from transformers import CLIPImageProcessor, CLIPTokenizerFast

    model_dir = get_data("models_clip")
    with open(model_dir / "preprocessor_config.json") as f:
        reference_images = CLIPImageProcessor(**json.load(f))
    reference_tokens = CLIPTokenizerFast(tokenizer_file=str(model_dir / "tokenizer.json"))

    rng = np.random.default_rng(0)
    images = (
        str(get_data("cafe.jpg")),
        rng.integers(0, 256, (480, 640, 3), dtype=np.uint8),
        rng.integers(0, 256, (37, 100, 3), dtype=np.uint8),
    )
    for image in images:
        pil = _prepare_image(image)
        expected = reference_images(images=pil, return_tensors="np")["pixel_values"]
        np.testing.assert_allclose(_pixel_values(pil), expected, rtol=1e-5, atol=1e-5)

    for text in ("where is the kitchen", "A Blue Square!!", "  odd   spacing ", "word " * 100):
        expected = reference_tokens(text, truncation=True, max_length=77)
        encoding = provider._tokenizer.encode(text)
        assert encoding.ids == expected["input_ids"]
        assert encoding.attention_mask == expected["attention_mask"]
