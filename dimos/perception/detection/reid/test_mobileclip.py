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

import numpy as np
import open_clip
import pytest
import torch
import torch.nn.functional as F
from PIL import Image as PILImage

from dimos.msgs.sensor_msgs import Image
from dimos.utils.data import get_data


@pytest.fixture(scope="session")
def mobileclip_model():
    """Load MobileCLIP model once for all tests."""
    model_name = "MobileCLIP2-S0"
    model_path = get_data("models_mobileclip") / "mobileclip2_s0.pt"
    device = "cuda" if torch.cuda.is_available() else "cpu"

    model, _, preprocess = open_clip.create_model_and_transforms(
        model_name, pretrained=str(model_path)
    )
    tokenizer = open_clip.get_tokenizer(model_name)
    model = model.eval().to(device)

    return {
        "model": model,
        "preprocess": preprocess,
        "tokenizer": tokenizer,
        "device": device,
    }


@pytest.fixture(scope="session")
def test_image():
    """Load test image."""
    return Image.from_file(get_data("cafe.jpg")).to_rgb()


def embed_images(model_dict, pil_images):
    """Embed PIL images using MobileCLIP."""
    model = model_dict["model"]
    preprocess = model_dict["preprocess"]
    device = model_dict["device"]

    with torch.inference_mode():
        batch = torch.stack([preprocess(img) for img in pil_images]).to(device)
        feats = model.encode_image(batch)
        feats = F.normalize(feats, dim=-1)
    return feats.detach().cpu().numpy()


@pytest.mark.heavy
def test_mobileclip_embedding(mobileclip_model, test_image):
    """Test that MobileCLIP can embed the test image."""
    # Convert to PIL
    pil_image = PILImage.fromarray(test_image.to_opencv())

    # Embed
    embedding = embed_images(mobileclip_model, [pil_image])[0]

    print(f"\nEmbedding shape: {embedding.shape}")
    print(f"Embedding dtype: {embedding.dtype}")
    print(f"Embedding norm: {np.linalg.norm(embedding):.4f}")
    print(f"Embedding min/max: [{embedding.min():.4f}, {embedding.max():.4f}]")

    # Validate embedding
    assert embedding.shape[0] > 0, "Embedding should have features"
    assert embedding.dtype == np.float32 or embedding.dtype == np.float64
    assert np.isfinite(embedding).all(), "Embedding should contain finite values"

    # Check L2 normalization (should be ~1.0)
    norm = np.linalg.norm(embedding)
    assert abs(norm - 1.0) < 0.01, f"Embedding should be L2 normalized, got norm={norm}"


@pytest.mark.heavy
def test_mobileclip_text_similarity(mobileclip_model, test_image):
    """Test text-image similarity with MobileCLIP."""
    model = mobileclip_model["model"]
    tokenizer = mobileclip_model["tokenizer"]
    device = mobileclip_model["device"]

    # Get image embedding
    pil_image = PILImage.fromarray(test_image.to_opencv())
    img_embedding = embed_images(mobileclip_model, [pil_image])[0]

    # Encode text queries
    queries = ["a cafe", "a person", "a car", "a dog", "potato", "food", "dinner", "rock"]

    with torch.inference_mode():
        text_tokens = tokenizer(queries).to(device)
        text_features = model.encode_text(text_tokens)
        text_features = F.normalize(text_features, dim=-1)
        text_embeddings = text_features.detach().cpu().numpy()

    # Compute similarities (cosine similarity = 1 - cosine distance)
    similarities = {}
    for query, text_emb in zip(queries, text_embeddings):
        similarity = float(img_embedding @ text_emb)
        similarities[query] = similarity
        print(f"\n'{query}': {similarity:.4f}")

    # Cafe image should match "a cafe" better than "a dog"
    assert similarities["a cafe"] > similarities["a dog"], "Should recognize cafe scene"
    assert similarities["a person"] > similarities["a car"], "Should detect people in cafe"


@pytest.mark.heavy
def test_mobileclip_cosine_distance(mobileclip_model, test_image):
    """Test cosine distance metric for re-identification."""
    pil_image = PILImage.fromarray(test_image.to_opencv())

    # Embed same image twice
    emb1 = embed_images(mobileclip_model, [pil_image])[0]
    emb2 = embed_images(mobileclip_model, [pil_image])[0]

    # Cosine distance between same image should be ~0
    cosine_dist = 1.0 - float(emb1 @ emb2)

    print(f"\nCosine distance (same image): {cosine_dist:.6f}")

    assert cosine_dist < 0.01, f"Same image should have distance ~0, got {cosine_dist}"
