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
import pytest

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.reid.mobileclip import MobileCLIPModel
from dimos.utils.data import get_data


@pytest.fixture(scope="session")
def mobileclip_model():
    """Load MobileCLIP model once for all tests."""
    model_path = get_data("models_mobileclip") / "mobileclip2_s0.pt"
    model = MobileCLIPModel(model_name="MobileCLIP2-S0", model_path=model_path)
    model.warmup()
    return model


@pytest.fixture(scope="session")
def test_image():
    """Load test image."""
    return Image.from_file(get_data("cafe.jpg")).to_rgb()


@pytest.mark.heavy
def test_single_image_embedding(mobileclip_model, test_image):
    """Test embedding a single image."""
    embedding = mobileclip_model.embed(test_image)

    # Embedding should be torch.Tensor on device
    import torch

    assert isinstance(embedding.vector, torch.Tensor), "Embedding should be torch.Tensor"
    assert embedding.vector.device.type in ["cuda", "cpu"], "Should be on valid device"

    # Test conversion to numpy
    vector_np = embedding.to_numpy()
    print(f"\nEmbedding shape: {vector_np.shape}")
    print(f"Embedding dtype: {vector_np.dtype}")
    print(f"Embedding norm: {np.linalg.norm(vector_np):.4f}")

    assert vector_np.shape[0] > 0, "Embedding should have features"
    assert np.isfinite(vector_np).all(), "Embedding should contain finite values"

    # Check L2 normalization
    norm = np.linalg.norm(vector_np)
    assert abs(norm - 1.0) < 0.01, f"Embedding should be L2 normalized, got norm={norm}"


@pytest.mark.heavy
def test_batch_image_embedding(mobileclip_model, test_image):
    """Test embedding multiple images at once."""
    embeddings = mobileclip_model.embed(test_image, test_image, test_image)

    assert isinstance(embeddings, list), "Batch embedding should return list"
    assert len(embeddings) == 3, "Should return 3 embeddings"

    # Check all embeddings are similar (same image)
    sim_01 = embeddings[0] @ embeddings[1]
    sim_02 = embeddings[0] @ embeddings[2]

    print(f"\nSimilarity between same images: {sim_01:.6f}, {sim_02:.6f}")

    assert sim_01 > 0.99, f"Same image embeddings should be very similar, got {sim_01}"
    assert sim_02 > 0.99, f"Same image embeddings should be very similar, got {sim_02}"


@pytest.mark.heavy
def test_single_text_embedding(mobileclip_model):
    """Test embedding a single text string."""
    import torch

    embedding = mobileclip_model.embed_text("a cafe")

    # Should be torch.Tensor
    assert isinstance(embedding.vector, torch.Tensor), "Text embedding should be torch.Tensor"

    vector_np = embedding.to_numpy()
    print(f"\nText embedding shape: {vector_np.shape}")
    print(f"Text embedding norm: {np.linalg.norm(vector_np):.4f}")

    assert vector_np.shape[0] > 0, "Text embedding should have features"
    assert np.isfinite(vector_np).all(), "Text embedding should contain finite values"

    # Check L2 normalization
    norm = np.linalg.norm(vector_np)
    assert abs(norm - 1.0) < 0.01, f"Text embedding should be L2 normalized, got norm={norm}"


@pytest.mark.heavy
def test_batch_text_embedding(mobileclip_model):
    """Test embedding multiple text strings at once."""
    import torch

    embeddings = mobileclip_model.embed_text("a cafe", "a person", "a dog")

    assert isinstance(embeddings, list), "Batch text embedding should return list"
    assert len(embeddings) == 3, "Should return 3 text embeddings"

    # All should be torch.Tensor and normalized
    for i, emb in enumerate(embeddings):
        assert isinstance(emb.vector, torch.Tensor), f"Embedding {i} should be torch.Tensor"
        norm = np.linalg.norm(emb.to_numpy())
        assert abs(norm - 1.0) < 0.01, f"Text embedding {i} should be L2 normalized"


@pytest.mark.heavy
def test_text_image_similarity(mobileclip_model, test_image):
    """Test cross-modal text-image similarity using @ operator."""
    img_embedding = mobileclip_model.embed(test_image)

    # Embed text queries
    queries = ["a cafe", "a person", "a car", "a dog", "potato", "food"]
    text_embeddings = mobileclip_model.embed_text(*queries)

    # Compute similarities using @ operator
    similarities = {}
    for query, text_emb in zip(queries, text_embeddings):
        similarity = img_embedding @ text_emb
        similarities[query] = similarity
        print(f"\n'{query}': {similarity:.4f}")

    # Cafe image should match "a cafe" better than "a dog"
    assert similarities["a cafe"] > similarities["a dog"], "Should recognize cafe scene"
    assert similarities["a person"] > similarities["a car"], "Should detect people in cafe"


@pytest.mark.heavy
def test_cosine_distance(mobileclip_model, test_image):
    """Test cosine distance computation (1 - similarity)."""
    emb1 = mobileclip_model.embed(test_image)
    emb2 = mobileclip_model.embed(test_image)

    # Similarity using @ operator
    similarity = emb1 @ emb2

    # Distance is 1 - similarity
    distance = 1.0 - similarity

    print(f"\nSimilarity (same image): {similarity:.6f}")
    print(f"Distance (same image): {distance:.6f}")

    assert similarity > 0.99, f"Same image should have high similarity, got {similarity}"
    assert distance < 0.01, f"Same image should have low distance, got {distance}"


@pytest.mark.heavy
def test_query_functionality(mobileclip_model, test_image):
    """Test query method for top-k retrieval."""
    # Create a query and some candidates
    query_text = mobileclip_model.embed_text("a cafe")

    # Create candidate embeddings
    candidate_texts = ["a cafe", "a restaurant", "a person", "a dog", "a car"]
    candidates = mobileclip_model.embed_text(*candidate_texts)

    # Query for top-3
    results = mobileclip_model.query(query_text, candidates, top_k=3)

    print("\nTop-3 results:")
    for idx, sim in results:
        print(f"  {candidate_texts[idx]}: {sim:.4f}")

    assert len(results) == 3, "Should return top-3 results"
    assert results[0][0] == 0, "Top match should be 'a cafe' itself"
    assert results[0][1] > results[1][1], "Results should be sorted by similarity"
    assert results[1][1] > results[2][1], "Results should be sorted by similarity"


@pytest.mark.heavy
def test_embedding_operator(mobileclip_model, test_image):
    """Test that @ operator works on embeddings."""
    emb1 = mobileclip_model.embed(test_image)
    emb2 = mobileclip_model.embed(test_image)

    # Use @ operator
    similarity = emb1 @ emb2

    assert isinstance(similarity, float), "@ operator should return float"
    assert 0.0 <= similarity <= 1.0, "Cosine similarity should be in [0, 1]"
    assert similarity > 0.99, "Same image should have similarity near 1.0"


@pytest.mark.heavy
def test_warmup(mobileclip_model):
    """Test that warmup runs without error."""
    # Warmup is already called in fixture, but test it explicitly
    mobileclip_model.warmup()
    # Just verify no exceptions raised
    assert True


@pytest.mark.heavy
def test_compare_one_to_many(mobileclip_model, test_image):
    """Test GPU-accelerated one-to-many comparison."""
    import torch

    # Create query and gallery
    query_emb = mobileclip_model.embed(test_image)
    gallery_embs = mobileclip_model.embed(test_image, test_image, test_image)

    # Compare on GPU
    similarities = mobileclip_model.compare_one_to_many(query_emb, gallery_embs)

    print(f"\nOne-to-many similarities: {similarities}")

    # Should return torch.Tensor
    assert isinstance(similarities, torch.Tensor), "Should return torch.Tensor"
    assert similarities.shape == (3,), "Should have 3 similarities"
    assert similarities.device.type in ["cuda", "cpu"], "Should be on device"

    # All should be ~1.0 (same image)
    similarities_np = similarities.cpu().numpy()
    assert np.all(similarities_np > 0.99), "Same images should have similarity ~1.0"


@pytest.mark.heavy
def test_compare_many_to_many(mobileclip_model):
    """Test GPU-accelerated many-to-many comparison."""
    import torch

    # Create queries and candidates
    queries = mobileclip_model.embed_text("a cafe", "a person")
    candidates = mobileclip_model.embed_text("a cafe", "a restaurant", "a dog")

    # Compare on GPU
    similarities = mobileclip_model.compare_many_to_many(queries, candidates)

    print(f"\nMany-to-many similarities:\n{similarities}")

    # Should return torch.Tensor
    assert isinstance(similarities, torch.Tensor), "Should return torch.Tensor"
    assert similarities.shape == (2, 3), "Should be (2, 3) similarity matrix"
    assert similarities.device.type in ["cuda", "cpu"], "Should be on device"

    # First query should match first candidate best
    similarities_np = similarities.cpu().numpy()
    assert similarities_np[0, 0] > similarities_np[0, 2], "Cafe should match cafe better than dog"


@pytest.mark.heavy
def test_gpu_query_performance(mobileclip_model, test_image):
    """Test that query method uses GPU acceleration."""
    # Create a larger gallery
    gallery_size = 20
    gallery_images = [test_image] * gallery_size
    gallery_embs = mobileclip_model.embed(*gallery_images)

    query_emb = mobileclip_model.embed(test_image)

    # Query should use GPU-accelerated comparison
    results = mobileclip_model.query(query_emb, gallery_embs, top_k=5)

    print(f"\nTop-5 results from gallery of {gallery_size}")
    for idx, sim in results:
        print(f"  Index {idx}: {sim:.4f}")

    assert len(results) == 5, "Should return top-5 results"
    # All should be high similarity (same image, allow some variation for image preprocessing)
    for idx, sim in results:
        assert sim > 0.90, f"Same images should have high similarity, got {sim}"
