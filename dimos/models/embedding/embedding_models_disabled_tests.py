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
from dimos.utils.data import get_data


@pytest.fixture(scope="session", params=["clip", "mobileclip", "treid"])
def embedding_model(request):  # type: ignore[no-untyped-def]
    """Load embedding model once for all tests. Parametrized for different models."""
    if request.param == "mobileclip":
        from dimos.models.embedding.mobileclip import MobileCLIPModel

        model_path = get_data("models_mobileclip") / "mobileclip2_s0.pt"
        model = MobileCLIPModel(model_name="MobileCLIP2-S0", model_path=model_path)
    elif request.param == "clip":
        from dimos.models.embedding.clip import CLIPModel

        model = CLIPModel(model_name="openai/clip-vit-base-patch32")  # type: ignore[assignment]
    elif request.param == "treid":
        from dimos.models.embedding.treid import TorchReIDModel

        model = TorchReIDModel(model_name="osnet_x1_0")  # type: ignore[assignment]
    else:
        raise ValueError(f"Unknown model: {request.param}")

    model.warmup()
    return model


@pytest.fixture(scope="session")
def test_image():  # type: ignore[no-untyped-def]
    """Load test image."""
    return Image.from_file(get_data("cafe.jpg")).to_rgb()  # type: ignore[arg-type]


@pytest.mark.heavy
def test_single_image_embedding(embedding_model, test_image) -> None:  # type: ignore[no-untyped-def]
    """Test embedding a single image."""
    embedding = embedding_model.embed(test_image)

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
def test_batch_image_embedding(embedding_model, test_image) -> None:  # type: ignore[no-untyped-def]
    """Test embedding multiple images at once."""
    embeddings = embedding_model.embed(test_image, test_image, test_image)

    assert isinstance(embeddings, list), "Batch embedding should return list"
    assert len(embeddings) == 3, "Should return 3 embeddings"

    # Check all embeddings are similar (same image)
    sim_01 = embeddings[0] @ embeddings[1]
    sim_02 = embeddings[0] @ embeddings[2]

    print(f"\nSimilarity between same images: {sim_01:.6f}, {sim_02:.6f}")

    assert sim_01 > 0.99, f"Same image embeddings should be very similar, got {sim_01}"
    assert sim_02 > 0.99, f"Same image embeddings should be very similar, got {sim_02}"


@pytest.mark.heavy
def test_single_text_embedding(embedding_model) -> None:  # type: ignore[no-untyped-def]
    """Test embedding a single text string."""
    import torch

    if not hasattr(embedding_model, "embed_text"):
        pytest.skip("Model does not support text embeddings")

    embedding = embedding_model.embed_text("a cafe")

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
def test_batch_text_embedding(embedding_model) -> None:  # type: ignore[no-untyped-def]
    """Test embedding multiple text strings at once."""
    import torch

    if not hasattr(embedding_model, "embed_text"):
        pytest.skip("Model does not support text embeddings")

    embeddings = embedding_model.embed_text("a cafe", "a person", "a dog")

    assert isinstance(embeddings, list), "Batch text embedding should return list"
    assert len(embeddings) == 3, "Should return 3 text embeddings"

    # All should be torch.Tensor and normalized
    for i, emb in enumerate(embeddings):
        assert isinstance(emb.vector, torch.Tensor), f"Embedding {i} should be torch.Tensor"
        norm = np.linalg.norm(emb.to_numpy())
        assert abs(norm - 1.0) < 0.01, f"Text embedding {i} should be L2 normalized"


@pytest.mark.heavy
def test_text_image_similarity(embedding_model, test_image) -> None:  # type: ignore[no-untyped-def]
    """Test cross-modal text-image similarity using @ operator."""
    if not hasattr(embedding_model, "embed_text"):
        pytest.skip("Model does not support text embeddings")

    img_embedding = embedding_model.embed(test_image)

    # Embed text queries
    queries = ["a cafe", "a person", "a car", "a dog", "potato", "food"]
    text_embeddings = embedding_model.embed_text(*queries)

    # Compute similarities using @ operator
    similarities = {}
    for query, text_emb in zip(queries, text_embeddings, strict=False):
        similarity = img_embedding @ text_emb
        similarities[query] = similarity
        print(f"\n'{query}': {similarity:.4f}")

    # Cafe image should match "a cafe" better than "a dog"
    assert similarities["a cafe"] > similarities["a dog"], "Should recognize cafe scene"
    assert similarities["a person"] > similarities["a car"], "Should detect people in cafe"


@pytest.mark.heavy
def test_cosine_distance(embedding_model, test_image) -> None:  # type: ignore[no-untyped-def]
    """Test cosine distance computation (1 - similarity)."""
    emb1 = embedding_model.embed(test_image)
    emb2 = embedding_model.embed(test_image)

    # Similarity using @ operator
    similarity = emb1 @ emb2

    # Distance is 1 - similarity
    distance = 1.0 - similarity

    print(f"\nSimilarity (same image): {similarity:.6f}")
    print(f"Distance (same image): {distance:.6f}")

    assert similarity > 0.99, f"Same image should have high similarity, got {similarity}"
    assert distance < 0.01, f"Same image should have low distance, got {distance}"


@pytest.mark.heavy
def test_query_functionality(embedding_model, test_image) -> None:  # type: ignore[no-untyped-def]
    """Test query method for top-k retrieval."""
    if not hasattr(embedding_model, "embed_text"):
        pytest.skip("Model does not support text embeddings")

    # Create a query and some candidates
    query_text = embedding_model.embed_text("a cafe")

    # Create candidate embeddings
    candidate_texts = ["a cafe", "a restaurant", "a person", "a dog", "a car"]
    candidates = embedding_model.embed_text(*candidate_texts)

    # Query for top-3
    results = embedding_model.query(query_text, candidates, top_k=3)

    print("\nTop-3 results:")
    for idx, sim in results:
        print(f"  {candidate_texts[idx]}: {sim:.4f}")

    assert len(results) == 3, "Should return top-3 results"
    assert results[0][0] == 0, "Top match should be 'a cafe' itself"
    assert results[0][1] > results[1][1], "Results should be sorted by similarity"
    assert results[1][1] > results[2][1], "Results should be sorted by similarity"


@pytest.mark.heavy
def test_embedding_operator(embedding_model, test_image) -> None:  # type: ignore[no-untyped-def]
    """Test that @ operator works on embeddings."""
    emb1 = embedding_model.embed(test_image)
    emb2 = embedding_model.embed(test_image)

    # Use @ operator
    similarity = emb1 @ emb2

    assert isinstance(similarity, float), "@ operator should return float"
    assert 0.0 <= similarity <= 1.0, "Cosine similarity should be in [0, 1]"
    assert similarity > 0.99, "Same image should have similarity near 1.0"


@pytest.mark.heavy
def test_warmup(embedding_model) -> None:  # type: ignore[no-untyped-def]
    """Test that warmup runs without error."""
    # Warmup is already called in fixture, but test it explicitly
    embedding_model.warmup()
    # Just verify no exceptions raised
    assert True


@pytest.mark.heavy
def test_compare_one_to_many(embedding_model, test_image) -> None:  # type: ignore[no-untyped-def]
    """Test GPU-accelerated one-to-many comparison."""
    import torch

    # Create query and gallery
    query_emb = embedding_model.embed(test_image)
    gallery_embs = embedding_model.embed(test_image, test_image, test_image)

    # Compare on GPU
    similarities = embedding_model.compare_one_to_many(query_emb, gallery_embs)

    print(f"\nOne-to-many similarities: {similarities}")

    # Should return torch.Tensor
    assert isinstance(similarities, torch.Tensor), "Should return torch.Tensor"
    assert similarities.shape == (3,), "Should have 3 similarities"
    assert similarities.device.type in ["cuda", "cpu"], "Should be on device"

    # All should be ~1.0 (same image)
    similarities_np = similarities.cpu().numpy()
    assert np.all(similarities_np > 0.99), "Same images should have similarity ~1.0"


@pytest.mark.heavy
def test_compare_many_to_many(embedding_model) -> None:  # type: ignore[no-untyped-def]
    """Test GPU-accelerated many-to-many comparison."""
    import torch

    if not hasattr(embedding_model, "embed_text"):
        pytest.skip("Model does not support text embeddings")

    # Create queries and candidates
    queries = embedding_model.embed_text("a cafe", "a person")
    candidates = embedding_model.embed_text("a cafe", "a restaurant", "a dog")

    # Compare on GPU
    similarities = embedding_model.compare_many_to_many(queries, candidates)

    print(f"\nMany-to-many similarities:\n{similarities}")

    # Should return torch.Tensor
    assert isinstance(similarities, torch.Tensor), "Should return torch.Tensor"
    assert similarities.shape == (2, 3), "Should be (2, 3) similarity matrix"
    assert similarities.device.type in ["cuda", "cpu"], "Should be on device"

    # First query should match first candidate best
    similarities_np = similarities.cpu().numpy()
    assert similarities_np[0, 0] > similarities_np[0, 2], "Cafe should match cafe better than dog"


@pytest.mark.heavy
def test_gpu_query_performance(embedding_model, test_image) -> None:  # type: ignore[no-untyped-def]
    """Test that query method uses GPU acceleration."""
    # Create a larger gallery
    gallery_size = 20
    gallery_images = [test_image] * gallery_size
    gallery_embs = embedding_model.embed(*gallery_images)

    query_emb = embedding_model.embed(test_image)

    # Query should use GPU-accelerated comparison
    results = embedding_model.query(query_emb, gallery_embs, top_k=5)

    print(f"\nTop-5 results from gallery of {gallery_size}")
    for idx, sim in results:
        print(f"  Index {idx}: {sim:.4f}")

    assert len(results) == 5, "Should return top-5 results"
    # All should be high similarity (same image, allow some variation for image preprocessing)
    for idx, sim in results:
        assert sim > 0.90, f"Same images should have high similarity, got {sim}"


@pytest.mark.heavy
def test_embedding_performance(embedding_model) -> None:  # type: ignore[no-untyped-def]
    """Measure embedding performance over multiple real video frames."""
    import time

    from dimos.utils.testing import TimedSensorReplay

    # Load actual video frames
    data_dir = "unitree_go2_lidar_corrected"
    get_data(data_dir)

    video_replay = TimedSensorReplay(f"{data_dir}/video")  # type: ignore[var-annotated]

    # Collect 10 real frames from the video
    test_images = []
    for _ts, frame in video_replay.iterate_ts(duration=1.0):
        test_images.append(frame.to_rgb())
        if len(test_images) >= 10:
            break

    if len(test_images) < 10:
        pytest.skip(f"Not enough video frames found (got {len(test_images)})")

    # Measure single image embedding time
    times = []
    for img in test_images:
        start = time.perf_counter()
        _ = embedding_model.embed(img)
        end = time.perf_counter()
        elapsed_ms = (end - start) * 1000
        times.append(elapsed_ms)

    # Calculate statistics
    avg_time = sum(times) / len(times)
    min_time = min(times)
    max_time = max(times)
    std_time = (sum((t - avg_time) ** 2 for t in times) / len(times)) ** 0.5

    print("\n" + "=" * 60)
    print("Embedding Performance Statistics:")
    print("=" * 60)
    print(f"Number of images: {len(test_images)}")
    print(f"Average time: {avg_time:.2f} ms")
    print(f"Min time: {min_time:.2f} ms")
    print(f"Max time: {max_time:.2f} ms")
    print(f"Std dev: {std_time:.2f} ms")
    print(f"Throughput: {1000 / avg_time:.1f} images/sec")
    print("=" * 60)

    # Also test batch embedding performance
    start = time.perf_counter()
    batch_embeddings = embedding_model.embed(*test_images)
    end = time.perf_counter()
    batch_time = (end - start) * 1000
    batch_per_image = batch_time / len(test_images)

    print("\nBatch Embedding Performance:")
    print(f"Total batch time: {batch_time:.2f} ms")
    print(f"Time per image (batched): {batch_per_image:.2f} ms")
    print(f"Batch throughput: {1000 / batch_per_image:.1f} images/sec")
    print(f"Speedup vs single: {avg_time / batch_per_image:.2f}x")
    print("=" * 60)

    # Verify embeddings are valid
    assert len(batch_embeddings) == len(test_images)
    assert all(e.vector is not None for e in batch_embeddings)

    # Sanity check: verify embeddings are meaningful by testing text-image similarity
    # Skip for models that don't support text embeddings
    if hasattr(embedding_model, "embed_text"):
        print("\n" + "=" * 60)
        print("Sanity Check: Text-Image Similarity on First Frame")
        print("=" * 60)
        first_frame_emb = batch_embeddings[0]

        # Test common object/scene queries
        test_queries = [
            "indoor scene",
            "outdoor scene",
            "a person",
            "a dog",
            "a robot",
            "grass and trees",
            "furniture",
            "a car",
        ]

        text_embeddings = embedding_model.embed_text(*test_queries)
        similarities = []
        for query, text_emb in zip(test_queries, text_embeddings, strict=False):
            sim = first_frame_emb @ text_emb
            similarities.append((query, sim))

        # Sort by similarity
        similarities.sort(key=lambda x: x[1], reverse=True)

        print("Top matching concepts:")
        for query, sim in similarities[:5]:
            print(f"  '{query}': {sim:.4f}")
        print("=" * 60)
