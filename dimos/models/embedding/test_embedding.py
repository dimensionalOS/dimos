import time
from typing import Any

import pytest
import torch

from dimos.models.embedding.clip import CLIPModel
from dimos.models.embedding.mobileclip import MobileCLIPModel
from dimos.models.embedding.treid import TorchReIDModel
from dimos.msgs.sensor_msgs import Image
from dimos.utils.data import get_data


@pytest.mark.parametrize(
    "model_class,model_name,supports_text",
    [
        (CLIPModel, "CLIP", True),
        pytest.param(MobileCLIPModel, "MobileCLIP", True),
        (TorchReIDModel, "TorchReID", False),
    ],
    ids=["clip", "mobileclip", "treid"],
)
@pytest.mark.gpu
def test_embedding_model(model_class: type, model_name: str, supports_text: bool) -> None:
    """Test embedding functionality across different model types."""
    image = Image.from_file(get_data("cafe.jpg")).to_rgb()

    print(f"\nTesting {model_name} embedding model")

    # Initialize model
    print(f"Loading {model_name} model...")
    model: Any = model_class()
    model.start()

    # Test single image embedding
    print("Embedding single image...")
    start_time = time.time()
    embedding = model.embed(image)
    embed_time = time.time() - start_time

    print(f"  Vector shape: {embedding.vector.shape}")
    print(f"  Time: {embed_time:.3f}s")

    assert embedding.vector is not None
    assert len(embedding.vector.shape) == 1  # Should be 1D vector

    # Test batch embedding
    print("\nTesting batch embedding (3 images)...")
    start_time = time.time()
    embeddings = model.embed(image, image, image)
    batch_time = time.time() - start_time

    print(f"  Batch size: {len(embeddings)}")
    print(f"  Total time: {batch_time:.3f}s")
    print(f"  Per image: {batch_time / 3:.3f}s")

    assert len(embeddings) == 3
    assert all(e.vector is not None for e in embeddings)

    # Test similarity computation
    print("\nTesting similarity computation...")
    sim = embedding @ embeddings[0]
    print(f"  Self-similarity: {sim:.4f}")
    # Self-similarity should be ~1.0 for normalized embeddings
    assert sim > 0.99, "Self-similarity should be ~1.0 for normalized embeddings"

    # Test text embedding if supported
    if supports_text:
        print("\nTesting text embedding...")
        start_time = time.time()
        text_embedding = model.embed_text("a photo of a cafe")
        text_time = time.time() - start_time

        print(f"  Text vector shape: {text_embedding.vector.shape}")
        print(f"  Time: {text_time:.3f}s")

        # Test cross-modal similarity
        cross_sim = embedding @ text_embedding
        print(f"  Image-text similarity: {cross_sim:.4f}")

        assert text_embedding.vector is not None
        assert embedding.vector.shape == text_embedding.vector.shape
    else:
        print(f"\nSkipping text embedding (not supported by {model_name})")

    print(f"\n{model_name} embedding test passed!")


@pytest.mark.parametrize(
    "model_class,model_name",
    [
        (CLIPModel, "CLIP"),
        pytest.param(MobileCLIPModel, "MobileCLIP"),
    ],
    ids=["clip", "mobileclip"],
)
@pytest.mark.gpu
def test_text_image_retrieval(model_class: type, model_name: str) -> None:
    """Test text-to-image retrieval using embedding similarity."""
    image = Image.from_file(get_data("cafe.jpg")).to_rgb()

    print(f"\nTesting {model_name} text-image retrieval")

    model: Any = model_class(normalize=True)
    model.start()

    # Embed images
    image_embeddings = model.embed(image, image, image)

    # Embed text queries
    queries = ["a cafe", "a dog", "a car"]
    text_embeddings = model.embed_text(*queries)

    # Compute similarities
    print("\nSimilarity matrix (text x image):")
    for query, text_emb in zip(queries, text_embeddings, strict=False):
        sims = [text_emb @ img_emb for img_emb in image_embeddings]
        print(f"  '{query}': {[f'{s:.3f}' for s in sims]}")

    # The cafe query should have highest similarity
    cafe_sims = [text_embeddings[0] @ img_emb for img_emb in image_embeddings]
    other_sims = [text_embeddings[1] @ img_emb for img_emb in image_embeddings]

    assert cafe_sims[0] > other_sims[0], "Cafe query should match cafe image better than dog query"

    print(f"\n{model_name} retrieval test passed!")


@pytest.mark.gpu
def test_embedding_device_transfer() -> None:
    """Test embedding device transfer operations."""
    image = Image.from_file(get_data("cafe.jpg")).to_rgb()

    model = CLIPModel()
    embedding = model.embed(image)
    assert not isinstance(embedding, list)

    # Test to_numpy
    np_vec = embedding.to_numpy()
    assert not isinstance(np_vec, torch.Tensor)
    print(f"NumPy vector shape: {np_vec.shape}")

    # Test to_torch
    torch_vec = embedding.to_torch()
    assert isinstance(torch_vec, torch.Tensor)
    print(f"Torch vector shape: {torch_vec.shape}, device: {torch_vec.device}")

    # Test to_cpu
    embedding.to_cpu()
    assert isinstance(embedding.vector, torch.Tensor)
    assert embedding.vector.device == torch.device("cpu")
    print("Successfully moved to CPU")
