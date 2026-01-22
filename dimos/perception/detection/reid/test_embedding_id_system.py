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

import numpy as np
import pytest

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.reid.embedding_id_system import EmbeddingIDSystem
from dimos.utils.data import get_data


@pytest.fixture(scope="session")
def mobileclip_model():
    """Load MobileCLIP model once for all tests."""
    from dimos.models.embedding.mobileclip import MobileCLIPModel

    model = MobileCLIPModel()  # Uses default MobileCLIP2-S4
    model.start()
    return model


@pytest.fixture
def track_associator(mobileclip_model):
    """Create fresh EmbeddingIDSystem for each test."""
    return EmbeddingIDSystem(
        model=lambda: mobileclip_model,
        similarity_threshold=0.75,
        min_embeddings_for_matching=1,  # Allow matching with single embedding for tests
    )


@pytest.fixture(scope="session")
def test_image():
    """Load test image."""
    return Image.from_file(get_data("cafe.jpg")).to_rgb()


@pytest.mark.gpu
def test_update_embedding_single(track_associator, mobileclip_model, test_image) -> None:
    """Test updating embedding for a single track."""
    embedding = mobileclip_model.embed(test_image)

    # First update
    track_associator.update_embedding(track_id=1, new_embedding=embedding)

    assert 1 in track_associator.track_embeddings
    assert len(track_associator.track_embeddings[1]) == 1

    # Verify embedding is stored as numpy array and normalized
    emb_vec = track_associator.track_embeddings[1][0]
    assert isinstance(emb_vec, np.ndarray)
    norm = np.linalg.norm(emb_vec)
    assert abs(norm - 1.0) < 0.01, "Embedding should be normalized"


@pytest.mark.gpu
def test_update_embedding_multiple(track_associator, mobileclip_model, test_image) -> None:
    """Test storing multiple embeddings per track."""
    embedding1 = mobileclip_model.embed(test_image)
    embedding2 = mobileclip_model.embed(test_image)

    # Add first embedding
    track_associator.update_embedding(track_id=1, new_embedding=embedding1)
    first_vec = track_associator.track_embeddings[1][0].copy()

    # Add second embedding (same image, should be very similar)
    track_associator.update_embedding(track_id=1, new_embedding=embedding2)

    # Should have 2 embeddings now
    assert len(track_associator.track_embeddings[1]) == 2

    # Both should be normalized
    for emb in track_associator.track_embeddings[1]:
        norm = np.linalg.norm(emb)
        assert abs(norm - 1.0) < 0.01, "Embedding should be normalized"

    # Second embedding should be similar to first (same image)
    second_vec = track_associator.track_embeddings[1][1]
    similarity = float(np.dot(first_vec, second_vec))
    assert similarity > 0.99, "Same image should produce very similar embeddings"


@pytest.mark.gpu
def test_negative_constraints(track_associator) -> None:
    """Test negative constraint recording."""
    # Simulate frame with 3 tracks
    track_ids = [1, 2, 3]
    track_associator.add_negative_constraints(track_ids)

    # Check that all pairs are recorded
    assert 2 in track_associator.negative_pairs[1]
    assert 3 in track_associator.negative_pairs[1]
    assert 1 in track_associator.negative_pairs[2]
    assert 3 in track_associator.negative_pairs[2]
    assert 1 in track_associator.negative_pairs[3]
    assert 2 in track_associator.negative_pairs[3]


@pytest.mark.gpu
def test_associate_new_track(track_associator, mobileclip_model, test_image) -> None:
    """Test associating a new track creates new long_term_id."""
    embedding = mobileclip_model.embed(test_image)
    track_associator.update_embedding(track_id=1, new_embedding=embedding)

    # First association should create new long_term_id
    long_term_id = track_associator.associate(track_id=1)

    assert long_term_id == 0, "First track should get long_term_id=0"
    assert track_associator.track_to_long_term[1] == 0
    assert track_associator.long_term_counter == 1


@pytest.mark.gpu
def test_associate_similar_tracks(track_associator, mobileclip_model, test_image) -> None:
    """Test associating similar tracks to same long_term_id."""
    # Create embeddings from same image (should be very similar)
    embedding1 = mobileclip_model.embed(test_image)
    embedding2 = mobileclip_model.embed(test_image)

    # Add first track
    track_associator.update_embedding(track_id=1, new_embedding=embedding1)
    long_term_id_1 = track_associator.associate(track_id=1)

    # Add second track with similar embedding
    track_associator.update_embedding(track_id=2, new_embedding=embedding2)
    long_term_id_2 = track_associator.associate(track_id=2)

    # Should get same long_term_id (similarity > 0.75)
    assert long_term_id_1 == long_term_id_2, "Similar tracks should get same long_term_id"
    assert track_associator.long_term_counter == 1, "Only one long_term_id should be created"


@pytest.mark.gpu
def test_associate_with_negative_constraint(track_associator, mobileclip_model, test_image) -> None:
    """Test that negative constraints prevent association."""
    # Create similar embeddings
    embedding1 = mobileclip_model.embed(test_image)
    embedding2 = mobileclip_model.embed(test_image)

    # Add first track
    track_associator.update_embedding(track_id=1, new_embedding=embedding1)
    long_term_id_1 = track_associator.associate(track_id=1)

    # Add negative constraint (tracks co-occurred)
    track_associator.add_negative_constraints([1, 2])

    # Add second track with similar embedding
    track_associator.update_embedding(track_id=2, new_embedding=embedding2)
    long_term_id_2 = track_associator.associate(track_id=2)

    # Should get different long_term_ids despite high similarity
    assert long_term_id_1 != long_term_id_2, (
        "Co-occurring tracks should get different long_term_ids"
    )
    assert track_associator.long_term_counter == 2, "Two long_term_ids should be created"


@pytest.mark.gpu
def test_associate_different_objects(track_associator, mobileclip_model, test_image) -> None:
    """Test that dissimilar embeddings get different long_term_ids."""
    # Create embeddings for image and text (very different)
    image_emb = mobileclip_model.embed(test_image)
    text_emb = mobileclip_model.embed_text("a dog")

    # Add first track (image)
    track_associator.update_embedding(track_id=1, new_embedding=image_emb)
    long_term_id_1 = track_associator.associate(track_id=1)

    # Add second track (text - very different embedding)
    track_associator.update_embedding(track_id=2, new_embedding=text_emb)
    long_term_id_2 = track_associator.associate(track_id=2)

    # Should get different long_term_ids (similarity < 0.75)
    assert long_term_id_1 != long_term_id_2, "Different objects should get different long_term_ids"
    assert track_associator.long_term_counter == 2


@pytest.mark.gpu
def test_associate_returns_cached(track_associator, mobileclip_model, test_image) -> None:
    """Test that repeated calls return same long_term_id."""
    embedding = mobileclip_model.embed(test_image)
    track_associator.update_embedding(track_id=1, new_embedding=embedding)

    # First call
    long_term_id_1 = track_associator.associate(track_id=1)

    # Second call should return cached result
    long_term_id_2 = track_associator.associate(track_id=1)

    assert long_term_id_1 == long_term_id_2
    assert track_associator.long_term_counter == 1, "Should not create new ID"


@pytest.mark.gpu
def test_associate_no_embedding(track_associator) -> None:
    """Test that associate creates new ID for track without embedding."""
    # Track with no embedding gets assigned a new ID
    long_term_id = track_associator.associate(track_id=999)
    assert long_term_id == 0, "Track without embedding should get new long_term_id"
    assert track_associator.long_term_counter == 1


@pytest.mark.gpu
def test_embeddings_stored_as_numpy(track_associator, mobileclip_model, test_image) -> None:
    """Test that embeddings are stored as numpy arrays for efficient CPU comparisons."""
    embedding = mobileclip_model.embed(test_image)
    track_associator.update_embedding(track_id=1, new_embedding=embedding)

    # Embeddings should be stored as numpy arrays
    emb_list = track_associator.track_embeddings[1]
    assert isinstance(emb_list, list)
    assert len(emb_list) == 1
    assert isinstance(emb_list[0], np.ndarray)

    # Add more embeddings
    embedding2 = mobileclip_model.embed(test_image)
    track_associator.update_embedding(track_id=1, new_embedding=embedding2)

    assert len(track_associator.track_embeddings[1]) == 2
    for emb in track_associator.track_embeddings[1]:
        assert isinstance(emb, np.ndarray)


@pytest.mark.gpu
def test_similarity_threshold_configurable(mobileclip_model) -> None:
    """Test that similarity threshold is configurable."""
    associator_strict = EmbeddingIDSystem(model=lambda: mobileclip_model, similarity_threshold=0.95)
    associator_loose = EmbeddingIDSystem(model=lambda: mobileclip_model, similarity_threshold=0.50)

    assert associator_strict.similarity_threshold == 0.95
    assert associator_loose.similarity_threshold == 0.50


@pytest.mark.gpu
def test_multi_track_scenario(track_associator, mobileclip_model, test_image) -> None:
    """Test realistic scenario with multiple tracks across frames."""
    # Frame 1: Track 1 appears
    emb1 = mobileclip_model.embed(test_image)
    track_associator.update_embedding(1, emb1)
    track_associator.add_negative_constraints([1])
    lt1 = track_associator.associate(1)

    # Frame 2: Track 1 and Track 2 appear (different objects)
    text_emb = mobileclip_model.embed_text("a dog")
    track_associator.update_embedding(1, emb1)  # Update embedding
    track_associator.update_embedding(2, text_emb)
    track_associator.add_negative_constraints([1, 2])  # Co-occur = different
    lt2 = track_associator.associate(2)

    # Track 2 should get different ID due to negative constraint
    assert lt1 != lt2

    # Frame 3: Track 1 disappears, Track 3 appears (same as Track 1)
    emb3 = mobileclip_model.embed(test_image)
    track_associator.update_embedding(3, emb3)
    track_associator.add_negative_constraints([2, 3])
    lt3 = track_associator.associate(3)

    # Track 3 should match Track 1 (not co-occurring, similar embedding)
    assert lt3 == lt1

    print("\nMulti-track scenario results:")
    print(f"  Track 1 -> long_term_id {lt1}")
    print(f"  Track 2 -> long_term_id {lt2} (different object, co-occurred)")
    print(f"  Track 3 -> long_term_id {lt3} (re-identified as Track 1)")
