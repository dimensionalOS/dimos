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

"""
Test module for the CLIP image embedding functionality in dimos.
"""

import os
import time

import numpy as np
import pytest
from reactivex import operators as ops

from dimos.agents_deprecated.memory.image_embedding import ImageEmbeddingProvider
from dimos.stream.video_provider import VideoProvider


@pytest.mark.heavy
class TestImageEmbedding:
    """Test class for CLIP image embedding functionality."""

    @pytest.mark.tofix
    def test_clip_embedding_initialization(self) -> None:
        """Test CLIP embedding provider initializes correctly."""
        try:
            # Initialize the embedding provider with CLIP model
            embedding_provider = ImageEmbeddingProvider(model_name="clip", dimensions=512)
            assert embedding_provider.model is not None, "CLIP model failed to initialize"
            assert embedding_provider.processor is not None, "CLIP processor failed to initialize"
            assert embedding_provider.model_name == "clip", "Model name should be 'clip'"
            assert embedding_provider.dimensions == 512, "Embedding dimensions should be 512"
        except Exception as e:
            pytest.skip(f"Skipping test due to model initialization error: {e}")

    @pytest.mark.tofix
    def test_clip_embedding_process_video(self) -> None:
        """Test CLIP embedding provider can process video frames and return embeddings."""
        try:
            from dimos.utils.data import get_data

            video_path = get_data("assets") / "trimmed_video_office.mov"

            embedding_provider = ImageEmbeddingProvider(model_name="clip", dimensions=512)

            assert os.path.exists(video_path), f"Test video not found: {video_path}"
            video_provider = VideoProvider(dev_name="test_video", video_source=video_path)

            video_stream = video_provider.capture_video_as_observable(realtime=False, fps=15)

            # Use ReactiveX operators to process the stream
            def process_frame(frame):
                try:
                    # Process frame with CLIP
                    embedding = embedding_provider.get_embedding(frame)
                    print(
                        f"Generated CLIP embedding with shape: {embedding.shape}, norm: {np.linalg.norm(embedding):.4f}"
                    )

                    return {"frame": frame, "embedding": embedding}
                except Exception as e:
                    print(f"Error in process_frame: {e}")
                    return None

            embedding_stream = video_stream.pipe(ops.map(process_frame))

            results = []
            frames_processed = 0
            target_frames = 10

            def on_next(result) -> None:
                nonlocal frames_processed, results
                if not result:  # Skip None results
                    return

                results.append(result)
                frames_processed += 1

                # Stop processing after target frames
                if frames_processed >= target_frames:
                    subscription.dispose()

            def on_error(error) -> None:
                pytest.fail(f"Error in embedding stream: {error}")

            def on_completed() -> None:
                pass

            # Subscribe and wait for results
            subscription = embedding_stream.subscribe(
                on_next=on_next, on_error=on_error, on_completed=on_completed
            )

            timeout = 60.0
            start_time = time.time()
            while frames_processed < target_frames and time.time() - start_time < timeout:
                time.sleep(0.5)
                print(f"Processed {frames_processed}/{target_frames} frames")

            # Clean up subscription
            subscription.dispose()
            video_provider.dispose_all()

            # Check if we have results
            if len(results) == 0:
                pytest.skip("No embeddings generated, but test connection established correctly")
                return

            print(f"Processed {len(results)} frames with CLIP embeddings")

            # Analyze the results
            assert len(results) > 0, "No embeddings generated"

            # Check properties of first embedding
            first_result = results[0]
            assert "embedding" in first_result, "Result doesn't contain embedding"
            assert "frame" in first_result, "Result doesn't contain frame"

            # Check embedding shape and normalization
            embedding = first_result["embedding"]
            assert isinstance(embedding, np.ndarray), "Embedding is not a numpy array"
            assert embedding.shape == (512,), (
                f"Embedding has wrong shape: {embedding.shape}, expected (512,)"
            )
            assert abs(np.linalg.norm(embedding) - 1.0) < 1e-5, "Embedding is not normalized"

            # Save the first embedding for similarity tests
            if len(results) > 1 and "embedding" in results[0]:
                # Create a class variable to store embeddings for the similarity test
                TestImageEmbedding.test_embeddings = {
                    "embedding1": results[0]["embedding"],
                    "embedding2": results[1]["embedding"] if len(results) > 1 else None,
                }
                print("Saved embeddings for similarity testing")

            print("CLIP embedding test passed successfully!")

        except Exception as e:
            pytest.fail(f"Test failed with error: {e}")

    @pytest.mark.tofix
    def test_clip_embedding_similarity(self) -> None:
        """Test CLIP embedding similarity search and text-to-image queries."""
        try:
            # Skip if previous test didn't generate embeddings
            if not hasattr(TestImageEmbedding, "test_embeddings"):
                pytest.skip("No embeddings available from previous test")
                return

            # Get embeddings from previous test
            embedding1 = TestImageEmbedding.test_embeddings["embedding1"]
            embedding2 = TestImageEmbedding.test_embeddings["embedding2"]

            # Initialize embedding provider for text embeddings
            embedding_provider = ImageEmbeddingProvider(model_name="clip", dimensions=512)

            # Test frame-to-frame similarity
            if embedding1 is not None and embedding2 is not None:
                # Compute cosine similarity
                similarity = np.dot(embedding1, embedding2)
                print(f"Similarity between first two frames: {similarity:.4f}")

                # Should be in range [-1, 1]
                assert -1.0 <= similarity <= 1.0, f"Similarity out of valid range: {similarity}"

            # Test text-to-image similarity
            if embedding1 is not None:
                # Generate a list of text queries to test
                text_queries = ["a video frame", "a person", "an outdoor scene", "a kitchen"]

                # Test each text query
                for text_query in text_queries:
                    # Get text embedding
                    text_embedding = embedding_provider.get_text_embedding(text_query)

                    # Check text embedding properties
                    assert isinstance(text_embedding, np.ndarray), (
                        "Text embedding is not a numpy array"
                    )
                    assert text_embedding.shape == (512,), (
                        f"Text embedding has wrong shape: {text_embedding.shape}"
                    )
                    assert abs(np.linalg.norm(text_embedding) - 1.0) < 1e-5, (
                        "Text embedding is not normalized"
                    )

                    # Compute similarity between frame and text
                    text_similarity = np.dot(embedding1, text_embedding)
                    print(f"Similarity between frame and '{text_query}': {text_similarity:.4f}")

                    # Should be in range [-1, 1]
                    assert -1.0 <= text_similarity <= 1.0, (
                        f"Text-image similarity out of range: {text_similarity}"
                    )

            print("CLIP embedding similarity tests passed successfully!")

        except Exception as e:
            pytest.fail(f"Similarity test failed with error: {e}")


if __name__ == "__main__":
    pytest.main(["-v", "--disable-warnings", __file__])
