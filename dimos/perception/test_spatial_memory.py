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

import os
import time
import tempfile
import pytest
import numpy as np
import cv2
import shutil
import reactivex as rx
from reactivex import operators as ops
from reactivex.subject import Subject
from reactivex import Observable

from dimos.perception.spatial_perception import SpatialMemory
from dimos.types.position import Position
from dimos.stream.video_provider import VideoProvider
from dimos.types.position import Position
from dimos.types.vector import Vector


class TestSpatialMemory:
    @pytest.fixture(scope="function")
    def temp_dir(self):
        # Create a temporary directory for storing spatial memory data
        temp_dir = tempfile.mkdtemp()
        yield temp_dir
        # Clean up
        shutil.rmtree(temp_dir)

    def test_spatial_memory_initialization(self):
        """Test SpatialMemory initializes correctly with CLIP model."""
        try:
            # Initialize spatial memory with default CLIP model
            memory = SpatialMemory(
                collection_name="test_collection", embedding_model="clip", new_memory=True
            )
            assert memory is not None
            assert memory.embedding_model == "clip"
            assert memory.embedding_provider is not None
        except Exception as e:
            # If the model doesn't initialize, skip the test
            pytest.fail(f"Failed to initialize model: {e}")

    def test_image_embedding(self):
        """Test generating image embeddings using CLIP."""
        try:
            # Initialize spatial memory with CLIP model
            memory = SpatialMemory(
                collection_name="test_collection", embedding_model="clip", new_memory=True
            )

            # Create a test image - use a simple colored square
            test_image = np.zeros((224, 224, 3), dtype=np.uint8)
            test_image[50:150, 50:150] = [0, 0, 255]  # Blue square

            # Generate embedding
            embedding = memory.embedding_provider.get_embedding(test_image)

            # Check embedding shape and characteristics
            assert embedding is not None
            assert isinstance(embedding, np.ndarray)
            assert embedding.shape[0] == memory.embedding_dimensions

            # Check that embedding is normalized (unit vector)
            assert np.isclose(np.linalg.norm(embedding), 1.0, atol=1e-5)

            # Test text embedding
            text_embedding = memory.embedding_provider.get_text_embedding("a blue square")
            assert text_embedding is not None
            assert isinstance(text_embedding, np.ndarray)
            assert text_embedding.shape[0] == memory.embedding_dimensions
            assert np.isclose(np.linalg.norm(text_embedding), 1.0, atol=1e-5)
        except Exception as e:
            pytest.fail(f"Error in test: {e}")

    def test_spatial_memory_processing(self, temp_dir):
        """Test processing video frames and building spatial memory with CLIP embeddings."""
        try:
            # Initialize spatial memory with temporary storage
            memory = SpatialMemory(
                collection_name="test_collection",
                embedding_model="clip",
                new_memory=True,
                db_path=os.path.join(temp_dir, "chroma_db"),
                visual_memory_path=os.path.join(temp_dir, "visual_memory.pkl"),
                output_dir=os.path.join(temp_dir, "images"),
                min_distance_threshold=0.01,
                min_time_threshold=0.01,
            )

            from dimos.utils.testing import testData

            video_path = testData("assets") / "trimmed_video_office.mov"
            assert os.path.exists(video_path), f"Test video not found: {video_path}"
            video_provider = VideoProvider(dev_name="test_video", video_source=video_path)
            video_stream = video_provider.capture_video_as_observable(realtime=False, fps=15)

            # Create a frame counter for position generation
            frame_counter = 0

            # Process each video frame directly
            def process_frame(frame):
                nonlocal frame_counter

                # Generate a unique position for this frame to ensure minimum distance threshold is met
                pos = Position(frame_counter * 0.5, frame_counter * 0.5, 0)
                transform = {"position": pos, "timestamp": time.time()}
                frame_counter += 1

                # Create a dictionary with frame, position and rotation for SpatialMemory.process_stream
                return {
                    "frame": frame,
                    "position": transform["position"],
                    "rotation": transform["position"],  # Using position as rotation for testing
                }

            # Create a stream that processes each frame
            formatted_stream = video_stream.pipe(ops.map(process_frame))

            # Process the stream using SpatialMemory's built-in processing
            print("Creating spatial memory stream...")
            spatial_stream = memory.process_stream(formatted_stream)

            # Stream is now created above using memory.process_stream()

            # Collect results from the stream
            results = []

            frames_processed = 0
            target_frames = 100  # Process more frames for thorough testing

            def on_next(result):
                nonlocal results, frames_processed
                if not result:  # Skip None results
                    return

                results.append(result)
                frames_processed += 1

                # Stop processing after target frames
                if frames_processed >= target_frames:
                    subscription.dispose()

            def on_error(error):
                pytest.fail(f"Error in spatial stream: {error}")

            def on_completed():
                pass

            # Subscribe and wait for results
            subscription = spatial_stream.subscribe(
                on_next=on_next, on_error=on_error, on_completed=on_completed
            )

            # Wait for frames to be processed
            timeout = 30.0  # seconds
            start_time = time.time()
            while frames_processed < target_frames and time.time() - start_time < timeout:
                time.sleep(0.5)

            subscription.dispose()

            assert len(results) > 0, "Failed to process any frames with spatial memory"

            relevant_queries = ["office", "room with furniture"]
            irrelevant_query = "star wars"

            for query in relevant_queries:
                results = memory.query_by_text(query, limit=2)
                print(f"\nResults for query: '{query}'")

                assert len(results) > 0, f"No results found for relevant query: {query}"

                similarities = [1 - r.get("distance") for r in results]
                print(f"Similarities: {similarities}")

                assert any(d > 0.24 for d in similarities), (
                    f"Expected at least one result with similarity > 0.24 for query '{query}'"
                )

            results = memory.query_by_text(irrelevant_query, limit=2)
            print(f"\nResults for query: '{irrelevant_query}'")

            if results:
                similarities = [1 - r.get("distance") for r in results]
                print(f"Similarities: {similarities}")

                assert all(d < 0.25 for d in similarities), (
                    f"Expected all results to have similarity < 0.25 for irrelevant query '{irrelevant_query}'"
                )

        except Exception as e:
            pytest.fail(f"Error in test: {e}")
        finally:
            memory.cleanup()
            video_provider.dispose_all()


if __name__ == "__main__":
    pytest.main(["-v", __file__])
