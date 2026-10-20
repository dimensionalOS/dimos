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

from contextlib import ExitStack
import os
import shutil
import tempfile
import time

import numpy as np
import pytest
from reactivex import of, operators as ops
from reactivex.scheduler import ThreadPoolScheduler

from dimos.core.cdr_transport import CdrTransport
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.msgs.image import image_from_array
from dimos.perception.experimental import spatial_perception
from dimos.perception.experimental.spatial_perception import SpatialMemory
from dimos.perception.experimental.visual_memory import VisualMemory
from dimos.stream.video_provider import VideoProvider


@pytest.mark.self_hosted
class TestSpatialMemory:
    @pytest.fixture(scope="class")
    def temp_dir(self):
        # Create a temporary directory for storing spatial memory data
        temp_dir = tempfile.mkdtemp()
        yield temp_dir
        # Clean up
        shutil.rmtree(temp_dir)

    @pytest.fixture(scope="class")
    def spatial_memory(self, temp_dir):
        # Create a single SpatialMemory instance to be reused across all tests
        memory = SpatialMemory(
            collection_name="test_collection",
            new_memory=True,
            db_path=os.path.join(temp_dir, "chroma_db"),
            visual_memory_path=os.path.join(temp_dir, "visual_memory.pkl"),
            output_dir=os.path.join(temp_dir, "images"),
            min_distance_threshold=0.01,
            min_time_threshold=0.01,
        )
        yield memory
        # Clean up
        memory.stop()

    def test_spatial_memory_initialization(self, spatial_memory) -> None:
        """Test SpatialMemory initializes correctly with CLIP model."""
        # Use the shared spatial_memory fixture
        assert spatial_memory is not None
        assert spatial_memory.embedding_provider is not None

    def test_image_embedding(self, spatial_memory) -> None:
        """Test generating image embeddings using CLIP."""
        # Use the shared spatial_memory fixture
        # Create a test image - use a simple colored square
        test_image = np.zeros((224, 224, 3), dtype=np.uint8)
        test_image[50:150, 50:150] = [0, 0, 255]  # Blue square

        # Generate embedding
        embedding = spatial_memory.embedding_provider.get_embedding(test_image)

        # Check embedding shape and characteristics
        assert embedding is not None
        assert isinstance(embedding, np.ndarray)
        assert embedding.shape[0] == 512

        # Check that embedding is normalized (unit vector)
        assert np.isclose(np.linalg.norm(embedding), 1.0, atol=1e-5)

        # Test text embedding
        text_embedding = spatial_memory.embedding_provider.get_text_embedding("a blue square")
        assert text_embedding is not None
        assert isinstance(text_embedding, np.ndarray)
        assert text_embedding.shape[0] == 512
        assert np.isclose(np.linalg.norm(text_embedding), 1.0, atol=1e-5)

    def test_spatial_memory_processing(self, spatial_memory, temp_dir) -> None:
        """Test processing video frames and building spatial memory with CLIP embeddings."""
        test_scheduler = ThreadPoolScheduler(max_workers=4)
        try:
            # Use the shared spatial_memory fixture
            memory = spatial_memory

            from dimos.utils.data import get_data

            video_path = get_data("assets") / "trimmed_video_office.mov"
            assert os.path.exists(video_path), f"Test video not found: {video_path}"
            video_provider = VideoProvider(
                dev_name="test_video", video_source=video_path, pool_scheduler=test_scheduler
            )
            video_stream = video_provider.capture_video_as_observable(realtime=False, fps=15)

            # Create a frame counter for position generation
            frame_counter = 0

            # Process each video frame directly
            def process_frame(frame):
                nonlocal frame_counter

                # Generate a unique position for this frame to ensure minimum distance threshold is met
                pos = Vector3(x=frame_counter * 0.5, y=frame_counter * 0.5, z=0.0)
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

            def on_next(result) -> None:
                nonlocal results, frames_processed
                if not result:  # Skip None results
                    return

                results.append(result)
                frames_processed += 1

                # Stop processing after target frames
                if frames_processed >= target_frames:
                    subscription.dispose()

            def on_error(error) -> None:
                pytest.fail(f"Error in spatial stream: {error}")

            def on_completed() -> None:
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

                assert any(d > 0.22 for d in similarities), (
                    f"Expected at least one result with similarity > 0.22 for query '{query}'"
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
            video_provider.dispose_all()
            test_scheduler.executor.shutdown(wait=True)

    def test_processing_skips_failed_embedding(self, spatial_memory, monkeypatch) -> None:
        """A failed embedding must not end the stream: later frames are still stored."""
        real_get_embedding = spatial_memory.embedding_provider.get_embedding
        calls = 0

        def flaky_get_embedding(image):
            nonlocal calls
            calls += 1
            if calls == 1:
                raise RuntimeError("inference failed")
            return real_get_embedding(image)

        monkeypatch.setattr(spatial_memory.embedding_provider, "get_embedding", flaky_get_embedding)
        frame = np.zeros((224, 224, 3), dtype=np.uint8)
        stored_before = spatial_memory.stored_frame_count
        results = []
        errors = []

        spatial_memory.process_stream(
            of(
                {
                    "frame": frame,
                    "position": Vector3(x=1000.0, y=0.0, z=0.0),
                    "rotation": Vector3(x=0.0, y=0.0, z=0.0),
                },
                {
                    "frame": frame,
                    "position": Vector3(x=1001.0, y=0.0, z=0.0),
                    "rotation": Vector3(x=0.0, y=0.0, z=0.0),
                },
            )
        ).subscribe(on_next=results.append, on_error=errors.append)

        assert errors == []
        assert [r["position"] for r in results] == [(1001, 0, 0)]
        assert spatial_memory.stored_frame_count == stored_before + 1


def test_generated_image_and_tf_reach_spatial_backend_with_correct_colors_and_pose(
    tmp_path, mocker
):
    embedding = mocker.Mock()
    embedding.get_embedding.return_value = np.ones(512, dtype=np.float32) / np.sqrt(512)
    database = mocker.Mock()
    mocker.patch.object(spatial_perception, "ImageEmbeddingProvider", return_value=embedding)
    mocker.patch.object(spatial_perception, "SpatialVectorDB", return_value=database)
    with ExitStack() as cleanup:
        wire = CdrTransport("spatial_tf", TFMessage, backend="lcm", url="memq://")
        cleanup.callback(wire.stop)
        memory = SpatialMemory(
            chroma_client=mocker.Mock(),
            db_path=None,
            visual_memory_path=None,
            visual_memory=VisualMemory(output_dir=str(tmp_path)),
            min_time_threshold=0,
        )
        cleanup.callback(memory.stop)
        memory.tf.transport = wire
        memory.tfbuffer.receive_transform(
            TransformStamped(
                header=Header(frame_id="world", stamp=Time(sec=1, nanosec=0)),
                child_frame_id="base_link",
                transform=Transform(
                    translation=Vector3(x=1, y=2, z=3), rotation=Quaternion(x=0, y=0, z=0, w=1)
                ),
            )
        )
        pixels = np.zeros((4, 4, 3), dtype=np.uint8)
        pixels[:, :, 0] = 255
        image = image_from_array(
            pixels, encoding="rgb8", header=Header(frame_id="camera", stamp=Time(sec=0, nanosec=1))
        )
        memory._receive_frame(image)
        memory._process_frame()
        database.add_image_vector.assert_called_once()
        stored = database.add_image_vector.call_args.kwargs
        np.testing.assert_array_equal(stored["image"][0, 0], [0, 0, 255])
        assert [stored["metadata"][key] for key in ["pos_x", "pos_y", "pos_z"]] == [1, 2, 3]
        assert [stored["metadata"][key] for key in ["rot_x", "rot_y", "rot_z"]] == [0, 0, 0]
        assert memory.stored_frame_count == 1
