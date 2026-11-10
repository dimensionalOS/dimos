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

import cv2
import numpy as np
import pytest
from reactivex import from_iterable, of, operators as ops

from dimos.core.transport import LCMTransport
from dimos.message_utils import image as image_utils
from dimos_messages.builtin_interfaces.msg import Time
from dimos_messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_messages.std_msgs.msg import Header
from dimos_messages.tf2_msgs.msg import TFMessage
from dimos.perception.experimental import spatial_perception
from dimos.perception.experimental.spatial_perception import SpatialMemory
from dimos.perception.experimental.visual_memory import VisualMemory
from dimos.utils.data import get_data


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

    def test_spatial_memory_processing(self, spatial_memory, monkeypatch) -> None:
        """Test processing video frames and building spatial memory with CLIP embeddings."""
        # Exercise storage and retrieval without depending on playback timing.
        monkeypatch.setattr(spatial_memory, "min_time_threshold", 0.0)
        video_path = get_data("assets") / "trimmed_video_office.mov"
        capture = cv2.VideoCapture(str(video_path))
        target_frames = 100

        def frames():
            for index in range(target_frames):
                success, frame = capture.read()
                assert success, f"Could not read frame {index} from {video_path}"
                yield {
                    "frame": frame,
                    "position": Vector3(index * 0.5, index * 0.5, 0),
                    "rotation": Vector3(),
                }

        try:
            assert capture.isOpened(), f"Could not open test video: {video_path}"
            results = (
                spatial_memory.process_stream(from_iterable(frames())).pipe(ops.to_list()).run()
            )
        finally:
            capture.release()

        assert len(results) == target_frames

        for query in ["office", "room with furniture"]:
            results = spatial_memory.query_by_text(query, limit=2)
            assert results, f"No results found for relevant query: {query}"
            assert any(1 - result["distance"] > 0.22 for result in results), (
                f"Expected at least one result with similarity > 0.22 for query '{query}'"
            )

        results = spatial_memory.query_by_text("star wars", limit=2)
        assert all(1 - result["distance"] < 0.25 for result in results), (
            "Expected all results to have similarity < 0.25 for irrelevant query 'star wars'"
        )

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
        wire = LCMTransport("spatial_tf", TFMessage, url="memq://")
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
        image = image_utils.from_array(
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
