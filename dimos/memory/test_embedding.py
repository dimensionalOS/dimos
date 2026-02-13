# Copyright 2026 Dimensional Inc.
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

import pytest

from dimos.memory.embedding import EmbeddingMemory, SpatialEntry
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.utils.data import get_data
from dimos.utils.testing import TimedSensorReplay

dir_name = "unitree_go2_bigoffice"


@pytest.mark.skip
def test_embed_frame() -> None:
    """Test embedding a single frame."""
    # Load a frame from recorded data
    video = TimedSensorReplay(get_data(dir_name) / "video")
    frame = video.find_closest_seek(10)

    # Create memory and embed
    memory = EmbeddingMemory()

    try:
        # Create a spatial entry with dummy pose (no TF needed for this test)
        dummy_pose = PoseStamped(
            position=[0, 0, 0],
            orientation=[0, 0, 0, 1],  # identity quaternion
        )
        spatial_entry = SpatialEntry(image=frame, pose=dummy_pose)

        # Embed the frame
        result = memory._embed_spatial_entry(spatial_entry)

        # Verify
        assert result is not None
        assert result.embedding is not None
        assert result.embedding.vector is not None
        print(f"Embedding shape: {result.embedding.vector.shape}")
        print(f"Embedding vector (first 5): {result.embedding.vector[:5]}")
    finally:
        memory.stop()
