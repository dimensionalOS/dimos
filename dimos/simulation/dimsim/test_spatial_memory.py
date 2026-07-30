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

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.visual_memory import VisualMemory
from dimos.simulation.dimsim.spatial_memory import DimSimSpatialMemory


@pytest.fixture
def memory_setup(
    mocker,
    tmp_path,
):
    vector_db = mocker.patch("dimos.perception.spatial_perception.SpatialVectorDB").return_value
    mocker.patch("dimos.perception.spatial_perception.ImageEmbeddingProvider")
    visual_memory = VisualMemory(output_dir=str(tmp_path))
    clear_visual_memory = mocker.spy(visual_memory, "clear")
    memory = DimSimSpatialMemory(
        db_path=None,
        visual_memory=visual_memory,
        visual_memory_path=None,
    )
    try:
        yield memory, vector_db, clear_visual_memory
    finally:
        memory.stop()


def test_clear_eval_memory_removes_observations_tags_and_sampling_state(
    mocker,
    memory_setup,
) -> None:
    memory, vector_db, clear_visual_memory = memory_setup
    vector_db.image_collection.get.return_value = {"ids": ["frame-1", "frame-2"]}
    vector_db.location_collection.get.return_value = {"ids": ["tag-1"]}
    memory.robot_locations.append(mocker.Mock())
    memory.last_position = mocker.Mock()
    memory.last_record_time = 123.0
    memory.frame_count = 9
    memory.stored_frame_count = 4
    memory._latest_video_frame = mocker.Mock()

    assert memory.eval_memory_generation() == 0
    cleared = memory.clear_eval_memory()

    assert cleared == 3
    assert memory.eval_memory_generation() == 1
    vector_db.image_collection.delete.assert_called_once_with(
        ids=["frame-1", "frame-2"],
    )
    vector_db.location_collection.delete.assert_called_once_with(ids=["tag-1"])
    clear_visual_memory.assert_called_once_with()
    assert memory.robot_locations == []
    assert memory._latest_video_frame is None
    assert memory.last_position is None
    assert memory.last_record_time is None
    assert memory.frame_count == 0
    assert memory.stored_frame_count == 0
    assert memory.query_detection_viewpoint("bathtub") is None


def test_clear_eval_memory_advances_generation_even_when_empty(
    memory_setup,
) -> None:
    memory, vector_db, _clear_visual_memory = memory_setup
    vector_db.image_collection.get.return_value = {"ids": []}
    vector_db.location_collection.get.return_value = {"ids": []}

    memory.clear_eval_memory()
    memory.clear_eval_memory()

    assert memory.eval_memory_generation() == 2


def test_detection_viewpoint_uses_latest_matching_frame(memory_setup) -> None:
    memory, _vector_db, _clear_visual_memory = memory_setup
    old_pose = PoseStamped(position=Vector3(1.0, 2.0, 0.5))
    new_pose = PoseStamped(position=Vector3(3.0, 4.0, 0.5))

    memory.record_detection_viewpoint(["Bathtub"], old_pose, 10.0)
    memory.record_detection_viewpoint(["gray bathtub"], new_pose, 20.0)

    assert memory.query_detection_viewpoint("the gray bathtub in the bathroom") is new_pose
    assert memory.query_detection_viewpoint("bathtub") is new_pose
    assert memory.query_detection_viewpoint("couch") is None


def test_clear_eval_memory_forgets_detection_viewpoints(memory_setup) -> None:
    memory, vector_db, _clear_visual_memory = memory_setup
    vector_db.image_collection.get.return_value = {"ids": []}
    vector_db.location_collection.get.return_value = {"ids": []}
    memory.record_detection_viewpoint(
        ["bathtub"],
        PoseStamped(position=Vector3(1.0, 2.0, 0.5)),
        10.0,
    )

    memory.clear_eval_memory()

    assert memory.query_detection_viewpoint("bathtub") is None
