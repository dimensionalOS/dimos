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

import pytest


@pytest.mark.skip
def test_to_foxglove_scene_update(detections3dpc) -> None:
    # Convert to scene update
    scene_update = detections3dpc.to_foxglove_scene_update()

    # Verify scene update structure
    assert scene_update is not None
    assert scene_update.deletions_length == 0
    assert len(scene_update.deletions) == 0
    assert scene_update.entities_length == len(detections3dpc.detections)
    assert len(scene_update.entities) == len(detections3dpc.detections)

    # Verify each entity corresponds to a detection
    for _i, (entity, detection) in enumerate(
        zip(scene_update.entities, detections3dpc.detections, strict=False)
    ):
        assert entity.id == str(detection.track_id)
        assert entity.frame_id == detection.frame_id
        assert entity.cubes_length == 1
        assert entity.texts_length == 1
