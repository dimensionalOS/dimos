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

from dimos.memory.objects import DETECTIONS_STREAM, detections3d_stream
from dimos.perception.detection.type.detection3d.imageDetections3DPC import ImageDetections3DPC


def test_detections3d_stream_materializes_canonical_frames() -> None:
    detections = [
        {"class_name": "chair", "confidence": 0.8, "ts": 1.0, "x": 0.0, "y": 0.0, "z": 0.3},
        {"class_name": "tv", "confidence": 0.9, "ts": 1.0, "x": 5.0, "y": 5.0, "z": 1.0},
        {"class_name": "chair", "confidence": 0.6, "ts": 2.0, "x": 1.0, "y": 0.0, "z": 0.3},
    ]
    stream = detections3d_stream(detections)
    assert stream.name == DETECTIONS_STREAM == "detections3d"

    observations = stream.to_list()
    # one observation per frame ts, every detection kept (no curation)
    assert [o.ts for o in observations] == [1.0, 2.0]
    frame = observations[0].data
    assert isinstance(frame, ImageDetections3DPC)
    assert sorted(det.name for det in frame) == ["chair", "tv"]

    tv = next(det for det in frame if det.name == "tv")
    assert tv.frame_id == "world"
    assert (tv.center.x, tv.center.y, tv.center.z) == (5.0, 5.0, 1.0)
    assert tv.confidence == 0.9
    assert len(tv.pointcloud) == 1

    later = observations[1].data
    assert [det.name for det in later] == ["chair"]
    assert later[0].ts == 2.0
