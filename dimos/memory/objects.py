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

"""Materialize frozen raw detections as the canonical perception detection stream.

``detections3d_stream`` takes raw per-frame detection dicts (``class_name``,
``confidence``, ``ts``, odom-grounded world ``x``/``y``/``z``) and materializes
the memory convention established by ``dimos/perception/memory/tool_localize.py``:
a stream named ``"detections3d"`` whose payload is
:class:`~dimos.perception.detection.type.detection3d.imageDetections3DPC.ImageDetections3DPC`
— one observation per camera frame, holding one
:class:`~dimos.perception.detection.type.detection3d.pointcloud.Detection3DPC`
per detection.

The source recording has no depth_image/camera_info, so tool_localize's depth
projection cannot run. Positions come from the YOLO+odom grounding instead
(object world position = robot odom position at detection time) and are stored
as a single-point world-frame pointcloud per detection; the frame image is a
stub ``Image`` carrying only the timestamp.
"""

from __future__ import annotations

from collections import defaultdict
from typing import Any

import numpy as np

from dimos.memory.store.memory import MemoryStore
from dimos.memory.stream import Stream
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.type.detection3d.imageDetections3DPC import ImageDetections3DPC
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC

DETECTIONS_STREAM = "detections3d"  # stream name convention from tool_localize.py


def detections3d_stream(detections: list[dict[str, Any]]) -> Stream[ImageDetections3DPC]:
    """Materialize raw detection dicts as the full-span ``detections3d`` stream.

    No curation: every detection of every frame is stored, matching what the
    perception pipeline itself would have written. Summarizing this stream for
    an agent is the encoder's job, not the store's.
    """
    frames: dict[float, list[dict[str, Any]]] = defaultdict(list)
    for det in sorted(detections, key=lambda d: d["ts"]):
        frames[det["ts"]].append(det)

    stream: Stream[ImageDetections3DPC] = MemoryStore().stream(
        DETECTIONS_STREAM, ImageDetections3DPC
    )
    for ts, dets in frames.items():
        image = Image(ts=ts)
        stream.append(
            ImageDetections3DPC(
                image=image,
                detections=[
                    Detection3DPC(
                        bbox=(0.0, 0.0, 0.0, 0.0),
                        track_id=-1,
                        class_id=-1,
                        confidence=d["confidence"],
                        name=d["class_name"],
                        ts=ts,
                        image=image,
                        frame_id="world",
                        pointcloud=PointCloud2.from_numpy(
                            np.array([[d["x"], d["y"], d["z"]]]),
                            frame_id="world",
                            timestamp=ts,
                        ),
                    )
                    for d in dets
                ],
            ),
            ts=ts,
        )
    return stream
