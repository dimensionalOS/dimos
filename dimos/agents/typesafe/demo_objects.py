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
"""Publish fixed world-frame objects as Detection3DArray: a stand-in for a 3D detector.

``scene_json`` replaces ``objects`` with a ground-truth snapshot in the
``detection3d_array_to_dict`` layout: ``{"detections": [{"label", "center_xyz", "size_xyz"}]}``.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
import threading
import time
from typing import Any

from dimos_lcm.vision_msgs import (
    BoundingBox3D,
    Detection3D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)
from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray


class DemoObjectsConfig(ModuleConfig):
    # (label, x, y, z) world frame. DimSim apartment spawn is (3, 2) facing east; y=2 is clear for x in 1..5.
    objects: list[tuple[str, float, float, float]] = Field(
        default_factory=lambda: [("chair", 1.2, 2.0, 0.4)]
    )
    scene_json: Path | None = None
    exclude: str = ""  # labels matching this regex are not published
    size: tuple[float, float, float] = (0.5, 0.5, 0.9)  # for ``objects``, which carry none
    rate_hz: float = 2.0


Object = tuple[str, tuple[float, float, float], tuple[float, float, float]]  # label, center, size


def load_scene_objects(path: Path, exclude: str = "") -> list[Object]:
    raw = json.loads(Path(path).expanduser().read_text())
    skip = re.compile(exclude) if exclude else None
    return [
        (str(d["label"]), tuple(map(float, d["center_xyz"])), tuple(map(float, d["size_xyz"])))  # type: ignore[misc]
        for d in raw["detections"]
        if skip is None or not skip.search(str(d["label"]))
    ]


class DemoObjects(Module):
    config: DemoObjectsConfig
    detections_3d: Out[Detection3DArray]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._objects: list[Object] = []

    @rpc
    def start(self) -> None:
        super().start()
        self._objects = (
            load_scene_objects(self.config.scene_json, self.config.exclude)
            if self.config.scene_json is not None
            else [(label, (x, y, z), self.config.size) for label, x, y, z in self.config.objects]
        )
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._publish_loop, name="DemoObjects", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        super().stop()

    def _message(self) -> Detection3DArray:
        now = time.time()
        dets = []
        for label, center, size in self._objects:
            d = Detection3D()
            d.header = Header(now, "world")
            d.results = [
                ObjectHypothesisWithPose(hypothesis=ObjectHypothesis(class_id=label, score=0.95))
            ]
            d.results_length = 1
            d.bbox = BoundingBox3D(center=Pose(position=center), size=Vector3(*size))
            dets.append(d)
        return Detection3DArray(
            detections_length=len(dets), header=Header(now, "world"), detections=dets
        )

    def _publish_loop(self) -> None:
        while not self._stop_event.is_set():
            self.detections_3d.publish(self._message())
            self._stop_event.wait(1.0 / self.config.rate_hz)
