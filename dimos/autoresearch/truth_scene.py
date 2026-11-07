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

"""Scene scans answered from the simulator's true body poses instead of a
detector, in the shape of the object-registration module's RPCs."""

from __future__ import annotations

import threading
from typing import Any

from dimos_lcm.geometry_msgs import Pose
from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
from pydantic import BaseModel
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray


class TruthObject(BaseModel):
    """One tracked object's box: ``size`` is its extents and ``centre`` where the
    box centre sits relative to the body origin, both in metres."""

    size: tuple[float, float, float]
    centre: tuple[float, float, float] = (0.0, 0.0, 0.0)


class TruthSceneModuleConfig(ModuleConfig):
    """``objects`` maps each tracked body name to its box."""

    objects: dict[str, TruthObject]


class TruthSceneModule(Module):
    """Answers ``scan_scene`` with a world-frame box for every tracked body named
    in the query, from the latest ``body_poses`` message. Takes the place of
    the object-registration module on a simulated stack without a detector."""

    config: TruthSceneModuleConfig

    body_poses: In[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._latest: dict[str, Transform] = {}
        self._prompts: list[str] | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.body_poses.subscribe(self._on_body_poses)))

    def _on_body_poses(self, msg: TFMessage) -> None:
        with self._lock:
            for transform in msg.transforms:
                self._latest[transform.child_frame_id] = transform

    @rpc
    def set_prompts(self, text: list[str] | None = None) -> None:
        """Remember the words later scans without a query use."""
        self._prompts = text

    @rpc
    def scan_scene(self, text: list[str] | None = None) -> Detection3DArray:
        """Boxes in the world frame for the tracked bodies ``text`` names (a body
        matches a query that contains its name as a word); every body when no words."""
        words = self._prompts if text is None else text
        with self._lock:
            latest = dict(self._latest)
        detections = [
            _detection(name, transform, self.config.objects[name])
            for name, transform in latest.items()
            if name in self.config.objects and _named(name, words)
        ]
        ts = max((transform.ts for transform in latest.values()), default=0.0)
        return Detection3DArray(
            detections_length=len(detections),
            header=Header(ts, "world"),
            detections=detections,
        )

    @rpc
    def get_object_pointcloud_by_name(self, name: str) -> PointCloud2 | None:
        """None: the truth scene has no camera, so no point clouds."""
        return None

    @rpc
    def get_object_pointcloud_by_object_id(self, object_id: str) -> PointCloud2 | None:
        """None: the truth scene has no camera, so no point clouds."""
        return None

    @rpc
    def get_full_scene_pointcloud(
        self,
        exclude_object_id: str | None = None,
        depth_trunc: float = 2.0,
        voxel_size: float = 0.01,
    ) -> PointCloud2 | None:
        """None: the truth scene has no camera, so no point clouds."""
        return None


def _named(body: str, words: list[str] | None) -> bool:
    return words is None or any(body in word.lower().split() for word in words)


def _detection(name: str, transform: Transform, box: TruthObject) -> Detection3D:
    t, q = transform.translation, transform.rotation
    detection = Detection3D()
    detection.header = Header(transform.ts, "world")
    detection.id = name
    detection.results = [
        ObjectHypothesisWithPose(hypothesis=ObjectHypothesis(class_id=name, score=1.0))
    ]
    detection.results_length = 1
    detection.bbox = BoundingBox3D(
        center=Pose(
            position=Vector3(t.x + box.centre[0], t.y + box.centre[1], t.z + box.centre[2]),
            orientation=Quaternion(q.x, q.y, q.z, q.w),
        ),
        size=Vector3(*box.size),
    )
    return detection
