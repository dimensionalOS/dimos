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

"""Privileged scene registration backed by MuJoCo ground truth.

Implements the same ``ObjectSceneRegistrationSpec`` the camera-and-detector
module does, so pick-and-place cannot tell them apart. Prompts resolve to
MuJoCo body names instead of to detections, which takes the detector and the
segmenter out of the loop while the manipulation side is being built.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.experimental.object import Object, to_detection3d_array
from dimos.simulation.perception.sim_scene_spec import SimSceneGeometrySpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SimSceneRegistrationConfig(ModuleConfig):
    target_frame: str = "world"
    # Bodies never offered as objects: the robot and the room it stands in.
    # Substring match, so one entry covers a whole articulated chain.
    ignore_substrings: list[str] = Field(
        default_factory=lambda: ["world", "table", "wall", "floor", "camera", "gate"]
    )
    # Prompt -> body name, for the cases where neither exact nor substring
    # matching gets there ("red cube" -> "cube_red").
    aliases: dict[str, str] = Field(default_factory=dict)
    # Bodies left out of the obstacle cloud entirely. The robot belongs here:
    # a depth camera sees it and the self-filter removes it, but sampling the
    # model would otherwise hand the planner its own arm as an obstacle.
    scene_exclude_substrings: list[str] = Field(default_factory=list)
    object_sample_count: int = 512
    scene_sample_count: int = 20000
    scene_voxel_size: float = 0.01


class SimSceneRegistrationModule(Module):
    """Ground-truth ``ObjectSceneRegistrationSpec`` over a MuJoCo sim."""

    config: SimSceneRegistrationConfig
    _sim: SimSceneGeometrySpec

    detections_3d: Out[Detection3DArray]
    objects: Out[list[Object]]
    pointcloud: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._prompts: list[str] = []
        self._objects: dict[str, Object] = {}
        self._lock = threading.RLock()

    @rpc
    def set_prompts(self, text: list[str] | None = None) -> None:
        with self._lock:
            self._prompts = list(text or [])

    @rpc
    def scan_scene(self, text: list[str] | None = None) -> Detection3DArray:
        """Resolve prompts to MuJoCo bodies and publish them as scene objects."""
        with self._lock:
            if text is not None:
                self._prompts = list(text)
            prompts = list(self._prompts)
            bodies = self._sim.list_body_names()
            matches = self._match_prompts(prompts, bodies)
            ts = time.time()
            objects = [
                obj
                for prompt, body in matches.items()
                if (obj := self._build_object(body, prompt, ts)) is not None
            ]
            self._objects = {obj.object_id: obj for obj in objects}

        self.objects.publish(objects)
        detections = to_detection3d_array(objects, frame_id=self.config.target_frame, ts=ts)
        self.detections_3d.publish(detections)
        logger.info(
            "SimSceneRegistration scan",
            prompts=prompts,
            matched={obj.name: obj.object_id for obj in objects},
        )
        return detections

    @rpc
    def get_object_pointcloud_by_object_id(self, object_id: str) -> PointCloud2 | None:
        with self._lock:
            obj = self._objects.get(object_id)
        return obj.pointcloud if obj is not None else None

    @rpc
    def get_object_pointcloud_by_name(self, name: str) -> PointCloud2 | None:
        with self._lock:
            matches = [obj for obj in self._objects.values() if obj.name == name]
        return matches[0].pointcloud if matches else None

    @rpc
    def get_full_scene_pointcloud(
        self,
        exclude_object_id: str | None = None,
        depth_trunc: float = 2.0,
        voxel_size: float = 0.01,
    ) -> PointCloud2 | None:
        """Everything except one object, as obstacle geometry for planning.

        ``depth_trunc`` has no meaning here: the samples come from the model,
        not from a camera that could be truncated. It stays in the signature
        because the spec is shared with the camera-backed module.
        """
        exclude = [exclude_object_id] if exclude_object_id else []
        if self.config.scene_exclude_substrings:
            exclude += [
                body
                for body in self._sim.list_body_names()
                if any(
                    term.lower() in body.lower() for term in self.config.scene_exclude_substrings
                )
            ]
        points = np.asarray(
            self._sim.sample_scene_surface(
                exclude=exclude,
                voxel_size=voxel_size,
                count=self.config.scene_sample_count,
            ),
            dtype=np.float64,
        )
        if points.size == 0:
            return None
        cloud = PointCloud2.from_numpy(
            points.reshape(-1, 3),
            frame_id=self.config.target_frame,
            timestamp=time.time(),
        )
        self.pointcloud.publish(cloud)
        return cloud

    def _match_prompts(self, prompts: list[str], bodies: list[str]) -> dict[str, str]:
        """Resolve each prompt to at most one body: alias, then exact, then substring."""
        candidates = [body for body in bodies if not self._ignored(body)]
        matches: dict[str, str] = {}
        for prompt in prompts:
            key = prompt.strip().lower()
            if not key:
                continue
            alias = self.config.aliases.get(prompt) or self.config.aliases.get(key)
            if alias and alias in bodies:
                matches[prompt] = alias
                continue
            exact = [body for body in candidates if body.lower() == key]
            if exact:
                matches[prompt] = exact[0]
                continue
            token = key.replace(" ", "_")
            partial = [
                body for body in candidates if token in body.lower() or body.lower() in token
            ]
            if partial:
                matches[prompt] = min(partial, key=len)
        return matches

    def _ignored(self, body: str) -> bool:
        lowered = body.lower()
        return any(term in lowered for term in self.config.ignore_substrings)

    def _build_object(self, body: str, prompt: str, ts: float) -> Object | None:
        points = np.asarray(
            self._sim.sample_body_surface(body, self.config.object_sample_count),
            dtype=np.float64,
        )
        if points.size == 0:
            logger.warning("SimSceneRegistration: body has no sampleable geometry", body=body)
            return None
        points = points.reshape(-1, 3)
        lower, upper = points.min(axis=0), points.max(axis=0)
        center = (lower + upper) * 0.5
        size = upper - lower
        frame = self.config.target_frame
        return Object(
            # object_id is the body name so the agent can round-trip an id
            # from scan_objects straight back into the simulator.
            object_id=body,
            name=prompt,
            center=Vector3(*center),
            size=Vector3(*size),
            pose=PoseStamped(
                ts=ts,
                frame_id=frame,
                position=Vector3(*center),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
            pointcloud=PointCloud2.from_numpy(points, frame_id=frame, timestamp=ts),
            frame_id=frame,
            bbox=(0.0, 0.0, 0.0, 0.0),
            track_id=abs(hash(body)) % 100000,
            class_id=0,
            confidence=1.0,
            ts=ts,
            image=Image(),
        )
