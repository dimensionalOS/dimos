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

"""Ground-truth object scene from simulator state, in place of camera perception.

Implements the same spec and ports as ObjectSceneRegistrationModule, so the
manipulation stack cannot tell the difference, but the detections come from the
simulator's own body poses and the clouds from the objects' meshes. Sim only:
it exists to take perception out of the loop while grasping is under test.
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

import numpy as np
import open3d as o3d
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.experimental.object import (
    Object as DetObject,
    to_detection3d_array,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


class SimObjectSceneConfig(ModuleConfig):
    # MuJoCo body name -> mesh file sampled to produce that object's cloud.
    objects: dict[str, str] = Field(default_factory=dict)
    frame_id: str = "world"
    publish_hz: float = Field(default=2.0, gt=0.0)
    points_per_object: int = Field(default=2000, gt=0)


class SimObjectScene(Module):
    """Publish simulator ground truth through the perception object-scene API."""

    config: SimObjectSceneConfig
    _sim: MujocoSimModule | None = None

    objects: Out[list[DetObject]]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._samples: dict[str, NDArray[np.float64]] = {}
        self._extents: dict[str, NDArray[np.float64]] = {}
        self._clouds: dict[str, PointCloud2] = {}
        self._prompts: tuple[str, ...] = ()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        for name, mesh_path in self.config.objects.items():
            mesh = o3d.io.read_triangle_mesh(str(mesh_path))
            if mesh.is_empty():
                logger.warning(f"SimObjectScene: empty mesh for '{name}' at {mesh_path}")
                continue
            o3d.utility.random.seed(42)
            sampled = mesh.sample_points_uniformly(number_of_points=self.config.points_per_object)
            self._samples[name] = np.asarray(sampled.points, dtype=np.float64)
            bounds = mesh.get_axis_aligned_bounding_box()
            self._extents[name] = np.asarray(bounds.get_extent(), dtype=np.float64)
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        logger.info(f"SimObjectScene started with {len(self._samples)} objects")

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        super().stop()

    def _publish_loop(self) -> None:
        period = 1.0 / self.config.publish_hz
        while not self._stop_event.is_set():
            try:
                detections = self._build_detections()
                if detections:
                    self.objects.publish(detections)
            except Exception:
                logger.warning("SimObjectScene publish failed", exc_info=True)
            self._stop_event.wait(period)

    def _build_detections(self) -> list[DetObject]:
        if self._sim is None or not self._samples:
            return []
        poses = self._sim.get_body_poses(list(self._samples))
        now = time.time()
        detections: list[DetObject] = []
        clouds: dict[str, PointCloud2] = {}
        for name, sample in self._samples.items():
            pose = poses.get(name)
            if pose is None:
                continue
            translation = Vector3(*pose[:3])
            rotation = Quaternion(*pose[3:])
            cloud = PointCloud2.from_numpy(
                sample, frame_id=self.config.frame_id, timestamp=now
            ).transform(
                Transform(
                    translation=translation,
                    rotation=rotation,
                    frame_id=self.config.frame_id,
                    child_frame_id=name,
                    ts=now,
                )
            )
            cloud.ts = now
            clouds[name] = cloud
            if not self._matches(name):
                continue
            extent = self._extents[name]
            detections.append(
                DetObject(  # type: ignore[abstract]
                    name=name,
                    object_id=name,
                    center=Vector3(pose[0], pose[1], pose[2] + extent[2] / 2.0),
                    size=Vector3(*extent),
                    pose=PoseStamped(),
                    pointcloud=cloud,
                    frame_id=self.config.frame_id,
                    bbox=(0.0, 0.0, 1.0, 1.0),
                    track_id=0,
                    class_id=0,
                    confidence=1.0,
                    ts=now,
                    image=Image(),
                )
            )
        with self._lock:
            self._clouds = clouds
        return detections

    def _matches(self, name: str) -> bool:
        lowered = name.lower()
        return not self._prompts or any(p in lowered or lowered in p for p in self._prompts)

    @rpc
    def set_prompts(self, text: list[str] | None = None) -> None:
        """Restrict published detections to objects whose name matches a prompt.

        Ground truth has no detector to prompt, so the prompts filter by name
        substring instead. Scene and per-object clouds stay complete, matching a
        real detector whose prompts never affect the raw depth cloud.
        """
        self._prompts = tuple(prompt.lower() for prompt in text or ())

    @rpc
    def scan_scene(self) -> Detection3DArray:
        """Publish and return one ground-truth detection pass."""
        detections = self._build_detections()
        if detections:
            self.objects.publish(detections)
        return to_detection3d_array(
            detections,
            frame_id=self.config.frame_id,
            ts=detections[0].ts if detections else time.time(),
        )

    def _stamped(self, cloud: PointCloud2 | None) -> PointCloud2 | None:
        # Callers reject clouds older than a few seconds; the geometry is exact
        # at any age, so re-stamp on read rather than force a faster loop.
        if cloud is not None:
            cloud.ts = time.time()
        return cloud

    @rpc
    def get_object_pointcloud_by_name(self, name: str) -> PointCloud2 | None:
        with self._lock:
            return self._stamped(self._clouds.get(name))

    @rpc
    def get_object_pointcloud_by_object_id(self, object_id: str) -> PointCloud2 | None:
        with self._lock:
            return self._stamped(self._clouds.get(object_id))

    @rpc
    def get_full_scene_pointcloud(
        self,
        exclude_object_id: str | None = None,
        depth_trunc: float = 2.0,
        voxel_size: float = 0.01,
    ) -> PointCloud2 | None:
        with self._lock:
            clouds = [c for name, c in self._clouds.items() if name != exclude_object_id]
        if not clouds:
            return None
        merged = clouds[0]
        for cloud in clouds[1:]:
            merged = merged + cloud
        merged = merged.voxel_downsample(voxel_size)
        merged.ts = time.time()
        return merged
