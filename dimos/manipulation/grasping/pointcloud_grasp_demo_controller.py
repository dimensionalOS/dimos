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

from __future__ import annotations

import threading
import time
from typing import Protocol

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.grasping_msgs.TargetBounds import TargetBounds
from dimos.msgs.perception_msgs.RegisteredObject import RegisteredObject
from dimos.perception.object_scene_registration_spec import ObjectSceneRegistrationSpec
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PointcloudGraspingSpec(Spec, Protocol):
    def generate_grasps(
        self,
        object_name: str = "object",
        object_id: str | None = None,
        filter_collisions: bool = True,
    ) -> str: ...


class PointcloudGraspDemoControllerConfig(ModuleConfig):
    target_name: str = "sphere"
    detection_timeout_s: float = 45.0
    retry_interval_s: float = 0.5
    pointcloud_settle_s: float = 1.0
    filter_collisions: bool = True
    workspace_center: tuple[float, float, float] = (0.45, 0.0, 0.18)


class PointcloudGraspDemoController(Module):
    """Trigger pointcloud grasp generation for a registered object without robot execution."""

    config: PointcloudGraspDemoControllerConfig

    grasp_target_bounds: Out[TargetBounds]

    _scene_registration: ObjectSceneRegistrationSpec
    _grasping: PointcloudGraspingSpec

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run_demo,
            name="PointcloudGraspDemoController",
            daemon=True,
        )
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        super().stop()

    def _run_demo(self) -> None:
        target_name = self.config.target_name
        logger.info("Starting pointcloud GPD grasp demo for '%s'", target_name)
        self._scene_registration.set_prompts(text=[target_name])
        target = self._wait_for_target(target_name)
        if target is None:
            logger.warning("No registered object matched target '%s'", target_name)
            self._publish_empty_target_bounds(target_name)
            return

        self._publish_target_bounds(target)
        self._wait(self.config.pointcloud_settle_s)
        if self._stop_event.is_set():
            return
        result = self._grasping.generate_grasps(
            object_name=target.name,
            object_id=target.object_id,
            filter_collisions=self.config.filter_collisions,
        )
        logger.info("Pointcloud GPD grasp demo result: %s", result)

    def _wait_for_target(self, target_name: str) -> RegisteredObject | None:
        deadline = time.time() + self.config.detection_timeout_s
        while time.time() < deadline and not self._stop_event.is_set():
            matches = [
                obj
                for obj in self._scene_registration.get_registered_objects()
                if obj.name.lower() == target_name.lower()
            ]
            if matches:
                return min(matches, key=self._workspace_distance)
            self._wait(self.config.retry_interval_s)
        return None

    def _workspace_distance(self, obj: RegisteredObject) -> float:
        cx, cy, cz = self.config.workspace_center
        center = Vector3(cx, cy, cz)
        return center.distance(obj.center)

    def _wait(self, duration_s: float) -> None:
        self._stop_event.wait(max(0.0, duration_s))

    def _publish_target_bounds(self, target: RegisteredObject) -> None:
        self.grasp_target_bounds.publish(
            TargetBounds(
                center=target.center,
                size=target.size,
                frame_id=target.frame_id,
                ts=target.ts,
                label=f"{target.name}:{target.object_id}",
            )
        )

    def _publish_empty_target_bounds(self, target_name: str) -> None:
        self.grasp_target_bounds.publish(
            TargetBounds(
                center=Vector3(),
                size=Vector3(),
                frame_id="world",
                label=f"no target: {target_name}",
            )
        )
