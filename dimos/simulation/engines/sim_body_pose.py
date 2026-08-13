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

"""Publish one simulated body's world pose in place of perception.

Sim only. It exists so a skill that consumes an object pose can be developed
and measured without perception in the loop; the consumer sees an ordinary
world-frame PoseStamped and cannot tell where it came from.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SimBodyPoseConfig(ModuleConfig):
    # MuJoCo body whose world pose is published.
    body_name: str = ""
    frame_id: str = "world"
    publish_hz: float = Field(default=5.0, gt=0.0)
    missing_body_grace_seconds: float = Field(default=5.0, ge=0.0)


class SimBodyPose(Module):
    """Ground-truth pose of one simulator body, published as a PoseStamped."""

    config: SimBodyPoseConfig
    _sim: MujocoSimModule | None = None

    object_pose: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest: PoseStamped | None = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._missing_logged = False
        self._started_at = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        if not self.config.body_name:
            logger.warning("SimBodyPose has no body_name; nothing will be published")
            return
        self._stop_event.clear()
        self._started_at = time.monotonic()
        self._thread = threading.Thread(target=self._publish_loop, name="SimBodyPose", daemon=True)
        self._thread.start()
        logger.info(f"SimBodyPose publishing '{self.config.body_name}'")

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        super().stop()

    @rpc
    def get_pose(self) -> PoseStamped | None:
        """Latest published pose, for callers that pull instead of subscribing."""
        return self._latest

    def _publish_loop(self) -> None:
        period = 1.0 / self.config.publish_hz
        while not self._stop_event.is_set():
            try:
                pose = self._read_pose()
                if pose is not None:
                    self._latest = pose
                    self.object_pose.publish(pose)
            except Exception:
                logger.warning("SimBodyPose publish failed", exc_info=True)
            self._stop_event.wait(period)

    def _read_pose(self) -> PoseStamped | None:
        if self._sim is None:
            return None
        name = self.config.body_name
        poses = self._sim.get_body_poses([name])
        values = poses.get(name)
        if values is None:
            grace_elapsed = time.monotonic() - self._started_at
            if not self._missing_logged and grace_elapsed >= self.config.missing_body_grace_seconds:
                logger.error(f"SimBodyPose: no body named '{name}' in the loaded scene")
                self._missing_logged = True
            return None
        self._missing_logged = False
        return PoseStamped(
            frame_id=self.config.frame_id,
            position=list(values[:3]),
            orientation=list(values[3:]),
            ts=time.time(),
        )
