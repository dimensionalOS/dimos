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

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class TfPoseSourceConfig(ModuleConfig):
    target_frame: str = "world"
    source_frame: str = "base_link"
    tf_tolerance_s: float = 0.1
    publish_rate_hz: float = 10.0


class TfPoseSource(Module):
    """Publishes pose-only odometry from a TF lookup at a fixed rate."""

    config: TfPoseSourceConfig  # type: ignore[assignment]

    odometry: Out[Odometry]
    tf: In[TFMessage]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        if self.pose_source_config.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        if self.pose_source_config.tf_tolerance_s < 0.0:
            raise ValueError("tf_tolerance_s must be non-negative")
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        super().stop()

    def tick(self) -> bool:
        """Publish one odometry sample if the configured TF is available."""

        config = self.pose_source_config
        transform = self.tfbuffer.get(
            config.target_frame,
            config.source_frame,
            time_point=time.time(),
            time_tolerance=config.tf_tolerance_s,
        )
        if transform is None:
            return False

        odometry = Odometry(
            ts=transform.ts,
            frame_id=config.target_frame,
            child_frame_id=config.source_frame,
            pose=Pose((transform.translation, transform.rotation)),
        )
        self.odometry.publish(odometry)
        return True

    def _run_loop(self) -> None:
        period_s = 1.0 / self.pose_source_config.publish_rate_hz
        next_tick = time.monotonic()
        while not self._stop_event.is_set():
            self.tick()
            next_tick += period_s
            sleep_s = max(0.0, next_tick - time.monotonic())
            self._stop_event.wait(sleep_s)

    @property
    def pose_source_config(self) -> TfPoseSourceConfig:
        return self.config  # type: ignore[return-value]


tf_pose_source = TfPoseSource.blueprint
