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

import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class StartRelayConfig(ModuleConfig):
    world_frame: str = "odom"
    base_frame: str = "base_link"


class StartRelay(Module):
    """Publish the tf base pose as the PoseStamped topic legacy consumers expect."""

    # While the tf chain is incomplete, retry the lookup at most this often.
    # The buffer warns on every miss, so per-message retries would flood the log.
    RETRY_PERIOD_S = 1.0

    config: StartRelayConfig

    tf: In[TFMessage]

    # TODO: Remove start pose once all planners and controllers use tfs
    start_pose: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._next_lookup = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        # Build the buffer first so it subscribes ahead of _on_tf.
        self.tfbuffer  # noqa: B018
        self.register_disposable(Disposable(self.tf.subscribe(self._on_tf)))

    def _on_tf(self, msg: TFMessage) -> None:
        if time.monotonic() < self._next_lookup:
            return
        pose = self.tfbuffer.get_pose(self.config.world_frame, self.config.base_frame)
        if pose is None:
            self._next_lookup = time.monotonic() + self.RETRY_PERIOD_S
            return
        self.start_pose.publish(pose)
