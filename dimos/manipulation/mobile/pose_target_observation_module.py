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

"""Adapt a pose provider to the typed mobile-manipulation target contract."""

from __future__ import annotations

import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.manipulation.mobile.target_observation import (
    TargetObservation,
    TargetObservationSource,
    copy_pose_stamped,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


class PoseTargetObservationConfig(ModuleConfig):
    object_id: str
    label: str = "object"
    source: TargetObservationSource


class PoseTargetObservationModule(Module):
    """Attach target identity and provenance to a world-frame pose stream."""

    config: PoseTargetObservationConfig

    object_pose: In[PoseStamped]
    target_observation: Out[TargetObservation]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest: TargetObservation | None = None

    @rpc
    def start(self) -> None:
        super().start()
        unsubscribe = self.object_pose.subscribe(self._on_pose)
        self.register_disposable(Disposable(unsubscribe) if callable(unsubscribe) else unsubscribe)

    @rpc
    def get_latest(self) -> TargetObservation | None:
        """Return the last observation produced by this adapter."""
        return self._latest

    def _on_pose(self, pose: PoseStamped) -> None:
        copied = copy_pose_stamped(pose)
        observed_at = float(copied.ts) if copied.ts > 0.0 else time.time()
        observation = TargetObservation(
            object_id=self.config.object_id,
            label=self.config.label,
            pose=copied,
            source=self.config.source,
            observed_at=observed_at,
        )
        self._latest = observation
        self.target_observation.publish(observation)
