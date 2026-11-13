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

"""Publish simulation body poses on TF independently of robot model serving."""

from typing import Any

from dimos_lcm.sim_msgs import WorldState
from pydantic import Field

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.geometry.poses import poses_by_id


class SimTfConfig(ModuleConfig):
    hz: float = Field(default=30.0, gt=0, allow_inf_nan=False)


class SimTfModule(Module):
    config: SimTfConfig
    observations: In[WorldState]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._clock: str | None = None
        self._epoch = -1
        self._next_tf_ns = 0

    async def handle_observations(self, sample: WorldState) -> None:
        """Use measured world poses, including floating objects and fixed sensor mounts."""
        h = sample.header
        if self._clock is not None and self._clock != h.clock_id:
            raise ValueError("simulation TF clock changed; restart its binding")
        if h.epoch < self._epoch:
            return
        if h.epoch == self._epoch and h.time_ns < self._next_tf_ns:
            return
        poses = poses_by_id(sample)
        edges = [
            Transform(
                translation=Vector3(pose.position.x, pose.position.y, pose.position.z),
                rotation=Quaternion(
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
                ),
                frame_id=h.frame_id,
                child_frame_id=name,
                ts=h.time_ns / 1e9,
            )
            for name, pose in poses.items()
            if name != h.frame_id
        ]
        self.tfbuffer.publish(*edges)
        self._clock, self._epoch = h.clock_id, h.epoch
        period = max(1, round(1e9 / self.config.hz))
        self._next_tf_ns = (h.time_ns // period + 1) * period
