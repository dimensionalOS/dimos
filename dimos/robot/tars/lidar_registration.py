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

"""Registers sensor-frame lidar clouds into a fixed frame with a TF lookup per scan."""

from __future__ import annotations

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class LidarRegistrationConfig(ModuleConfig):
    target_frame: str = "world"
    # TF lookup is nearest-sample, not interpolated: on a fast-moving mount even a few ms
    # off smears the map. TarsConnection publishes TF at each scan's own timestamp, so keep
    # this tight enough that only that exact sample matches.
    tf_tolerance: float = 0.002
    # how long to wait for a TF sample that arrives after its cloud
    tf_wait: float = 0.2


class LidarRegistration(Module):
    """lidar (any sensor frame) -> registered_lidar (target_frame), via TF at the scan time."""

    config: LidarRegistrationConfig
    lidar: In[PointCloud2]
    tf: In[TFMessage]
    registered_lidar: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.lidar.subscribe(self._on_lidar)))

    def _on_lidar(self, msg: PointCloud2) -> None:
        cfg = self.config
        if msg.frame_id == cfg.target_frame:
            self.registered_lidar.publish(msg)
            return
        tf = self.tfbuffer.get(
            cfg.target_frame,
            msg.frame_id,
            msg.ts,
            cfg.tf_tolerance,
            forward_tolerance=cfg.tf_wait,
        )
        if tf is not None:
            self.registered_lidar.publish(msg.transform(tf))
