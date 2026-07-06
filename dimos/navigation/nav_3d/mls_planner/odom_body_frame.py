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

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.nav_msgs.Odometry import Odometry


class OdomBodyFrameConfig(ModuleConfig):
    # base_link from sensor mount rotation, xyzw.
    mount_rotation: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    body_frame_id: str = "base_link"


class OdomBodyFrame(Module):
    """Re-express tilted-sensor LIO odometry in the level robot body frame.

    Composes out the fixed mount rotation from the orientation. Position and
    twist pass through.
    """

    config: OdomBodyFrameConfig

    odometry: In[Odometry]
    body_odometry: Out[Odometry]

    @rpc
    def start(self) -> None:
        super().start()
        self._mount_inv = Quaternion(*self.config.mount_rotation).inverse()
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))

    def _on_odometry(self, msg: Odometry) -> None:
        leveled = msg.orientation * self._mount_inv
        self.body_odometry.publish(
            Odometry(
                ts=msg.ts,
                frame_id=msg.frame_id,
                child_frame_id=self.config.body_frame_id,
                pose=Pose(msg.position, leveled),
                twist=msg.twist,
            )
        )
