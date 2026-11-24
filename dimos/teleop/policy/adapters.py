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

"""TF adapter kept outside the velocity blender."""

from pydantic import Field

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class TcpPoseConfig(ModuleConfig):
    world_frame: str = Field(default="world", min_length=1)
    tcp_frame: str = Field(default="link_tcp", min_length=1)


class TcpPoseAdapter(Module):
    """Select a direct world-to-TCP transform, preserving its measurement time.

    Coordinator TF already contains this composed world pose. This adapter does
    not compose arbitrary TF trees or refresh old measurements.
    """

    config: TcpPoseConfig
    tf: In[TFMessage]
    current_pose: Out[Transform]

    async def handle_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if (
                transform.frame_id == self.config.world_frame
                and transform.child_frame_id == self.config.tcp_frame
            ):
                self.current_pose.publish(transform)
