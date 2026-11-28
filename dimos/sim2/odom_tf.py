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

"""A robot's odometry on tf, which its HardwareModule may not publish: odometry is an estimate."""

from dimos.core.stream import In
from dimos.core.tf_module import TfModule, TfModuleConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry


class OdomTfConfig(TfModuleConfig):
    """Its frames must name the roles ``odom`` (the frame odometry is in) and ``body`` (what it tracks)."""

    tf_edges: list[tuple[str, str]] = [("odom", "body")]


class OdomTf(TfModule):
    """Puts each odometry reading on tf as odom -> body, stamped as the reading is."""

    config: OdomTfConfig
    odom: In[Odometry]

    async def handle_odom(self, odom: Odometry) -> None:
        self.tfbuffer.publish(Transform.from_pose(self.frame.body, odom.to_pose_stamped()))
