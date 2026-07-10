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

"""Relay an external SLAM point cloud map into the DimOS graph."""

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class SlamMapRelay(Module):
    """Republish an externally produced SLAM map as DimOS `global_map`."""

    ros_laser_cloud_map: In[PointCloud2]
    global_map: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(self.ros_laser_cloud_map.subscribe(self.global_map.publish))

    @rpc
    def stop(self) -> None:
        super().stop()
