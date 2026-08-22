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

"""Record the zed_mid360 rig (ZED eyes + Point-LIO odom/lidar) into a memory2 db.

Extends :class:`~dimos.hardware.sensors.camera.zed.sdkless_recorder.ZedRecorder`
(stereo h264 + camera_info + batched ~800 Hz IMU) with PointLio's ``odometry`` /
``lidar``. All In port names match the producers' Out names, so autoconnect
wires everything with no remappings. Point-LIO publishes the moving
``world -> lidar_link`` edge onto tf and the rig's static urdf frames tie the
cameras into that tree, so every stream lands world-anchored.
"""

from __future__ import annotations

from dimos.core.stream import In
from dimos.hardware.sensors.camera.zed.sdkless_recorder import ZedRecorder
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class ZedMid360Recorder(ZedRecorder):
    lidar: In[PointCloud2]
    odometry: In[Odometry]
