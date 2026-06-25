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

"""Records the Go2 + Mid-360 rig into a memory2 SQLite db.

Captures Point-LIO odom + lidar (trajectory baked into ``pointlio_lidar`` via the
inherited ``@pose_setter_for``) plus the Go2's companion streams. The raw Livox
stream is NOT recorded here — enable the pcap recorder in the record blueprint to
capture it. Companion streams are recorded as-is and anchored via the static mount
frames published on tf.
"""

from __future__ import annotations

from dimos.core.stream import In
from dimos.hardware.sensors.lidar.pointlio.pose_recorder import PointlioPoseRecorder
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class Go2Mid360Recorder(PointlioPoseRecorder):
    go2_lidar: In[PointCloud2]
    go2_odom: In[PoseStamped]
    color_image: In[Image]
