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

"""NavRecord: records all nav stack streams to a memory2 SQLite database."""

from __future__ import annotations

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos_lcm.std_msgs import Bool as LcmBool  # type: ignore[import-untyped]
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Int8 import Int8


class NavRecordConfig(RecorderConfig):
    db_path: str = "nav_recording.db"


class NavRecord(Recorder):
    """Records nav stack outputs to SQLite via memory2.

    All ports are auto-wired by name via autoconnect. Only streams
    that are actually connected will be recorded.
    """

    config: NavRecordConfig

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    # Core nav outputs
    cmd_vel: In[Twist]
    corrected_odometry: In[Odometry]
    path: In[NavPath]
    goal_path: In[NavPath]
    way_point: In[PointStamped]
    goal: In[PointStamped]
    stop_movement: In[LcmBool]

    # LocalPlanner details
    effective_cmd_vel: In[Twist]
    slow_down: In[Int8]
    goal_reached: In[Bool]

    # Point clouds (high bandwidth — recorded if connected)
    terrain_map: In[PointCloud2]
    global_map: In[PointCloud2]

    # Raw inputs
    odometry: In[Odometry]
    registered_scan: In[PointCloud2]
