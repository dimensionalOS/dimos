# Copyright 2025-2026 Dimensional Inc.
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

"""A broad Recorder that captures the full Go2 sim stream to one SQLite file.

Opt-in per run by appending the module name, e.g.::

    dimos --simulation mujoco run unitree-go2-agentic-gemini go2-full-recorder

Every declared ``In`` port is subscribed and persisted with the robot pose +
timestamp; the resulting DB is replayable (``dimos --replay --replay-db ...``)
and readable via ``SqliteStore(db).stream(name).iterate()``.
"""

from pathlib import Path

from langchain_core.messages.base import BaseMessage

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos_lcm.std_msgs.Bool import Bool


class Go2FullRecorderConfig(RecorderConfig):
    db_path: str | Path = "assets/output/captures/go2_full.db"


class Go2FullRecorder(Recorder):
    """Records the full robot story: vision, 3D/maps, nav state, voice + agent."""

    # Vision
    color_image: In[Image]
    camera_info: In[CameraInfo]
    # 3D / maps
    lidar: In[PointCloud2]
    global_map: In[PointCloud2]
    global_costmap: In[OccupancyGrid]
    # State / navigation
    odom: In[PoseStamped]
    path: In[NavPath]
    goal: In[PointStamped]
    way_point: In[PointStamped]
    cmd_vel: In[Twist]
    goal_reached: In[Bool]
    # Voice + agent
    human_input: In[str]
    agent: In[BaseMessage]

    config: Go2FullRecorderConfig

    @rpc
    def start(self) -> None:
        # super().start() opens the SQLite store at db_path, which fails if the
        # parent directory doesn't exist yet — create it first.
        Path(self.config.db_path).parent.mkdir(parents=True, exist_ok=True)
        super().start()
