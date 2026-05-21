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

from dimos_lcm.std_msgs import Bool as LcmBool  # type: ignore[import-untyped]

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.ContourPolygons3D import ContourPolygons3D
from dimos.msgs.nav_msgs.Graph3D import Graph3D
from dimos.msgs.nav_msgs.GraphDelta3D import GraphDelta3D
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Int8 import Int8


class NavRecordConfig(RecorderConfig):
    db_path: str = "nav_recording.db"
    # Robot body frame, for unstamped messages.
    default_frame_id: str = "current_point"
    # Generous so PGO iSAM2 stalls (~500ms) don't cause lookup misses.
    tf_tolerance: float = 3.0


class NavRecord(Recorder):
    """Records nav stack outputs to SQLite via memory2 (only connected streams are recorded)."""

    config: NavRecordConfig

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    # MovementManager outputs (muxed nav + teleop)
    cmd_vel: In[Twist]
    goal: In[PointStamped]
    stop_movement: In[LcmBool]

    # PathFollower output (raw nav cmd before MovementManager mux; remapped from "cmd_vel")
    nav_cmd_vel: In[Twist]

    # LocalPlanner outputs
    path: In[NavPath]
    effective_cmd_vel: In[Twist]
    free_paths: In[PointCloud2]
    slow_down: In[Int8]
    goal_reached: In[Bool]

    # SimplePlanner / FarPlanner / TarePlanner outputs
    way_point: In[PointStamped]
    goal_path: In[NavPath]
    costmap_cloud: In[PointCloud2]  # SimplePlanner only
    # FarPlanner-specific
    graph: In[Graph3D]
    contour_polygons: In[ContourPolygons3D]
    nav_boundary: In[LineSegments3D]

    # TerrainAnalysis / TerrainMapExt outputs
    terrain_map: In[PointCloud2]
    terrain_map_ext: In[PointCloud2]

    # PGO outputs
    corrected_odometry: In[Odometry]
    global_map: In[PointCloud2]
    pose_graph: In[Graph3D]
    loop_closure_event: In[GraphDelta3D]

    # FastLio2 outputs (SLAM source; blueprints typically remap FastLio2's
    # "lidar" -> "registered_scan" and "global_map" -> "global_map_fastlio")
    odometry: In[Odometry]
    registered_scan: In[PointCloud2]
    global_map_fastlio: In[PointCloud2]

    # External inputs to the nav stack (recorded for context)
    clicked_point: In[PointStamped]  # from rerun click-to-drive
    tele_cmd_vel: In[Twist]  # from keyboard / quest / phone teleop
