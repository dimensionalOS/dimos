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

"""Rust multi-level surface path planner."""

from __future__ import annotations

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class MLSPlannerNativeConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/mls_planner"
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True

    world_frame: str = "map"
    voxel_size: float = 0.1
    robot_height: float = 1.5

    surface_dilation_passes: int = 3
    surface_erosion_passes: int = 3
    node_spacing_m: float = 1.0
    node_wall_buffer_m: float = 0.3
    node_step_threshold_m: float = 0.25


class MLSPlannerNative(NativeModule):
    """Rust-backed MLS planner."""

    config: MLSPlannerNativeConfig

    global_map: In[PointCloud2]
    start_pose: In[PoseStamped]
    goal_pose: In[PoseStamped]

    path: Out[Path]
    surface_map: Out[PointCloud2]
    nodes: Out[PointCloud2]
    node_edges: Out[LineSegments3D]
