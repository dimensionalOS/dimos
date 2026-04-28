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

"""G1 GR00T sim — full perceptive stack: nav + memory + telemetry.

Composes the base sim (``unitree_g1_groot_wbc_sim``) with the dimos
nav stack and a memory2 recorder.  Mirrors the Go2 ``unitree_go2`` →
``unitree_go2_memory`` pattern: base locomotion underneath, mapping +
planning + recording on top, RerunBridge for telemetry.

Layers (bottom-up):
    unitree_g1_groot_wbc_sim   GR00T policy + viser + splat camera + lidar
    VoxelGridMapper            lidar pointcloud → voxel grid → global map
    CostMapper                 global map → occupancy grid for planning
    ReplanningAStarPlanner     odom + costmap + goal → cmd_vel
    G1Memory (Recorder)        records color_image + lidar to SQLite
    RerunBridgeModule          auto-logs everything with .to_rerun()

Usage:
    dimos run unitree-g1-groot-wbc-sim-full
"""

from __future__ import annotations

from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In
from dimos.core.transport import LCMTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc_sim import (
    unitree_g1_groot_wbc_sim,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule, _resolve_viewer_mode


class G1MemoryConfig(RecorderConfig):
    db_path: str | Path = "recording_g1.db"


class G1Memory(Recorder):
    """G1 ``Recorder`` subclass — records the visual + spatial streams.

    Mirrors ``Go2Memory`` shape so memory2's existing playback / search
    tooling works on G1 recordings without special-casing.
    """

    color_image: In[Image]
    lidar: In[PointCloud2]
    config: G1MemoryConfig


# Base sim already publishes pointcloud → /lidar topic and color_image
# → /splat/color_image, so downstream subscribers just need to bind to
# those topics by name.
unitree_g1_groot_wbc_sim_full = autoconnect(
    unitree_g1_groot_wbc_sim,
    VoxelGridMapper.blueprint().transports(
        {
            ("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2),
        }
    ),
    CostMapper.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    G1Memory.blueprint().transports(
        {
            ("color_image", Image): LCMTransport("/splat/color_image", Image),
            ("lidar", PointCloud2): LCMTransport("/lidar", PointCloud2),
        }
    ),
    RerunBridgeModule.blueprint(viewer_mode=_resolve_viewer_mode()),
).global_config(n_workers=10)


__all__ = ["unitree_g1_groot_wbc_sim_full"]
