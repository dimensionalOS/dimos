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

"""DimSim parallel eval blueprint — full temporal memory stack without visualization.

Same as sim_temporal_memory but replaces sim_basic with a headless variant
that skips rerun and websocket_vis. This avoids port conflicts when running
multiple dimos instances in parallel for eval.

Each instance gets isolated ChromaDB/visual-memory paths derived from
EVAL_INSTANCE_ID (defaults to "0").

Usage:
    EVAL_INSTANCE_ID=0 dimos --simulation run sim-parallel-eval
    EVAL_INSTANCE_ID=1 dimos --simulation run sim-parallel-eval
"""

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import JpegLcmTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.perception.experimental.temporal_memory.temporal_memory import TemporalMemory
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.sim.bridge import sim_bridge
from dimos.robot.sim.tf_module import sim_tf
from dimos.constants import DIMOS_PROJECT_ROOT

# Per-instance isolation: separate ChromaDB + visual memory paths
_instance_id = os.environ.get("EVAL_INSTANCE_ID", "0")
_instance_dir = DIMOS_PROJECT_ROOT / "assets" / "output" / "memory" / f"eval_{_instance_id}"
_db_path = str(_instance_dir / "chromadb_data")
_visual_memory_path = str(_instance_dir / "visual_memory.pkl")

# Headless sim_basic: LCM transports + bridge + TF
# No rerun, websocket_vis, web_input, speak_skill, or agent
# (these bind fixed ports that conflict when running multiple instances)
_transports = autoconnect().transports(
    {("color_image", Image): JpegLcmTransport("/color_image", Image)}
)

sim_parallel_eval = autoconnect(
    # sim_basic (headless — no visualization)
    _transports,
    sim_bridge(),
    sim_tf(),
    # sim_nav
    VoxelGridMapper.blueprint(voxel_size=0.1),
    CostMapper.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    WavefrontFrontierExplorer.blueprint(),
    # sim_spatial (isolated DB per instance)
    SpatialMemory.blueprint(
        db_path=_db_path, visual_memory_path=_visual_memory_path, new_memory=True
    ),
    # temporal_memory
    TemporalMemory.blueprint(),
).global_config(n_workers=8, robot_model="dimsim")

__all__ = ["sim_parallel_eval"]
