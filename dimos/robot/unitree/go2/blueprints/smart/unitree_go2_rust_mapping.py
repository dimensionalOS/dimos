#!/usr/bin/env python3
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

"""unitree-go2 with the mapping stack swapped for the Rust native modules
(issue #2820): RustVoxelGridMapper + RustCostMapper in place of the Python
VoxelGridMapper + CostMapper. Same ports, topics, and tuning, so the rest of
the graph (planner, explorer, patrolling, movement) is wired identically.

Build the binaries first::

    cd dimos/mapping/rust && cargo build --release

Then::

    uv run dimos --replay run unitree-go2-rust-mapping
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.rust_mappers import RustCostMapper, RustVoxelGridMapper
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.patrolling.module import PatrollingModule
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic

unitree_go2_rust_mapping = autoconnect(
    unitree_go2_basic,
    RustVoxelGridMapper.blueprint(emit_every=5),
    RustCostMapper.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    WavefrontFrontierExplorer.blueprint(),
    PatrollingModule.blueprint(),
    MovementManager.blueprint(),
).global_config(n_workers=10, robot_model="unitree_go2_rust_mapping")
