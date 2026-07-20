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

from __future__ import annotations

from dataclasses import dataclass, field

from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
from dimos.navigation.nav_3d.mls_planner.mls_planner import MLSPlanner


@dataclass
class EvalConfig:
    """Harness and gate parameters, sized for the Unitree Go2.

    (0.31m wide, 0.40m tall, ~0.16m stair risers.)

    Fixes only the shared voxel resolution, sensor range, sensor height, and
    the physical body and capability bounds it gates against. Algorithm tuning
    lives in the algorithm packages as their constructor defaults.
    """

    voxel_size: float = 0.08
    max_range: float = 30.0
    robot_height: float = 0.3

    # Physical body envelope for the collision gate: a box the robot's length
    # and width, oriented along the path and pitched with the slope. The gate
    # catches paths that drive the body through obstacles. Only the elevated
    # body is checked, from ground_margin to body_clearance up the tilted body
    # axis, so the legs and the terrain they stand on never count. Length and
    # width match the Go2 collision box.
    robot_length: float = 0.7
    robot_width: float = 0.31
    ground_margin: float = 0.25
    body_clearance: float = 0.45
    goal_tolerance: float = 0.5
    align_tol: float = 0.05
    # Paths must stand on final-map occupancy within support_radius_m of
    # each sample and support_depth_m below it. The radius models the Go2
    # straddling small scan holes (0.7m footprint), not its body width.
    support_radius_m: float = 0.35
    support_depth_m: float = 0.35
    # Climb limits, checked over a stride-scale window so planner cell
    # quantization does not read as a cliff. The slope bound comes from the
    # steepest climbs the Go2 demonstrated on the Athens stairs, where
    # switchback corners locally exceed the spec-sheet 40 degrees.
    max_slope: float = 1.2
    max_step_m: float = 0.2
    kinematic_window_m: float = 0.5
    # How far an endpoint may sit from a standable surface before it counts as
    # off the map, for both case generation and curation snapping.
    snap_max_m: float = 1.0

    # An improvement must not buy score with compute. p95 over the suite.
    plan_p95_budget_ms: float = 50.0
    map_update_p95_budget_ms: float = 1000.0

    # Planner constructor overrides, e.g. --set planner.wall_clearance_m=0.0.
    # Omitted keys keep the planner's own defaults, so nothing is duplicated
    # here, and the report records whatever was swept.
    planner: dict[str, float] = field(default_factory=dict)

    def make_mapper(self) -> VoxelRayMapper:
        return VoxelRayMapper(voxel_size=self.voxel_size, max_range=self.max_range)

    def make_planner(self) -> MLSPlanner:
        return MLSPlanner(
            voxel_size=self.voxel_size, robot_height=self.robot_height, **self.planner
        )

    def mapper_fingerprint(self) -> dict[str, float | int]:
        """Cache key parameters for the final map.

        Mapper internals are deliberately not fingerprinted. Changes to the
        mapper, code or defaults, require wiping data/.final instead.
        """
        return {"voxel_size": self.voxel_size, "max_range": self.max_range}
