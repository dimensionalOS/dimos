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

from dataclasses import dataclass

from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
from dimos.navigation.nav_3d.mls_planner.mls_planner import MLSPlanner


@dataclass
class EvalConfig:
    """Mapper, planner, and gate parameters.

    Defaults mirror production. Body and capability bounds are sized for the
    Unitree Go2 (0.31m wide, 0.40m tall, ~0.16m stair risers).
    """

    voxel_size: float = 0.08
    max_range: float = 30.0
    ray_subsample: int = 1
    shadow_depth: float = 0.1
    grace_depth: float = 0.2
    min_health: int = -1
    max_health: int = 5
    graze_cos: float = 0.7
    support_min: int = 4

    robot_height: float = 0.3
    max_overhead_m: float = 2.0
    surface_closing_radius: float = 0.4
    node_spacing_m: float = 1.0
    wall_clearance_m: float = 0.1
    wall_buffer_m: float = 0.75
    wall_buffer_weight: float = 100.0
    step_threshold_m: float = 0.16
    step_penalty_weight: float = 4.0

    # Physical body envelope for the collision gate. The gate catches paths
    # that penetrate obstacles, not near-grazes, so the radius is the true
    # body half-width. The ground margin over the radius bounds the terrain
    # slope the gate tolerates. Keep margin/radius above the steepest stairs.
    robot_radius: float = 0.16
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

    def make_mapper(self) -> VoxelRayMapper:
        return VoxelRayMapper(
            voxel_size=self.voxel_size,
            max_range=self.max_range,
            ray_subsample=self.ray_subsample,
            shadow_depth=self.shadow_depth,
            grace_depth=self.grace_depth,
            min_health=self.min_health,
            max_health=self.max_health,
            graze_cos=self.graze_cos,
            support_min=self.support_min,
        )

    def make_planner(self) -> MLSPlanner:
        return MLSPlanner(
            voxel_size=self.voxel_size,
            robot_height=self.robot_height,
            max_overhead_m=self.max_overhead_m,
            surface_closing_radius=self.surface_closing_radius,
            node_spacing_m=self.node_spacing_m,
            wall_clearance_m=self.wall_clearance_m,
            wall_buffer_m=self.wall_buffer_m,
            wall_buffer_weight=self.wall_buffer_weight,
            step_threshold_m=self.step_threshold_m,
            step_penalty_weight=self.step_penalty_weight,
        )

    def mapper_fingerprint(self) -> dict[str, float | int]:
        """The mapper parameters that determine final map content."""
        return {
            "voxel_size": self.voxel_size,
            "max_range": self.max_range,
            "ray_subsample": self.ray_subsample,
            "shadow_depth": self.shadow_depth,
            "grace_depth": self.grace_depth,
            "min_health": self.min_health,
            "max_health": self.max_health,
            "graze_cos": self.graze_cos,
            "support_min": self.support_min,
        }
