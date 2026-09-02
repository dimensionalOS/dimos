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

from dataclasses import dataclass, field, fields
import math


@dataclass
class EvalConfig:
    """Harness parameters, sized for the Unitree Go2."""

    # Passed to the pipeline under test, which owns the map it plans on.
    voxel_size: float = 0.08
    max_range: float = 30.0
    robot_height: float = 0.3

    # Collision-gate body box. Only the ground_margin to body_clearance band
    # is checked, so the legs and the terrain under them never count.
    robot_length: float = 0.7
    robot_width: float = 0.31
    ground_margin: float = 0.25
    body_clearance: float = 0.45
    # Ground-support reach. The radius models straddling small scan holes.
    support_radius_m: float = 0.35
    support_depth_m: float = 0.35

    goal_tolerance: float = 0.5
    align_tol: float = 0.05
    # How near the walked path an endpoint must be to count as visited, which
    # is what makes a demonstrated reference length available.
    visit_radius_m: float = 1.0
    # Climb limits, from the steepest climbs the Go2 demonstrated on stairs.
    max_slope: float = 1.2
    max_step_m: float = 0.2
    kinematic_window_m: float = 0.5

    # Which pipeline is under test, by registry name.
    pipeline: str = "mls"
    # Pipeline constructor overrides, e.g. --set planner.wall_clearance_m=0.0.
    planner: dict[str, float | int] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        """Called again after --set, which mutates an already-built config."""
        # nan compares false against every bound below, so it would pass each
        # check and then quietly fail whole cases downstream.
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f"{f.name} must be finite, got {value}")
        # An inverted band makes check_path admit nothing and pass every path,
        # which reads as a perfect score rather than a failure.
        if self.body_clearance <= self.ground_margin:
            raise ValueError(
                f"body_clearance ({self.body_clearance}) must exceed "
                f"ground_margin ({self.ground_margin})"
            )
        for name in (
            "voxel_size",
            "max_range",
            "robot_height",
            "robot_length",
            "robot_width",
            "ground_margin",
            "body_clearance",
            "support_radius_m",
            "support_depth_m",
            "goal_tolerance",
            "align_tol",
            "visit_radius_m",
            "max_slope",
            "max_step_m",
            "kinematic_window_m",
        ):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive, got {getattr(self, name)}")
