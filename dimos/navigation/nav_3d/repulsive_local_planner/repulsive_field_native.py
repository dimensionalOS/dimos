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

"""Rust repulsive-field local planner (native module).

GENUINE high-rate solves: the Python module (kept as the reference
implementation in ``local_planner.py``) re-anchored a cached plan at 60 Hz but
re-SOLVED at only ~2-4 Hz, and grew stability machinery to survive that
latency. The Rust port solves fresh every tick and owns its costmap internally
(consumes ``terrain_map`` directly — no CostMapper module needed) at higher
resolution. Config field names mirror the Python configs; the measured
rationale for each value lives in the reference implementation's comments.
"""

from __future__ import annotations

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class RepulsiveFieldNativeConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "result/bin/repulsive_field"
    build_command: str | None = "nix build path:."
    stdin_config: bool = True

    world_frame: str = "map"
    body_frame: str = "base_link"
    # The CMU pure-pursuit follower consumes a vehicle-frame route.
    output_base_frame: bool = True
    solve_hz: float = 60.0
    # Halt publishes when the newest terrain slice is older than this: hl62's
    # terrain input silently died (fragmented multi-MB LCM messages lost under
    # recv starvation) and the robot steered off its frozen costmap's edge,
    # parking 1.84 m short of wp3 for 13 min with zero warnings. Parked-with-
    # errors beats silently-wrong.
    max_costmap_age_s: float = 10.0
    # Publish the INTERNAL costmap's lethal cells for the viewer overlay (the
    # legacy CostMapper overlay showed a map this planner never used).
    costmap_cloud_hz: float = 5.0
    # Publishing every 60 Hz solve overloaded the consumer process's single
    # LCM intake thread (~1.5 s of delivery lag by the wp4 leg in hl61); the
    # follower then acted on paths anchored to a 1.5 s-old yaw and spun in a
    # stale-feedback limit cycle. Solves stay at solve_hz; publishes decimate.
    publish_hz: float = 30.0
    max_odom_age_s: float = 0.5
    route_change_persist_s: float = 10.0
    route_reroute_threshold_m: float = 2.0

    # Costmap (internal, level-aware). Matched to the terrain mapper's 0.1 m
    # voxel output — finer grids are under-sampled by the input (hl58 boxed-in
    # failure); raise together with the mapper voxel size.
    resolution: float = 0.1
    can_pass_under: float = 0.6
    # Traversable grade (rise/run) scaling the Sobel gradient cost; a cell's
    # cost = measured_gradient / max_grade * 100 (lethal at half of it). This
    # replaces `can_climb` (rise-per-cell): can_climb == max_grade x resolution.
    # CAVEAT — cell values are NOT physical robot grades: sub-cell risers
    # quantize onto the 0.1 m grid, so the 31-degree warehouse staircase
    # (0.17 m risers) measures 2-3x its true slope. 3.0 is the go2-physical
    # setting (Jeff, 2026-07-11: "0.3 m per cell for this robot"): it keeps
    # real-scale stairs open (climb corridor 95% free on the warehouse
    # recording) while sub-storey clutter finally scores lethal. History of
    # the old knob: 1.2 -> 0.6 caught cherry-picker-class clutter (recall
    # 5% -> 26%); 0.6 -> 0.3 catches the 0.35-0.7 m suitcase class. The
    # dim_city sim staircase is steeper than this robot could physically
    # climb (its terrain map reads ~0.3 m rise per cell) — the sim blueprint
    # overrides max_grade back to 6.0.
    max_grade: float = 3.0
    # Body-band occupancy gate (OPT-IN, body_min_points=0 disables): >=
    # body_min_points returns between body_step and can_pass_under above a
    # cell's own floor, spanning >= body_min_extent vertically, make it lethal
    # outright. Catches thin sparse-return clutter the gradient still misses —
    # worth enabling on real robots (gentle 0.17 m stair risers stay below
    # body_step), but left off by default: steep-staircase cells straddle tread
    # boundaries with band slivers, and the gate measurably stalled the
    # dim_city staircase climb (start-region free cells 52-76 of 81).
    body_step: float = 0.35
    body_min_points: int = 0
    body_min_extent: float = 0.1
    # Plateau-step gate (0 disables): a cell rising > max_step above the local
    # reference floor (30th percentile of strictly-lower 5x5 neighbors — a
    # min reference misreads open-riser staircases) is lethal unless a
    # staircase continuation excuses it (rise keeps going above / grades away
    # below, AND the nearest riser onto the cell is itself climbable —
    # 1 cm-deep 0.4 m risers are stair-shaped but not traversable). One
    # uphill-only dilation spreads rim hits across the object footprint.
    # This is the robot's single-step ability (0.3 m for the go2 — Jeff,
    # 2026-07-11) and is what marks sub-gradient obstacles: the 0.35-0.7 m
    # suitcases on the 2026-07-09 warehouse recording are unclimbable but
    # were invisible to the gradient cost (obstacle recall 34% -> 75% with
    # the gate + max_grade 3.0). The dim_city sim staircase is steeper than
    # the robot's physical ability — the sim blueprint disables the gate.
    max_step: float = 0.3
    max_safe_fall: float = 0.5
    void_depth_lethal: float = 2.5
    slice_below: float = 1.1
    slice_above: float = 1.5
    half_extent: float = 8.0
    level_hysteresis: float = 0.25

    # Solver (course-tuned values; stories in the Python reference).
    vehicle_width: float = 0.5
    safety_margin: float = 0.1
    influence_radius: float = 0.8
    clearance_weight: float = 4.0
    path_weight: float = 0.35
    commitment_weight: float = 2.0
    carrot_lookahead: float = 4.0
    carrot_lookahead_time_s: float = 4.0
    carrot_lookahead_max: float = 8.0
    carrot_gap_max: float = 1.0
    dijkstra_radius: float = 6.0
    horizon: float = 3.0
    goal_tolerance: float = 0.15
    smoothing_iterations: int = 12
    face_forward_weight: float = 0.8
    # Stop publishing local_path once within this distance of the final goal and
    # the solve can no longer advance (arrived, or pinned as close as the
    # repulsion field allows). Solves keep running at solve_hz so publishing
    # resumes the instant the goal moves — this only silences the near-zero paths
    # that would otherwise churn the trajectory follower at rest on the goal.
    arrival_stop_radius_m: float = 0.6


class RepulsiveFieldNative(NativeModule):
    """Rust-backed repulsive-field local planner — jnav LocalPlanner spec."""

    config: RepulsiveFieldNativeConfig

    terrain_map: In[PointCloud2]
    global_path: In[Path]
    odometry: In[Odometry]
    # Teleop override: MovementManager publishes here whenever keyboard/teleop
    # controls are used. Any message clears the committed route so the planner
    # stops steering toward the goal until a new global_path arrives.
    cancel_goal: In[Bool]

    local_path: Out[Path]
    costmap_cloud: Out[PointCloud2]
