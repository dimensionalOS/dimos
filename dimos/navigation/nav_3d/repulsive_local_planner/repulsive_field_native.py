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

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class RepulsiveFieldNativeConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/repulsive_field"
    build_command: str | None = "cargo build --release"
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
    can_climb: float = 1.2
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
    tail_reversal_trim_deg: float = 100.0


class RepulsiveFieldNative(NativeModule):
    """Rust-backed repulsive-field local planner — jnav LocalPlanner spec."""

    config: RepulsiveFieldNativeConfig

    terrain_map: In[PointCloud2]
    global_path: In[Path]
    odometry: In[Odometry]
    route_tail: In[Path]

    local_path: Out[Path]
    costmap_cloud: Out[PointCloud2]
