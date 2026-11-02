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

"""Lidar loop closure: Scan Context++ (https://arxiv.org/abs/2109.13494) place recognition and ICP,
gated so a closure does no harm, and a pose graph re-solved with GNC on a background thread. All Rust.
"""

from __future__ import annotations

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PGOConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/navigation/jnav/components/loop_closure/gsc_pgo/rust"
    executable: str = "result/bin/gsc-pgo"
    build_command: str | None = "nix build .#default --no-write-lock-file"
    stdin_config: bool = True

    # outputs: map → odom
    map_frame: str = "map"
    child_frame_id: str = "odom"
    # TODO: when rust gets tf.get, remove this (don't need to require things in base_link)
    body_frame: str = "base_link"

    # Keyframe detection
    keyframe_min_rotation_degrees: float = 10.0
    keyframe_min_distance_meters: float = 0.5

    # Loop closure knobs
    closure_search_radius: float = 3.0
    closure_score_thresh: float = 0.15
    closure_submap_half_range: int = 5
    submap_resolution: float = 0.1
    # higher=more strict, min number of Scan-Context cells
    # 0 disables
    closure_min_occupancy: int = 80
    # Observability gate (Zhang 2016 / X-ICP degeneracy): higher = less false positive matches
    # 0 disables.
    closure_min_degeneracy: float = 0.05

    # Where each pose graph snapshot's keyframes go for offline checks (f64 ts, raw xyz+q, corrected xyz+q), empty for never.
    dump_path: str = ""

    scan_context_num_rings: int = 20
    scan_context_num_sectors: int = 60
    # Ring range in meters; keep near the lidar's useful range (a fixed 80 m
    # collapses a short-range Go2 L1 to one ring). 0 = auto-scale from the
    # first scan's extent, which adapts across sensors (Go2 L1 vs mid360).
    scan_context_max_range_m: float = 0.0
    scan_context_top_k: int = 10
    scan_context_match_threshold: float = 0.4
    scan_context_lidar_height_m: float = 2.0
    # recompute the loop-overlap pressure (debug_log unresolved_loop_probability) every this-many-th scan; 0 = never
    pressure_check_frequency: int = 5

    # A closure's match confidence (scan-context x ICP x degeneracy, each normalized by its own gate) must clear
    # 1 - exp(-z^2/2k^2), z = how hard it yanks the graph (per sqrt keyframe gap) in robust std devs of the yanks scored so far; this is k
    closure_yank_gate_width: float = 1.0
    # Write every ICP-scored candidate's source cloud, target submap and ICP result to <dir>/<source>_<target>.icp
    closure_icp_dump_dir: str = ""
    # Re-check a too-weak match on the last 2, 4, 8, ... (up to this many) keyframes, seeded by the first ICP. 0 = off
    closure_extended_icp_max_keyframes: int = 16
    # Closure-verification ICP: iteration cap and correspondence gate (m).
    closure_icp_max_iterations: int = 50
    closure_icp_max_correspondence_m: float = 10.0

    # Each new closure re-solves the whole pose graph on a background thread from the odometry, with
    # GNC (truncated least squares) keeping the closures that agree with the odometry and each other.
    # closure_gnc_var_scale loosens each closure's ICP-fitness variance while GNC decides, and
    # closure_gnc_inlier_probability sets its chi-squared cutoff (lower rejects more).
    closure_gnc_var_scale: float = 10.0
    closure_gnc_inlier_probability: float = 0.95
    # Huber threshold on a kept closure's whitened residual in the final solve (0 = plain least squares).
    closure_robust_huber_k: float = 1.345
    # Hold keyframes with two or more independent cycles nearby, so a solve covers only the part of the map that can still move
    closure_cooling: bool = False

    # Odometry drift per keyframe (variance): Point-LIO's measured against ground truth on gir_stairs2 (roll/pitch
    # 1.4e-6, yaw 3.5e-6, xy 2.9e-4 over 50-keyframe spans, z 2.8e-5..1.6e-4). Locking roll/pitch and z tighter (a
    # level-robot assumption) makes a loop's vertical drift unabsorbable: office_555_loop1's start-to-end gap then
    # stays at 18.5 m instead of closing to 7 cm.
    odom_rot_roll_pitch_var: float = 1e-6
    odom_rot_yaw_var: float = 3e-6
    odom_trans_xy_var: float = 1e-4
    odom_trans_z_var: float = 1e-4

    # usually que is 1 lidar frame, only goes up after a compute spike (backpressure); past
    # this the oldest scans are dropped, so a stall can't leave the module seconds stale
    max_scan_queue: int = 5
    # threads closure search and solves may use, leaving the other cores to the nav stack (0 = every core)
    worker_threads: int = 2


class GscPGO(NativeModule):
    """Pose graph optimization with loop closure — Rust port of gsc_pgo."""

    config: PGOConfig

    cloud: In[PointCloud2]
    odometry: In[Odometry]
    corrected_odometry: Out[Odometry]
    # the map -> odometry correction, identity until the first one
    tf: Out[TFMessage]
    # a packed record per keyframe: x, y, z, node_id, ts, qx, qy, qz, qw
    pose_graph: Out[PointCloud2]
    # a packed record per keyframe a solve moved: its pose before, then the delta (post = delta * pre)
    loop_closure_event: Out[PointCloud2]
    # each closure candidate's lifecycle (proposed, gated, accepted, rejected, applied) as packed records
    # with both keyframes' current positions and a confidence, for drawing closure edges
    closure_edges: Out[PointCloud2]
    # debugging numbers as JSON lines, one per pressure recompute (pressure_check_frequency): ts, unresolved_loop_probability (chance the newest scan
    # overlaps an earlier stretch of the path no closure has tied it to), expected_overlaps, most_likely_keyframe/_probability
    debug_log: Out[String]

    @rpc
    def stop(self) -> None:
        super().stop()
