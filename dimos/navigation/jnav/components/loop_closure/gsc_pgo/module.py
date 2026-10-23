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
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PGOConfig(NativeModuleConfig):
    cwd: str | None = "rust"
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
    loop_search_radius: float = 3.0
    loop_time_thresh: float = 5.0
    loop_score_thresh: float = 0.15
    loop_submap_half_range: int = 5
    submap_resolution: float = 0.1
    # Voxel size (m) each incoming cloud is downsampled to before a keyframe stores it; 0 keeps clouds as they arrive
    keyframe_voxel_size: float = 0.0
    # higher=more strict, min number of Scan-Context cells
    # 0 disables
    loop_min_occupancy: int = 80
    # Observability gate (Zhang 2016 / X-ICP degeneracy): higher = less false positive matches
    # 0 disables.
    loop_min_degeneracy: float = 0.05

    # basically only needed because of the go2, and partly b/c we don't have rust tf.get yet
    subtract_odom_from_cloud: bool = False

    # only use global map for debugging
    global_map_publish_rate: float = 0.0
    # Where each pose graph snapshot's keyframes go for offline checks (f64 ts, raw xyz+q, corrected xyz+q), empty for never.
    dump_path: str = ""
    global_map_voxel_size: float = 0.1

    scan_context_num_rings: int = 20
    scan_context_num_sectors: int = 60
    # Ring range in meters; keep near the lidar's useful range (a fixed 80 m
    # collapses a short-range Go2 L1 to one ring). 0 = auto-scale from the
    # first scan's extent, which adapts across sensors (Go2 L1 vs mid360).
    scan_context_max_range_m: float = 0.0
    scan_context_top_k: int = 10
    scan_context_match_threshold: float = 0.4
    scan_context_lidar_height_m: float = 2.0

    # Reject a loop whose match confidence (scan-context x ICP x degeneracy, each normalized by its own gate) is
    # below 1 - exp(-z^2/2), z = how hard it yanks the accepted pose graph in robust std devs of the yanks of loops
    # scored so far. The inputs are recorded per candidate either way (pgo-eval json, <dump_path>.loops.jsonl).
    loop_yank_gate: bool = True
    # k in the yank gate's required confidence 1 - exp(-yank_z^2 / 2k^2)
    loop_yank_gate_width: float = 1.0
    # Divide yank by sqrt(keyframe gap) before scaling: odometry drift grows as a random walk between the loop's ends
    loop_yank_per_sqrt_gap: bool = True
    # Write every ICP-scored candidate's source cloud, target submap and ICP result to <dir>/<source>_<target>.icp
    loop_icp_dump_dir: str = ""
    # With loop_yank_gate: re-check a too-weak match on the last 2, 4, 8, ... (up to this many) keyframes, seeded by the first ICP. 0 = off
    loop_extended_icp_max_keyframes: int = 16
    # Loop-verification ICP: iteration cap and correspondence gate (m).
    loop_icp_max_iterations: int = 50
    loop_icp_max_correspondence_m: float = 10.0

    # Each new loop re-solves the whole pose graph on a background thread from the odometry, with
    # GNC (truncated least squares) keeping the loops that agree with the odometry and each other.
    # loop_gnc_var_scale loosens each loop's ICP-fitness variance while GNC decides, and
    # loop_gnc_inlier_probability sets its chi-squared cutoff (lower rejects more).
    loop_gnc_var_scale: float = 10.0
    loop_gnc_inlier_probability: float = 0.95
    # Huber threshold on a kept loop's whitened residual in the final solve (0 = plain least squares).
    loop_robust_huber_k: float = 1.345
    # A background solve is folded in once the map has grown by this fraction of its size at
    # dispatch (waiting for it if needed), so a replay gives the same graph however fast the machine
    # is. A solve's cost grows with the map (~2 ms per keyframe; keyframes come ~0.7 s apart), so 5%
    # leaves it several times the time it needs.
    loop_gnc_apply_after_fraction: float = 0.05
    # False folds a solve in as soon as it finishes, keyframes or not (a robot standing
    # still at a loop still gets the closure); True keeps the keyframe schedule so replays repeat.
    loop_gnc_deterministic: bool = True

    # "stiffness" in different dimensions.
    # Making roll/pitch stiff is important for mid360. irrelevant for go2 odom
    # These suit a robot on mostly level ground. A handheld rig or long stairs drifts vertically through a slow
    # pitch bias, which stiff roll/pitch and z cannot absorb, so GNC rejects the true loop; there use
    # odom_rot_roll_pitch_var = odom_rot_yaw_var and odom_trans_z_var = odom_trans_xy_var (office_555_loop1:
    # 36 m vertical drift, end-to-start 38.9 m -> 0.2 m).
    odom_rot_roll_pitch_var: float = 1e-8
    odom_rot_yaw_var: float = 1e-5
    odom_trans_xy_var: float = 1e-4
    odom_trans_z_var: float = 1e-6

    # usually que is 1 lidar frame, only goes up after a compute spike (backpressure); past
    # this the oldest scans are dropped, so a stall can't leave the module seconds stale
    max_scan_queue: int = 5


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
    # each loop candidate's lifecycle (proposed, gated, accepted, rejected, applied) as packed records
    # with both keyframes' current positions and a confidence, for drawing loop edges
    loop_edges: Out[PointCloud2]
    # debug only, gated on global_map_publish_rate > 0
    debug_global_map: Out[PointCloud2]

    @rpc
    def stop(self) -> None:
        super().stop()
