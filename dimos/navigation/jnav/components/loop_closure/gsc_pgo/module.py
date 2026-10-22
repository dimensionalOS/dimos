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

"""This PGO is "gated" to make sure it doesn't do harm.
Based on Scan Context++ https://arxiv.org/abs/2109.13494
All rust except for gtsam (ffi)
"""

from __future__ import annotations

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.jnav.msgs.DeformationNode import DeformationNode
from dimos.navigation.jnav.msgs.Graph3D import Graph3D
from dimos.navigation.jnav.msgs.GraphDelta3D import GraphDelta3D
from dimos.navigation.jnav.msgs.LocationConstraint import LocationConstraint
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

    # One dial over every false-closure gate below, instead of setting them individually:
    #   0  accept almost any match (no occupancy/degeneracy/GNC gating)
    #   2  the same values these fields already default to
    #   4  every gate on, plus a tight GNC
    # Redundant with the individual loop_* gates on purpose. Setting it overwrites all of
    # them, so use one or the other. -1 (default) leaves the individual fields alone.
    loop_conservativeness: int = -1

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
    min_loop_detect_duration: float = 2.0
    # Seconds of keyframe time between loop searches, whatever their outcome: in a small space every
    # keyframe finds a candidate and a rejected ICP per keyframe can outrun the scan rate. 0 disables.
    loop_search_min_interval: float = 1.0
    # Feature-poverty gate: skip loop search when the scan's descriptor vertical-structure std is below this
    # 0 disables
    min_descriptor_std: float = 0.0
    # higher=more strict, min number of Scan-Context cells
    # 0 disables
    loop_min_occupancy: int = 80
    # Observability gate (Zhang 2016 / X-ICP degeneracy): higher = less false positive matches
    # 0 disables.
    loop_min_degeneracy: float = 0.05
    # In self-similar spaces make higher in self-similar places to prevent false positives
    # 0 disables
    loop_max_lowe_ratio: float = 0.0

    # basically only needed because of the go2, and partly b/c we don't have rust tf.get yet
    subtract_odom_from_cloud: bool = False

    # only use global map for debugging
    global_map_publish_rate: float = 0.0
    # Where each pose graph snapshot's keyframes go for offline checks (f64 ts, raw xyz+q, corrected xyz+q), empty for never.
    dump_path: str = ""
    global_map_voxel_size: float = 0.1

    # Scan Context place recognition (used by loop closure search), https://arxiv.org/abs/2109.13494
    use_scan_context: bool = True
    scan_context_num_rings: int = 20
    scan_context_num_sectors: int = 60
    # Ring range in meters; keep near the lidar's useful range (a fixed 80 m
    # collapses a short-range Go2 L1 to one ring). 0 = auto-scale from the
    # first scan's extent, which adapts across sensors (Go2 L1 vs mid360).
    scan_context_max_range_m: float = 0.0
    scan_context_top_k: int = 10
    scan_context_match_threshold: float = 0.4
    scan_context_lidar_height_m: float = 2.0

    # Skip ICP on candidates farther than this (m). 0 disables. Must exceed the worst
    # expected odom drift at revisit so far-drifted large loops still reach ICP
    # (a 200 m cap discarded ~700 genuine revisits on huge_loop with drifted fastlio
    # odom); ICP fitness + GNC already reject the structural aliases this gated.
    loop_candidate_max_distance_m: float = 0.0

    # False-closure gate on graph yank (ICP-refined relative rotation vs the odom-chain
    # estimate). A near-coincident pair (candidate distance < gate distance) cannot truly
    # be rotated, so a large ICP rotation disagreement flags a structural-alias false match.
    # Reject when yank rotation exceeds this many degrees. 0 disables the gate entirely.
    loop_max_yank_rotation_deg: float = 0.0
    # Reject a loop whose match confidence (scan-context x ICP x degeneracy, each normalized by its own gate) is
    # below 1 - exp(-z^2/2), z = how hard it yanks the accepted pose graph in robust std devs of the yanks of loops
    # scored so far. The inputs are recorded per candidate either way (pgo-eval json, <dump_path>.loops.jsonl).
    loop_yank_gate: bool = False
    # k in the yank gate's required confidence 1 - exp(-yank_z^2 / 2k^2)
    loop_yank_gate_width: float = 1.0
    # Divide yank by sqrt(keyframe gap) before scaling: odometry drift grows as a random walk between the loop's ends
    loop_yank_per_sqrt_gap: bool = True
    # Write every ICP-scored candidate's source cloud, target submap and ICP result to <dir>/<source>_<target>.icp
    loop_icp_dump_dir: str = ""
    # With loop_yank_gate: re-check a too-weak match on the last 2, 4, 8, ... (up to this many) keyframes, seeded by the first ICP. 0 = off
    loop_extended_icp_max_keyframes: int = 0
    loop_yank_gate_max_distance_m: float = 0.0

    # Minimum keyframe-index separation for a loop closure. A closure only corrects
    # drift accumulated over the trajectory between the pair; a near-in-sequence pair
    # spans negligible drift, so a closure there can only inject error. 0 disables.
    loop_min_id_gap: int = 0

    # Robust (Huber) kernel on all loop factors (lidar + location). Keeps ISAM2 determinate
    # when a large loop applies a big one-shot correction; a no-op on already-tight graphs.
    loop_robust_kernel: bool = True
    loop_robust_huber_k: float = 1.345
    # Loop-verification ICP: iteration cap, correspondence gate (m), inlier-only fitness, minimum
    # submap points (0 = off), and seeding at the current pose instead of the matched keyframe.
    loop_icp_max_iterations: int = 50
    loop_icp_max_correspondence_m: float = 10.0
    loop_icp_inlier_fitness: bool = False
    loop_icp_min_points: int = 0
    loop_icp_seed_at_current_pose: bool = False
    # Loop factor noise: rotation variance (0 = the ICP fitness) and a translation-variance floor.
    loop_rotation_variance: float = 0.0
    loop_translation_variance_floor: float = 0.0
    # iSAM2 update passes after an update that adds a closure.
    loop_refinement_passes: int = 5
    # Then keep relinearizing until no keyframe moves more than this per pass (capped), so a
    # closure moves the map once instead of drifting over the following keyframes.
    loop_refinement_tolerance_m: float = 0.01
    loop_refinement_max_passes: int = 200

    # After each loop closure, re-solve the whole graph once with a batch GNC (graduated
    # non-convexity, TLS loss) optimizer, pinning the odometry backbone as known inliers. GNC
    # rejects the mutually conflicting closures that Huber only down-weights, so a loop that
    # under-closes incrementally actually snaps shut. loop_gnc_var_scale loosens each loop's
    # score-derived variance so GNC keeps the true consensus set instead of over-rejecting.
    # loop_gnc_inlier_probability is the chi-squared inlier cost threshold: lowering it tightens
    # the test so GNC rejects more loops, and it is independent of a loop's variance (unlike
    # loop_gnc_var_scale, which also weakens the surviving edges' pull).
    loop_gnc_final: bool = True
    loop_gnc_var_scale: float = 10.0
    loop_gnc_inlier_probability: float = 0.95
    # A background GNC solve is folded in once the map has grown by this fraction of its size at
    # dispatch (waiting for it if needed), so a replay gives the same graph however fast the machine
    # is. A solve's cost grows with the map (~2 ms per keyframe; keyframes come ~0.7 s apart), so 5%
    # leaves it several times the time it needs.
    loop_gnc_apply_after_fraction: float = 0.05
    # Consecutive GNC solves that must agree before a loop enters or leaves the graph, so a verdict
    # one solve gives and the next takes back never moves the map.
    loop_gnc_confirmations: int = 2
    # False folds a GNC verdict as soon as its solve finishes, keyframes or not (a robot standing
    # still at a loop still gets the closure); True keeps the keyframe schedule so replays repeat.
    loop_gnc_deterministic: bool = True

    # enable things like April tags to be contraints in the pose graph
    use_location_constraints: bool = False
    # for interpolating (value=time)
    odom_buffer_window: float = 10.0

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
    per_keyframe_roll_pitch_var: float = 1e-4
    per_keyframe_roll_pitch_prior: bool = False
    anchor_roll_pitch_var: float = 1e-12

    # usually que is 1 lidar frame, only goes up after a big gtsam compute spike (backpressure); past
    # this the oldest scans are dropped, so a stall can't leave the module seconds stale
    max_scan_queue: int = 5


class GscPGO(NativeModule):
    """Pose graph optimization with loop closure — Rust port of gsc_pgo."""

    config: PGOConfig

    cloud: In[PointCloud2]
    odometry: In[Odometry]
    # optional, allows April tags (and other stuff) to influence the pose graph
    # Is almost 1-to-1 to a BetweenFactor(node, location) in GTSAM by design
    location_constraints: In[LocationConstraint]
    corrected_odometry: Out[Odometry]
    pose_graph: Out[Graph3D]
    loop_closure_event: Out[GraphDelta3D]
    # pose_graph and loop_closure_event as packed PointCloud2 records, for fast-reform consumers.
    pose_graph_records: Out[PointCloud2]
    loop_closure_records: Out[PointCloud2]
    # each loop candidate's lifecycle (proposed, gated, GNC votes, accepted, rejected, applied) as
    # packed records with both keyframes' current positions and a confidence, for drawing loop edges
    loop_edges: Out[PointCloud2]
    correction: Out[
        TFMessage
    ]  # i know, kinda redundant, but makes visuals much easier to make and is low-cost
    tf_deformation_nodes: Out[DeformationNode]  # important for future fast-tf-graph lookup
    # The correction again, identity until the first one.
    tf: Out[TFMessage]
    # debug only, gated on global_map_publish_rate > 0
    debug_global_map: Out[PointCloud2]

    @rpc
    def stop(self) -> None:
        super().stop()
