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

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
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
    min_loop_detect_duration: float = 2.0
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
    loop_yank_gate_max_distance_m: float = 0.0

    # Minimum keyframe-index separation for a loop closure. A closure only corrects
    # drift accumulated over the trajectory between the pair; a near-in-sequence pair
    # spans negligible drift, so a closure there can only inject error. 0 disables.
    loop_min_id_gap: int = 0

    # Long-jump agreement buffer. A closure whose candidate distance is within
    # loop_instant_accept_distance_m is committed to the graph instantly; a longer
    # jump is parked in a buffer (max 4, FIFO) and only committed once
    # loop_buffer_min_agree buffered jumps agree on the same rigid map-correction:
    # implied-correction translation within loop_buffer_agreement_trans_m and
    # rotation within loop_buffer_agreement_rot_deg. <= 0 disables the buffer.
    loop_instant_accept_distance_m: float = 0.0
    loop_buffer_agreement_trans_m: float = 1.0
    loop_buffer_agreement_rot_deg: float = 10.0
    loop_buffer_min_agree: int = 2

    # Robust (Huber) kernel on all loop factors (lidar + location). Keeps ISAM2 determinate
    # when a large loop applies a big one-shot correction; a no-op on already-tight graphs.
    loop_robust_kernel: bool = True
    loop_robust_huber_k: float = 1.345

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
    loop_gnc_inlier_probability: float = 0.99

    # enable things like April tags to be contraints in the pose graph
    use_location_constraints: bool = False
    # for interpolating (value=time)
    odom_buffer_window: float = 10.0

    # "stiffness" in different dimensions.
    # Making roll/pitch stiff is important for mid360. irrelevant for go2 odom
    odom_rot_roll_pitch_var: float = 1e-8
    odom_rot_yaw_var: float = 1e-5
    odom_trans_xy_var: float = 1e-4
    odom_trans_z_var: float = 1e-6
    per_keyframe_roll_pitch_var: float = 1e-4
    per_keyframe_roll_pitch_prior: bool = False
    anchor_roll_pitch_var: float = 1e-12

    # usually que is 1 lidar frame, only goes up after a big gtsam compute spike (backpressure) 100 is overkill but whatever
    max_scan_queue: int = 100


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
    correction: Out[
        TFMessage
    ]  # i know, kinda redundant, but makes visuals much easier to make and is low-cost
    tf_deformation_nodes: Out[DeformationNode]  # important for future fast-tf-graph lookup
    tf: Out[TFMessage]
    # debug only
    _global_map: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()
        self.tf.publish(
            TFMessage(
                Transform(
                    frame_id=self.config.map_frame,
                    child_frame_id=self.config.child_frame_id,
                )
            )
        )
        self.register_disposable(
            Disposable(
                self.correction.transport.subscribe(self._on_correction_for_tf, self.correction)
            )
        )

    def _on_correction_for_tf(self, correction: TFMessage) -> None:
        self.tf.publish(TFMessage(*correction.transforms))

    @rpc
    def stop(self) -> None:
        super().stop()
