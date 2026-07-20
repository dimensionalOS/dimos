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

"""Native Rust PGO module.

GTSAM iSAM2 pose graph + ICP loop closure + Scan Context place recognition, with
the PCL/ICP/Scan-Context machinery implemented in Rust (see rust/src/) over a thin
C FFI shim onto pinned gtsam. The module executable speaks the dimos_module
stdin-JSON protocol, so ``stdin_config`` is on.
"""

from __future__ import annotations

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.jnav.msgs.DeformationNode import DeformationNode
from dimos.navigation.jnav.msgs.Graph3D import Graph3D
from dimos.navigation.jnav.msgs.GraphDelta3D import GraphDelta3D
from dimos.navigation.jnav.msgs.LocationConstraint import LocationConstraint
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PGOConfig(NativeModuleConfig):
    # The crate lives in this module dir; the flake pins gtsam + the toolchain
    # env (GTSAM_INCLUDE_DIR etc.) that build.rs consumes. NativeModule runs
    # build_command with cwd=<this dir>/rust, so `path:.` is the crate dir.
    cwd: str | None = "rust"
    executable: str = "target/release/gsc-pgo"
    build_command: str | None = "nix develop path:. --command cargo build --release"
    stdin_config: bool = True

    # Output/map frame. NOTE: named world_frame (not frame_id like the C++
    # wrapper) because NativeModuleConfig.to_config_dict() strips base-config
    # field names — frame_id is one — from the stdin config JSON.
    world_frame: str = "map"
    child_frame_id: str = "odom"
    body_frame: str = "base_link"

    # Keyframe detection
    key_pose_delta_deg: float = 10.0
    key_pose_delta_trans: float = 0.5

    # Loop closure
    loop_search_radius: float = 3.0
    loop_time_thresh: float = 5.0
    loop_score_thresh: float = 0.15
    loop_submap_half_range: int = 5
    submap_resolution: float = 0.1
    min_loop_detect_duration: float = 2.0
    # Feature-poverty gate: skip loop search when the scan's descriptor
    # vertical-structure std is below this (open grass can't place itself ->
    # PGO no-op). 0 = off. Superseded by loop_min_occupancy/loop_min_degeneracy
    # (structure overlaps too much between scenes to threshold cleanly).
    min_descriptor_std: float = 0.0

    # Structure-spread gate: require >= this many occupied Scan-Context cells.
    # Open grass clusters returns near the sensor (few rings filled); built
    # scenes spread out to range. Calibrated on go2 fastlio (1200-cell 20x60
    # descriptor): grassy ~70 vs gir_park ~88 vs downtown ~120 at equal point
    # count -> measures spread, not density. 0 disables.
    loop_min_occupancy: int = 80
    # Observability gate (Zhang 2016 / X-ICP degeneracy): reject a candidate
    # whose source scan's smallest normalized normal-scatter eigenvalue is below
    # this. Planar/degenerate (grass) -> ~0; ICP slides in-plane and reports low
    # fitness for a bogus closure. Real scenes (incl. sparse gir_park) sit >0.15.
    # 0 disables.
    loop_min_degeneracy: float = 0.05

    # Input mode: transform world-frame scans to body-frame using odom
    unregister_input: bool = True

    # Debug global-map publishing — OFF by default. Emitted on the internal
    # `_global_map` port (leading underscore) so it never autoconnects to a
    # consumer's `global_map` In: the terrain_mapper is the planner's single
    # authoritative global_map. Two producers on `global_map` made the costmap
    # flicker. Set a rate > 0 only for viz/debug of the PGO's corrected cloud.
    global_map_voxel_size: float = 0.1
    global_map_publish_rate: float = 0.0

    # Scan Context place recognition (used by loop closure search)
    use_scan_context: bool = True
    scan_context_num_rings: int = 20
    scan_context_num_sectors: int = 60
    scan_context_max_range_m: float = 80.0
    scan_context_top_k: int = 10
    scan_context_match_threshold: float = 0.4
    scan_context_lidar_height_m: float = 2.0

    # Skip ICP on candidates farther than this (m). 0 disables.
    loop_candidate_max_distance_m: float = 30.0

    # Robust (Huber) kernel on all loop factors (lidar + location). Off = original.
    loop_robust_kernel: bool = False
    loop_robust_huber_k: float = 1.345

    # Location constraints (decoupled perceiver -> PGO factor-graph manager).
    # When set, the PGO ingests LocationConstraint events on the
    # `location_constraints` In. Each becomes its own pose node (placed from
    # interpolated odometry at the constraint's timestamp) plus a
    # BetweenFactor(node, location) whose noise model is the covariance carried in
    # the message. Two constraints sharing a to_id share the location variable, so
    # a revisit closes the loop; a constraint_instance_id lets an external source
    # revise/remove its earlier constraints. Off by default.
    use_location_constraints: bool = False
    # Seconds of odometry history retained, for interpolating a constraint's pose
    # at its own timestamp.
    odom_buffer_window: float = 10.0

    # First-keyframe absolute anchor prior. The pose graph is relative-only, so
    # one keyframe must be pinned to fix the gauge. Each axis has its own stiffness
    # (smaller variance = harder pin). A tight anchor_rp_var pins roll/pitch to the
    # initial LIO attitude (which is gravity-aligned by the front end) so loop
    # closures cannot tilt the map; loosen it to let odom/loops decide roll/pitch.
    anchor_rp_var: float = 1e-12
    anchor_yaw_var: float = 1e-12
    anchor_trans_var: float = 1e-12
    # Optional roll/pitch-only prior on EVERY keyframe (yaw + translation left free).
    # Anchoring only kf0 lets a big loop closure tilt inner keyframes' roll/pitch,
    # which converts horizontal travel into vertical and corrupts z by tens of
    # metres. Pinning every keyframe's roll/pitch to its initial LIO attitude keeps
    # the closure in-plane and preserves z structure. Default OFF: the anisotropic
    # odometry between-factor is the primary tilt-preservation mechanism. Turn on
    # for a harder lock when the front end's absolute tilt is trustworthy (e.g. ZUPT
    # in the LIO estimator).
    per_keyframe_rp_prior: bool = False
    per_keyframe_rp_var: float = 1e-4

    # Anisotropic odometry between-factor: the LIO relative roll/pitch is accurate
    # (IMU sees gravity each step) but yaw drifts, so roll/pitch are stiff and yaw
    # looser. This keeps a loop closure from sloshing its (mostly-yaw) correction
    # into roll/pitch — a tilt that converts horizontal travel into vertical and
    # corrupts z. Roll/pitch variance is small but nonzero, so landmarks can still
    # correct slow tilt drift across the graph ("accurate but not perfect").
    odom_rot_rp_var: float = 1e-8
    odom_rot_yaw_var: float = 1e-5
    odom_trans_xy_var: float = 1e-4
    odom_trans_z_var: float = 1e-6

    # Bounded FIFO depth: keep at most this many pending scans, dropping the
    # oldest when full (<=0 = unbounded). Generous enough that an ack-gated eval
    # replay never drops a scan, bounded enough to cap live latency/memory.
    max_scan_queue: int = 100

    debug: bool = False


class PGO(NativeModule):
    """Pose graph optimization with loop closure — Rust port of gsc_pgo."""

    config: PGOConfig

    # named "lidar" to match the LoopClosure spec; the binary pairs it with the
    # latest odometry pose internally, so a raw sensor-frame scan is expected.
    lidar: In[PointCloud2]
    odometry: In[Odometry]
    # Optional: decoupled LocationConstraint events from a perceiver. Only
    # consumed when config.use_location_constraints is set; each becomes its own
    # pose node + a BetweenFactor(node, location) that GTSAM optimizes jointly.
    location_constraints: In[LocationConstraint]
    corrected_odometry: Out[Odometry]
    correction: Out[Transform]
    pose_graph: Out[Graph3D]
    loop_closure_event: Out[GraphDelta3D]
    # Per-keyframe pose-graph nodes, published individually (un-batched) so a
    # recorder can stream them. tf_id on each identifies the corrected edge
    # (world_frame -> child_frame_id). Autoconnects to the Recorder's like-named port.
    tf_deformation_nodes: Out[DeformationNode]
    # Internal/debug only (off by default) — see global_map_publish_rate. Named
    # with a leading underscore so autoconnect won't wire it to `global_map` Ins.
    _global_map: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()
        self.tf.publish(
            Transform(
                frame_id=self.config.world_frame,
                child_frame_id=self.config.child_frame_id,
            )
        )
        self.register_disposable(
            Disposable(
                self.correction.transport.subscribe(self._on_correction_for_tf, self.correction)
            )
        )
        if self.config.debug:
            logger.info("PGO native module started (Rust iSAM2 port)")

    def _on_correction_for_tf(self, correction: Transform) -> None:
        self.tf.publish(correction)

    @rpc
    def stop(self) -> None:
        super().stop()
