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

"""Lidar loop closure (Rust): Scan Context++ place recognition, gated ICP, and a background GNC pose graph solve."""

from __future__ import annotations

from typing import Any

from pydantic import Field, model_validator

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# Mid-360 + FAST-LIO/Point-LIO odometry error estimate (fitted to ground truth): the default for unset fields.
MID360_ODOMETRY_ERROR_ESTIMATE = {
    "odometry_error_estimate_xy_per_sqrt_m": 3.91e-4,
    "odometry_error_estimate_xy_per_sqrt_rad": 6.35e-3,
    "odometry_error_estimate_z_per_sqrt_m": 6.42e-4,
    "odometry_error_estimate_z_per_sqrt_rad": 3.86e-3,
    "odometry_error_estimate_yaw_per_sqrt_m": 1e-6,
    "odometry_error_estimate_yaw_per_sqrt_rad": 8.66e-3,
    "odometry_error_estimate_tilt_per_sqrt_m": 6.91e-4,
    "odometry_error_estimate_tilt_per_sqrt_rad": 3.3e-3,
}


class PGOConfig(NativeModuleConfig):
    # "deterministic": jobs held to deterministic_config estimates (same graph on any machine); "fast": land on finish
    mode: str = "deterministic"
    # overshoot warnings are logged at most once per this many seconds of data
    overshoot_warning_interval_s: float = 10.0
    # per-job cost estimates in scans (a job holds ceil(estimate) scans, at least one; *_power are unitless exponents)
    deterministic_config: dict[str, float] = Field(
        default_factory=lambda: {
            "fast_match_scans_fixed_overhead": 1.3885433563125162,
            "fast_match_scans_per_keyframe": 1.448251991030438e-07,
            "fast_match_scans_per_point": 7.970467536600849e-05,
            "fast_match_scans_per_source_point": 0.00019838511484150277,
            "verification_scans_fixed_overhead": 5.036066465159226,
            "verification_scans_per_round": 0.7489339387672495,
            "verification_scans_per_round_point": 6.563647896724165e-05,
            "solve_scans_scale": 9.097483642429608e-05,
            "solve_keyframe_power": 2.017,
            "solve_closure_power": -0.215,
            "solve_rejected_power": 0.558,
            "debounce_scans_per_keyframe": 6.093487031356368e-06,
            "merge_scans_per_keyframe": 3.0032671115943386e-07,
            "pressure_scans_fixed_overhead": 0.0007777038336858567,
            "pressure_scans_per_keyframe": 8.708997476399469e-06,
            "pressure_scans_per_closure": 1.2578234731596874e-05,
            "fast_path_scans_fixed_overhead": 0.42,
            "fast_path_scans_per_window_keyframe": 1.7e-3,
            "fast_path_scans_per_keyframe": 8.4e-5,
        }
    )
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
    # ICP pairs points only within this distance (m)
    closure_icp_inlier_distance_m: float = 0.3872983346207417
    # reject a closure whose ICP rms inlier distance (m) is larger
    closure_icp_max_fitness_m: float = 0.3872983346207417
    # closure ICP minimises gaps along surface normals (point-to-plane); False = point-to-point
    closure_icp_point_to_plane: bool = True
    # single-scan ICP target voxel size (m; fitness still on the full target; 0 = full)
    closure_icp_target_resolution_m: float = 0.15
    # stop single-scan ICP once its fitness can no longer pass the gate
    closure_icp_early_stop: bool = True
    # retry alternative places only at or above this pressure
    closure_alternative_places_min_pressure: float = 0.8
    # drop points a scan re-sends from the previous scan (Go2 L1 republishes its last seconds)
    drop_reseen_points: bool = True
    # min points per keyframe cloud; a due keyframe aggregates scans until reached (0 = off)
    auto_aggregate_cloud_size: int = 2000
    # target submap half range = ceil(source clouds x (1 + extra_fraction)) + extra_keyframes
    closure_target_extra_fraction: float = 0.3
    closure_target_extra_keyframes: int = 3
    submap_resolution: float = 0.1
    # min occupied Scan-Context cells (0 = off)
    closure_min_occupancy: int = 80
    # ICP degeneracy (observability) gate; higher = fewer false matches (0 = off)
    closure_min_degeneracy: float = 0.05

    # keyframe dump file for offline checks (f64 ts, raw xyz+q, corrected xyz+q); "" = none
    dump_path: str = ""

    scan_context_num_rings: int = 20
    scan_context_num_sectors: int = 60
    # ring range (m), near the lidar's useful range; 0 = auto-scale from the first scan's extent
    scan_context_max_range_m: float = 0.0
    scan_context_top_k: int = 10
    scan_context_match_threshold: float = 0.4
    scan_context_lidar_height_m: float = 2.0
    # recompute the loop-overlap pressure every this-many-th scan (0 = never)
    pressure_check_frequency: int = 5
    # display-only soft_pressure is 50% at this log10 loop hazard, log-compressed to 0% soft_pressure_decades below
    soft_pressure_half_log10_hazard: float = -0.5
    soft_pressure_decades: float = 100.0
    # reduced pose graph for very long runs: once a keyframe is graph_simplify_frontier_keyframes behind the newest, it rides
    # a keyframe kept on an older pass within graph_simplify_place_radius_m (cloud and descriptor freed) when a kept closure
    # is within graph_simplify_localized_keyframes of it; a closure folded onto riding keyframes goes to the solve only while
    # its pair of places has fewer than graph_simplify_closures_per_pair
    graph_simplify: bool = True
    graph_simplify_frontier_keyframes: int = 1000
    # a keyframe revisits a place only if the kept keyframe there is at least this many keyframes older
    graph_simplify_pass_gap_keyframes: int = 200
    graph_simplify_place_radius_m: float = 1.0
    graph_simplify_localized_keyframes: int = 50
    graph_simplify_closures_per_pair: int = 1
    # one sd of how far a removed keyframe's frozen offset may be off: edges and closures through one carry this much more variance
    graph_simplify_offset_sd_m: float = 0.05
    graph_simplify_offset_sd_rad: float = 0.005
    # odometry folds between consecutive kept keyframes (else it follows the nodes removed keyframes ride)
    graph_simplify_chain_odometry: bool = True
    # past this much folded odometry (m) the walk rides the current place's node instead
    graph_simplify_chain_fold_max_m: float = 30.0

    # k in the yank gate: confidence must clear 1 - exp(-z^2/2k^2), z = yank in robust std devs of yanks so far
    closure_yank_gate_width: float = 1.0
    # max ratio of true drift to the stated odometry error estimate, averaged over by the yank gate (1 = trust it)
    closure_yank_drift_range: float = 100.0
    # skip an extension candidate a full stack of scans could not carry
    closure_extension_skip_unreachable: bool = True
    # evidence / required (yank units): <= barely_fail reject, >= barely_pass accept, else extend (fail < 1 < pass)
    barely_fail_uncertainty_range: float = 0.05
    barely_pass_uncertainty_range: float = 3.0
    # uncertainty range narrows over extension steps, slowed by 1 / (1 - pressure)^this
    decisive_pressure_rate: float = 1.0
    # odometry distance between an extension's source clouds, as a fraction of mean scan reach
    extension_cloud_spacing: float = 0.25
    # extension score = median cloud ratio x exp(this (1 - min(d, 5)^2)) per step, d = step disagreement (0 = median)
    compounding_agreement_sensitivity: float = 1.5
    # adopt a solve only while heat <= threshold; adopting adds its largest move's yank above floor; decay per keyframe
    graph_change_heat_enabled: bool = True
    graph_change_heat_threshold: float = 30.0
    graph_change_heat_floor: float = 20.0
    graph_change_heat_decay: float = 0.995
    # seed a solve from the accepted graph (when it fits) and optionally from the last adopted GNC verdicts
    solve_seed_from_accepted: bool = True
    solve_seed_gnc_verdicts: bool = False
    # solver fast path: a new closure whose older end is within max_keyframes of the newest is applied at once by a local
    # least squares (no GNC) over those keyframes, until the GNC solve covering it lands and supersedes it
    solver_fast_path_enabled: bool = False
    solver_fast_path_max_keyframes: int = 1000
    # each of its Levenberg-Marquardt passes gives up (declines) after this many steps, bounding its cost estimate
    solver_fast_path_max_lm_steps: int = 10
    # closure search pipeline: SC buffer (most similar first) -> fast-match workers -> verification buffer -> workers
    fast_match_workers: int = 2
    sc_buffer_size: int = 4
    verification_workers: int = 2
    verification_buffer_slots: int = 2
    # recent speed = median over the newest this many distinct-pose steps
    solve_speed_window_poses: int = 500
    # write each ICP-scored candidate to <dir>/<source>_<target>.icp ("" = off)
    closure_icp_dump_dir: str = ""
    # re-check a weak match on the last 2, 4, 8, ... (up to this many) keyframes (0 = off)
    closure_extended_icp_max_keyframes: int = 16
    # closure evidence = per-point likelihood ratio, instead of scan-context x ICP x degeneracy
    closure_likelihood_confidence: bool = True
    # after a rejection, try the next scan-context match (up to 1 + floor(sqrt(min(ceil(1/(1-pressure)), top_k-1))))
    closure_alternative_places: bool = True
    # with closure_likelihood_confidence: ignore source points no target scan saw
    closure_likelihood_visibility: bool = True
    # learn the odometry noise scale from kept closures (max likelihood)
    learned_odometry_error_estimate_from_closures: bool = True
    # noise learner reads only pairwise-consistent (PCM core) kept closures
    odometry_error_estimate_evidence_core: bool = True
    # "scan_context": scan-context place recognition proposes closures; "inputs_only": none, the graph is odometry plus the
    # external inputs (landmark_sighting, world_position), and a landmark re-seen on a later visit seeds an ICP closure
    closure_sources: str = "scan_context"
    # with inputs_only: run that landmark-seeded ICP closure (false = landmarks and world positions alone)
    landmark_closure_icp: bool = True
    # 2D: odometry taken as x, y, yaw (z, roll, pitch dropped, clouds levelled) and the solve holds every keyframe's
    # z, roll and pitch at 0; closures, landmark sightings and world positions count only in the plane
    planar_pose_graph: bool = False
    # closure-verification ICP: iteration cap and correspondence gate (m)
    closure_icp_max_iterations: int = 50
    closure_icp_max_correspondence_m: float = 10.0

    # GNC: var_scale loosens closure variance while deciding; inlier_probability sets the chi-squared cutoff
    closure_gnc_var_scale: float = 10.0
    closure_gnc_inlier_probability: float = 0.95
    # Huber threshold on kept closures' whitened residuals in the final solve (0 = plain least squares)
    closure_robust_huber_k: float = 1.345

    # reject closures the graph is already surer of than ICP
    closure_uninformative_gate: bool = True
    # odometry drift std dev per sqrt(m travelled) / sqrt(rad turned); None = MID360_ODOMETRY_ERROR_ESTIMATE
    odometry_error_estimate_xy_per_sqrt_m: float | None = None
    odometry_error_estimate_xy_per_sqrt_rad: float | None = None
    odometry_error_estimate_z_per_sqrt_m: float | None = None
    odometry_error_estimate_z_per_sqrt_rad: float | None = None
    odometry_error_estimate_yaw_per_sqrt_m: float | None = None
    odometry_error_estimate_yaw_per_sqrt_rad: float | None = None
    odometry_error_estimate_tilt_per_sqrt_m: float | None = None
    odometry_error_estimate_tilt_per_sqrt_rad: float | None = None
    # scale the odometry error estimate by lidar range (L1..Mid-360); None = on only if no estimate field is given
    odometry_error_estimate_from_lidar_range: bool | None = None
    # LidarRangeConfig overrides (window_scans, decay_half_life_updates, ...); empty = calibrated defaults
    lidar_range: dict[str, Any] = Field(default_factory=dict)
    # loops' worth of evidence the stated estimate counts as in the learner (0 = closures alone, inf = never learn)
    learned_odometry_error_estimate_stiffness: float = 10.0

    # scan queue depth; past it all but the newest scan are dropped (<= 0 = unbounded)
    max_scan_queue: int = 5
    # rayon threads for closure search and solves (0 = every core)
    worker_threads: int = 0

    @model_validator(mode="after")
    def _odometry_error_estimate(self) -> PGOConfig:
        given = {name: getattr(self, name) for name in MID360_ODOMETRY_ERROR_ESTIMATE}
        if self.odometry_error_estimate_from_lidar_range is None:
            self.odometry_error_estimate_from_lidar_range = all(v is None for v in given.values())
        for name, value in given.items():
            if value is None:
                setattr(self, name, MID360_ODOMETRY_ERROR_ESTIMATE[name])
        return self


class GscPGO(NativeModule):
    """Pose graph optimization with loop closure (Rust)."""

    config: PGOConfig

    cloud: In[PointCloud2]
    odometry: In[Odometry]
    # a sensor's position in a fixed world frame (GPS, mocap): header.frame_id = world frame, child_frame_id = mount
    coordinate_factor: In[Odometry]
    # landmark sighting: header.frame_id = sensor, child_frame_id = landmark name, 6x6 covariance (unknown dims: -1)
    landmark_factor: In[Odometry]
    corrected_odometry: Out[Odometry]
    # the map -> odometry correction, identity until the first one
    tf: IO[TFMessage]
    # a packed record per keyframe: x, y, z, node_id, ts, qx, qy, qz, qw
    pose_graph: Out[PointCloud2]
    # a packed record per keyframe a solve moved: its pose before, then the delta (post = delta * pre)
    loop_closure_event: Out[PointCloud2]
    # closure candidate lifecycle events as packed records, for drawing closure edges
    closure_edges: Out[PointCloud2]
    # JSON lines tagged "kind" (pressure, closure, keyframe, solve, odometry_drift) and "ts"
    debug_log: Out[String]

    @rpc
    def stop(self) -> None:
        super().stop()
