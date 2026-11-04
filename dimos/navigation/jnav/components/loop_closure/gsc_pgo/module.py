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

"""Lidar loop closure (Rust) from lidar scans and tf alone: each scan is posed by odom_frame -> body_frame from tf at its stamp."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Literal

from pydantic import Field, model_validator

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.jnav.msgs.LocationConstraint import LocationConstraint
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# the core's defaults, shared with the Rust `Config::default()`
DEFAULTS: dict[str, Any] = json.loads((Path(__file__).parent / "rust" / "defaults.json").read_text())
ODOMETRY_ERROR_ESTIMATE_FIELDS = [
    f"odometry_error_estimate_{source}_per_sqrt_{unit}"
    for source in ("xy", "z", "yaw", "tilt")
    for unit in ("m", "rad")
]


class PGOConfig(NativeModuleConfig):
    # "deterministic": jobs held to deterministic_config estimates (same graph on any machine); "fast": land on finish
    mode: Literal["deterministic", "fast"] = DEFAULTS["mode"]
    # overshoot warnings are logged at most once per this many seconds of data
    overshoot_warning_interval_s: float = DEFAULTS["overshoot_warning_interval_s"]
    # per-job cost estimates in scans (ceil, at least one; *_power unitless), defaulting to the R1 Jetson fit; keys not given keep it
    deterministic_config: dict[str, float] = Field(
        default_factory=lambda: dict(DEFAULTS["deterministic_config"])
    )
    source_dir: str | None = "dimos/navigation/jnav/components/loop_closure/gsc_pgo/rust"
    executable: str = "result/bin/gsc-pgo"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    # tf poses body_frame in odom_frame; the map_frame -> odom_frame correction goes back on tf
    map_frame: str = "map"
    odom_frame: str = "odom"
    body_frame: str = "base_link"

    # Keyframe detection
    keyframe_min_rotation_degrees: float = DEFAULTS["keyframe_min_rotation_degrees"]
    keyframe_min_distance_meters: float = DEFAULTS["keyframe_min_distance_meters"]

    # Loop closure knobs
    closure_search_radius: float = DEFAULTS["closure_search_radius"]
    # ICP pairs points only within this distance (m)
    closure_icp_inlier_distance_m: float = DEFAULTS["closure_icp_inlier_distance_m"]
    # reject a closure whose ICP rms inlier distance (m) is larger
    closure_icp_max_fitness_m: float = DEFAULTS["closure_icp_max_fitness_m"]
    # closure ICP minimises gaps along surface normals (point-to-plane); False = point-to-point
    closure_icp_point_to_plane: bool = DEFAULTS["closure_icp_point_to_plane"]
    # single-scan ICP target voxel size (m; fitness still on the full target; 0 = full)
    closure_icp_target_resolution_m: float = DEFAULTS["closure_icp_target_resolution_m"]
    # stop single-scan ICP once its fitness can no longer pass the gate
    closure_icp_early_stop: bool = DEFAULTS["closure_icp_early_stop"]
    # retry alternative places only at or above this pressure
    closure_alternative_places_min_pressure: float = DEFAULTS["closure_alternative_places_min_pressure"]
    # drop points a scan re-sends from the previous scan (Go2 L1 republishes its last seconds)
    drop_reseen_points: bool = DEFAULTS["drop_reseen_points"]
    # min points per keyframe cloud; a due keyframe aggregates scans until reached (0 = off)
    auto_aggregate_cloud_size: int = DEFAULTS["auto_aggregate_cloud_size"]
    # target submap half range = ceil(source clouds x (1 + extra_fraction)) + extra_keyframes
    closure_target_extra_fraction: float = DEFAULTS["closure_target_extra_fraction"]
    closure_target_extra_keyframes: int = DEFAULTS["closure_target_extra_keyframes"]
    submap_resolution: float = DEFAULTS["submap_resolution"]
    # min occupied Scan-Context cells (0 = off)
    closure_min_occupancy: int = DEFAULTS["closure_min_occupancy"]
    # ICP degeneracy (observability) gate; higher = fewer false matches (0 = off)
    closure_min_degeneracy: float = DEFAULTS["closure_min_degeneracy"]

    # keyframe dump file for offline checks (f64 ts, raw xyz+q, corrected xyz+q); "" = none
    dump_path: str = ""

    scan_context_num_rings: int = DEFAULTS["scan_context_num_rings"]
    scan_context_num_sectors: int = DEFAULTS["scan_context_num_sectors"]
    # ring range (m), near the lidar's useful range; 0 = auto-scale from the first scan's extent
    scan_context_max_range_m: float = DEFAULTS["scan_context_max_range_m"]
    scan_context_top_k: int = DEFAULTS["scan_context_top_k"]
    scan_context_match_threshold: float = DEFAULTS["scan_context_match_threshold"]
    scan_context_lidar_height_m: float = DEFAULTS["scan_context_lidar_height_m"]
    # recompute the loop-overlap pressure every this-many-th scan (0 = never)
    pressure_check_frequency: int = DEFAULTS["pressure_check_frequency"]
    # display-only soft_pressure is 50% at this log10 loop hazard, log-compressed to 0% soft_pressure_decades below
    soft_pressure_half_log10_hazard: float = DEFAULTS["soft_pressure_half_log10_hazard"]
    soft_pressure_decades: float = DEFAULTS["soft_pressure_decades"]
    # reduced pose graph for long runs: a keyframe past the frontier rides an older pass's keyframe within place_radius_m
    graph_simplify: bool = DEFAULTS["graph_simplify"]
    graph_simplify_frontier_keyframes: int = DEFAULTS["graph_simplify_frontier_keyframes"]
    # a keyframe revisits a place only if the kept keyframe there is at least this many keyframes older
    graph_simplify_pass_gap_keyframes: int = DEFAULTS["graph_simplify_pass_gap_keyframes"]
    graph_simplify_place_radius_m: float = DEFAULTS["graph_simplify_place_radius_m"]
    graph_simplify_localized_keyframes: int = DEFAULTS["graph_simplify_localized_keyframes"]
    graph_simplify_closures_per_pair: int = DEFAULTS["graph_simplify_closures_per_pair"]
    # one sd of how far a removed keyframe's frozen offset may be off: edges and closures through one carry this much more variance
    graph_simplify_offset_sd_m: float = DEFAULTS["graph_simplify_offset_sd_m"]
    graph_simplify_offset_sd_rad: float = DEFAULTS["graph_simplify_offset_sd_rad"]
    # odometry folds between consecutive kept keyframes (else it follows the nodes removed keyframes ride)
    graph_simplify_chain_odometry: bool = DEFAULTS["graph_simplify_chain_odometry"]
    # past this much folded odometry (m) the walk rides the current place's node instead
    graph_simplify_chain_fold_max_m: float = DEFAULTS["graph_simplify_chain_fold_max_m"]

    # k in the yank gate: confidence must clear 1 - exp(-z^2/2k^2), z = yank in robust std devs of yanks so far
    closure_yank_gate_width: float = DEFAULTS["closure_yank_gate_width"]
    # max ratio of true drift to the stated odometry error estimate, averaged over by the yank gate (1 = trust it)
    closure_yank_drift_range: float = DEFAULTS["closure_yank_drift_range"]
    # skip an extension candidate a full stack of scans could not carry
    closure_extension_skip_unreachable: bool = DEFAULTS["closure_extension_skip_unreachable"]
    # evidence / required (yank units): <= barely_fail reject, >= barely_pass accept, else extend (fail < 1 < pass)
    barely_fail_uncertainty_range: float = DEFAULTS["barely_fail_uncertainty_range"]
    barely_pass_uncertainty_range: float = DEFAULTS["barely_pass_uncertainty_range"]
    # uncertainty range narrows over extension steps, slowed by 1 / (1 - pressure)^this
    decisive_pressure_rate: float = DEFAULTS["decisive_pressure_rate"]
    # odometry distance between an extension's source clouds, as a fraction of mean scan reach
    extension_cloud_spacing: float = DEFAULTS["extension_cloud_spacing"]
    # extension score = median cloud ratio x exp(this (1 - min(d, 5)^2)) per step, d = step disagreement (0 = median)
    compounding_agreement_sensitivity: float = DEFAULTS["compounding_agreement_sensitivity"]
    # adopt a solve only while heat <= threshold; adopting adds its largest move's yank above floor; decay per keyframe
    graph_change_heat_enabled: bool = DEFAULTS["graph_change_heat_enabled"]
    graph_change_heat_threshold: float = DEFAULTS["graph_change_heat_threshold"]
    graph_change_heat_floor: float = DEFAULTS["graph_change_heat_floor"]
    graph_change_heat_decay: float = DEFAULTS["graph_change_heat_decay"]
    # seed a solve from the accepted graph (when it fits) and optionally from the last adopted GNC verdicts
    solve_seed_from_accepted: bool = DEFAULTS["solve_seed_from_accepted"]
    solve_seed_gnc_verdicts: bool = DEFAULTS["solve_seed_gnc_verdicts"]
    # solver fast path: a recent closure is applied at once by local least squares until the GNC solve supersedes it
    solver_fast_path_enabled: bool = DEFAULTS["solver_fast_path_enabled"]
    solver_fast_path_max_keyframes: int = DEFAULTS["solver_fast_path_max_keyframes"]
    # each of its Levenberg-Marquardt passes gives up (declines) after this many steps, bounding its cost estimate
    solver_fast_path_max_lm_steps: int = DEFAULTS["solver_fast_path_max_lm_steps"]
    # closure search pipeline: SC buffer (most similar first) -> fast-match workers -> verification buffer -> workers
    fast_match_workers: int = DEFAULTS["fast_match_workers"]
    sc_buffer_size: int = DEFAULTS["sc_buffer_size"]
    verification_workers: int = DEFAULTS["verification_workers"]
    verification_buffer_slots: int = DEFAULTS["verification_buffer_slots"]
    # recent speed = median over the newest this many distinct-pose steps
    solve_speed_window_poses: int = DEFAULTS["solve_speed_window_poses"]
    # write each ICP-scored candidate to <dir>/<source>_<target>.icp ("" = off)
    closure_icp_dump_dir: str = DEFAULTS["closure_icp_dump_dir"]
    # re-check a weak match on the last 2, 4, 8, ... (up to this many) keyframes (0 = off)
    closure_extended_icp_max_keyframes: int = DEFAULTS["closure_extended_icp_max_keyframes"]
    # closure evidence = per-point likelihood ratio, instead of scan-context x ICP x degeneracy
    closure_likelihood_confidence: bool = DEFAULTS["closure_likelihood_confidence"]
    # after a rejection, try the next scan-context match (up to 1 + floor(sqrt(min(ceil(1/(1-pressure)), top_k-1))))
    closure_alternative_places: bool = DEFAULTS["closure_alternative_places"]
    # with closure_likelihood_confidence: ignore source points no target scan saw
    closure_likelihood_visibility: bool = DEFAULTS["closure_likelihood_visibility"]
    # learn the odometry noise scale from kept closures (max likelihood)
    learned_odometry_error_estimate_from_closures: bool = DEFAULTS["learned_odometry_error_estimate_from_closures"]
    # noise learner reads only pairwise-consistent (PCM core) kept closures
    odometry_error_estimate_evidence_core: bool = DEFAULTS["odometry_error_estimate_evidence_core"]
    # "scan_context": place recognition proposes closures; "inputs_only": only landmarks / world positions (plus landmark-seeded ICP)
    closure_sources: Literal["scan_context", "inputs_only"] = DEFAULTS["closure_sources"]
    # with inputs_only: run that landmark-seeded ICP closure (false = landmarks and world positions alone)
    landmark_closure_icp: bool = DEFAULTS["landmark_closure_icp"]
    # 2D: odometry and every factor count only as x, y, yaw; the solve holds z, roll and pitch at 0
    planar_pose_graph: bool = DEFAULTS["planar_pose_graph"]
    # landmark warm start: first solve with odometry xy variances times this (<= 1 = off) so a re-sighting can pull a loop shut
    landmark_warm_start_xy_variance_scale: float = DEFAULTS["landmark_warm_start_xy_variance_scale"]
    # ...then xy stiffens stage by stage (scale divided by this each time) so every stage's LM converges
    landmark_warm_start_xy_variance_ladder: float = DEFAULTS["landmark_warm_start_xy_variance_ladder"]
    # closure-verification ICP: iteration cap and correspondence gate (m)
    closure_icp_max_iterations: int = DEFAULTS["closure_icp_max_iterations"]
    closure_icp_max_correspondence_m: float = DEFAULTS["closure_icp_max_correspondence_m"]

    # GNC: var_scale loosens closure variance while deciding; inlier_probability sets the chi-squared cutoff
    closure_gnc_var_scale: float = DEFAULTS["closure_gnc_var_scale"]
    closure_gnc_inlier_probability: float = DEFAULTS["closure_gnc_inlier_probability"]
    # Huber threshold on kept closures' whitened residuals in the final solve (0 = plain least squares)
    closure_robust_huber_k: float = DEFAULTS["closure_robust_huber_k"]

    # reject closures the graph is already surer of than ICP
    closure_uninformative_gate: bool = DEFAULTS["closure_uninformative_gate"]
    # odometry drift std dev per sqrt(m travelled) / sqrt(rad turned); None = lidar_range's Mid-360 calibration
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
    # LidarRangeConfig: window, blends per window, and the Go2 L1 and Mid-360 range calibrations; keys not given keep the defaults
    lidar_range: dict[str, Any] = Field(default_factory=lambda: dict(DEFAULTS["lidar_range"]))
    # loops' worth of evidence the stated estimate counts as in the learner (0 = closures alone, inf = never learn)
    learned_odometry_error_estimate_stiffness: float = DEFAULTS["learned_odometry_error_estimate_stiffness"]

    # depth of each scan queue: waiting for tf drops the oldest, waiting for the solver keeps only the newest (<= 0 = unbounded)
    max_scan_queue: int = 5
    # rayon threads for closure search and solves (0 = every core)
    worker_threads: int = 0

    @model_validator(mode="after")
    def _fill_defaults(self) -> PGOConfig:
        self.deterministic_config = {**DEFAULTS["deterministic_config"], **self.deterministic_config}
        self.lidar_range = {**DEFAULTS["lidar_range"], **self.lidar_range}
        given = {name: getattr(self, name) for name in ODOMETRY_ERROR_ESTIMATE_FIELDS}
        if self.odometry_error_estimate_from_lidar_range is None:
            self.odometry_error_estimate_from_lidar_range = all(
                value is None for value in given.values()
            )
        mid360 = DEFAULTS["lidar_range"]["mid360_odometry_error_estimate"]
        for index, (name, value) in enumerate(given.items()):
            if value is None:
                setattr(self, name, mid360[index // 2][index % 2])
        return self


class GscPGO(NativeModule):
    """Pose graph optimization with loop closure (Rust)."""

    config: PGOConfig

    # any frame tf connects to body_frame
    cloud: In[PointCloud2]
    # a sensor's position in a fixed world frame (GPS, mocap): header.frame_id = world frame, child_frame_id = mount
    coordinate_factor: In[Odometry]
    # landmark sighting (TagSightings): frame_id = sensor, to_id = landmark; a repeated constraint_instance_id replaces it
    landmark_factor: In[LocationConstraint]
    corrected_odometry: Out[Odometry]
    # in: odom_frame -> body_frame and the scans' frames; out: the map_frame -> odom_frame correction, identity until the first one
    tf: IO[TFMessage]
    # a packed record per keyframe: x, y, z, node_id, ts, qx, qy, qz, qw
    pose_graph: Out[PointCloud2]
    # a packed record per keyframe a solve moved: its pose before, then the delta (post = delta * pre)
    loop_closure_event: Out[PointCloud2]
    # closure candidate lifecycle events as packed records, for drawing closure edges
    closure_edges: Out[PointCloud2]
    # JSON lines tagged "kind" (pressure, closure, keyframe, solve, odometry_drift) and "ts"
    debug_log: Out[String]
