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

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PGOConfig(NativeModuleConfig):
    # "deterministic" (default): every LET job (solve, closure search, verification, pressure) is held to its estimate in
    # deterministic_config, so a replay gives the same graph on any machine; "fast" (a real-life run): every job lands as
    # soon as it finishes, and the run prints the deterministic_config it measured, ready to paste here
    mode: str = "deterministic"
    # a job past its estimate logs a warning at most once per this many seconds of data, with each LET kind's overshoots
    # since the last (a warning only: a slower machine running a faster one's deterministic_config just waits)
    overshoot_warning_interval_s: float = 10.0
    # the deterministic mode's estimates, in scans (a job holds ceil(estimate) scans, at least one; exponents unitless)
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
            "merge_scans_per_keyframe": 2.56e-06,
            "pressure_scans_fixed_overhead": 0.0007777038336858567,
            "pressure_scans_per_keyframe": 8.708997476399469e-06,
            "pressure_scans_per_closure": 1.2578234731596874e-05,
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
    # ICP pairs points only within this distance (m); the default is the old 0.15 m^2 squared cutoff
    closure_icp_inlier_distance_m: float = 0.3872983346207417
    # reject a closure whose ICP rms inlier distance (m) is larger
    closure_icp_max_fitness_m: float = 0.3872983346207417
    # closure ICP minimises gaps along surface normals (point-to-plane); False = point-to-point
    closure_icp_point_to_plane: bool = True
    # the single-scan closure ICP aligns against its target voxelized at this (m, fitness still on the full target; 0 = full)
    closure_icp_target_resolution_m: float = 0.15
    # the single-scan closure ICP gives up once its fitness can no longer reach the gate's largest passing fitness
    closure_icp_early_stop: bool = True
    # alternative places are retried only at or above this pressure (below it, only the best match is tried)
    closure_alternative_places_min_pressure: float = 0.8
    # Drop the points a scan re-sends from the scan before (Go2 L1 republishes its last seconds) from every keyframe cloud (on by default)
    drop_reseen_points: bool = True
    # A keyframe's cloud needs at least this many points: scans are gathered and a due keyframe waits until they reach it (0 = off)
    auto_aggregate_cloud_size: int = 2000
    # Closure target submap half range = ceil(source clouds x (1 + extra_fraction)) + extra_keyframes (5 for the single scan)
    closure_target_extra_fraction: float = 0.3
    closure_target_extra_keyframes: int = 3
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
    # The pressure off the scan thread: a job on one worker (newest request pending), landing at dispatch + estimate
    # (scans: fixed overhead + per keyframe + per committed closure, rounded up); off = inline on the scan thread
    # soft_pressure (display only: the same probability without the cliff) is 50% where log10 of the loop hazard
    # (pressure = 1 - e^-hazard) is soft_pressure_half_log10_hazard, falling log-compressed to 0% soft_pressure_decades below
    soft_pressure_half_log10_hazard: float = -0.5
    soft_pressure_decades: float = 100.0

    # A closure's match confidence (scan-context x ICP x degeneracy, each normalized by its own gate) must clear
    # 1 - exp(-z^2/2k^2), z = how hard it yanks the graph (per sqrt keyframe gap) in robust std devs of the yanks scored so far; this is k
    closure_yank_gate_width: float = 1.0
    # How many times the stated odometry error estimate (sd) the true drift may be; the yank gate averages over it (1 = trust it)
    closure_yank_drift_range: float = 100.0
    # Extension stops: skip a candidate a full stack could not carry, stop when a step's evidence stops rising, stop when
    # a step's fit leaves the single fit's place
    closure_extension_skip_unreachable: bool = True
    # Reject before ICP a candidate whose least reachable yank needs more evidence than any stack of scans could give
    closure_yank_pre_check: bool = False
    closure_extension_stop_when_flat: bool = True
    closure_extension_stop_on_swing: bool = True
    # Uncertainty range as evidence / required evidence (yank units): at or above barely_pass accept, at or below barely_fail
    # reject, extend the match in between (barely_fail < 1 < barely_pass)
    barely_fail_uncertainty_range: float = 0.05
    barely_pass_uncertainty_range: float = 3.0
    # The uncertainty range narrows over the extension's steps, slowed by 1 / (1 - pressure)^this
    decisive_pressure_rate: float = 1.0
    # Odometry distance between an extension's separate source clouds, as a fraction of the mean scan reach
    extension_cloud_spacing: float = 0.25
    # Compounding agreement sensitivity: the yank gate scores an extension by its median cloud ratio times
    # exp(sensitivity (1 - min(d, 5)^2)) per step, d = how far that step's alignment is from the one before it at the
    # source keyframe, in inlier distances (0 = plain median)
    compounding_agreement_sensitivity: float = 1.5
    # Graph change heat: a finished solve is adopted only while the heat <= threshold; adopting adds the curbed yank of the
    # largest keyframe move it makes (stated odometry error sigmas along the path) above the floor; the heat decays by
    # decay per keyframe; a solve finishing while hot waits (a newer one replaces it), always adopted at the run's end
    graph_change_heat_enabled: bool = True
    graph_change_heat_threshold: float = 30.0
    graph_change_heat_floor: float = 20.0
    graph_change_heat_decay: float = 0.995
    # Deterministic solve schedule: a finished solve is held ceil(solve_scans_scale * keyframes^solve_keyframe_power *
    # (1 + closures)^solve_closure_power * (1 + rejected)^solve_rejected_power + debounce_scans_per_keyframe *
    # keyframes) scans after its dispatch (an upper bound on its cost in scans; rejected = closures the newest adopted
    # solve rejected; debounce = the graph change heat's cost its thread measures); one still running then is an
    # overshoot, logged and adopted as it finishes
    # Deterministic closure-search schedule: a search's result lands only once ceil(its estimate) scans have passed since
    # its dispatch: the fast match (fast_match_scans_fixed_overhead + per keyframe, the scan-context lookup, + per point,
    # source and target, + per source point) at dispatch, then the verification's fixed overhead as the extension
    # starts and one round's scans before each round (verification_scans_*)
    # The closure search's pipeline: SC candidates and retries wait in a priority buffer (most similar first, least
    # dropped when full) for fast_match_workers (cloud build, single ICP, gates); a match inside the uncertainty range
    # waits in a priority buffer (highest single-scan confidence ratio first, lowest dropped) for verification_workers to
    # extend it, each round held to its own estimate
    fast_match_workers: int = 2
    sc_buffer_size: int = 4
    verification_workers: int = 2
    verification_buffer_slots: int = 2
    # The body's recent speed (solve estimate, closure diagnostics): median over the newest this many distinct-pose steps
    solve_speed_window_poses: int = 500
    # Write every ICP-scored candidate's source cloud, target submap and ICP result to <dir>/<source>_<target>.icp
    closure_icp_dump_dir: str = ""
    # Re-check a too-weak match on the last 2, 4, 8, ... (up to this many) keyframes, seeded by the first ICP. 0 = off
    closure_extended_icp_max_keyframes: int = 16
    # Above 0: score a closure's fit by its sharpness s (share within submap_resolution, times how much overlap falls when
    # nudged closure_sharpness_nudge_m / _deg) as s / (s + this), extended stacks by their own sharpness; 0 = fitness
    closure_sharpness_half: float = 0.0
    closure_sharpness_nudge_m: float = 1.0
    closure_sharpness_nudge_deg: float = 5.0
    # True: a closure's evidence is its likelihood ratio (mean log ratio per point: aligned half-normal of a width measured
    # from the clouds vs uniform out to the ICP match distance), in place of scan-context x ICP x degeneracy
    closure_likelihood_confidence: bool = True
    # True: a keyframe whose closure search is rejected tries its next scan-context match, up to
    # 1 + floor(sqrt(min(ceil(1/(1 - pressure)), scan_context_top_k - 1))) tries (3 at pressure 0.8, 4 from 0.9 with top_k 10)
    closure_alternative_places: bool = True
    # True (with closure_likelihood_confidence): source points no target scan saw count for nothing, not as misfits
    closure_likelihood_visibility: bool = True
    # True: scale the odometry's reported noise by the factor the kept closures' disagreement with it implies (max likelihood)
    learned_odometry_error_estimate_from_closures: bool = True
    # True: the noise learner reads only the kept closures that agree pairwise (PCM core) on every solve
    odometry_error_estimate_evidence_core: bool = True
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

    # True: a closure is uninformative only where the graph's own covariance of its ends is surer than ICP in every direction
    closure_informative_by_graph_covariance: bool = True
    # False: never reject a closure as uninformative (the graph already surer of its ends than ICP)
    closure_uninformative_gate: bool = True
    # Odometry drift as a random walk in the motion (independent of keyframe spacing): per source, the standard deviation
    # it adds per sqrt(metre travelled) and per sqrt(radian turned); translation in m, rotation in rad. Defaults: Mid-360 +
    # FAST-LIO/Point-LIO fitted to ground truth on five recordings (drift is almost all from turning). After each solve
    # that keeps loops, debug_log (kind "odometry_drift") gives the values the run measured, to start the next run from.
    odometry_error_estimate_xy_per_sqrt_m: float = 3.91e-4
    odometry_error_estimate_xy_per_sqrt_rad: float = 6.35e-3
    odometry_error_estimate_z_per_sqrt_m: float = 6.42e-4
    odometry_error_estimate_z_per_sqrt_rad: float = 3.86e-3
    odometry_error_estimate_yaw_per_sqrt_m: float = 1e-6
    odometry_error_estimate_yaw_per_sqrt_rad: float = 8.66e-3
    odometry_error_estimate_tilt_per_sqrt_m: float = 6.91e-4
    odometry_error_estimate_tilt_per_sqrt_rad: float = 3.3e-3
    # spread (sd of ln) of each source's true size across recordings: the online estimate's prior; 0 = one closure's worth
    learned_odometry_error_estimate_prior_ln_sd_xy: float = 1.72
    learned_odometry_error_estimate_prior_ln_sd_z: float = 2.05
    learned_odometry_error_estimate_prior_ln_sd_yaw: float = 1.15
    learned_odometry_error_estimate_prior_ln_sd_tilt: float = 0.96
    # how many closed loops' worth of evidence the stated odometry error estimate counts as for the learner (its plasticity); 0 = the closures alone, inf = never learned (the config's drift is used)
    learned_odometry_error_estimate_stiffness: float = 10.0

    # usually que is 1 lidar frame, only goes up after a compute spike (backpressure); past
    # this every queued scan but the newest is dropped, so a stall can't leave the module stale
    max_scan_queue: int = 5
    # threads closure search and solves may use (the rayon pool): 0 (default) = every core, so GSC never runs itself
    # handicapped; set a number to leave cores to the rest of the robot where measurements show it pays
    worker_threads: int = 0


class GscPGO(NativeModule):
    """Pose graph optimization with loop closure — Rust port of gsc_pgo."""

    config: PGOConfig

    cloud: In[PointCloud2]
    odometry: In[Odometry]
    # a sensor's position in a fixed world frame (GPS, mocap): header.frame_id = world frame, child_frame_id = mount
    coordinate_factor: In[Odometry]
    # a landmark seen by a sensor: header.frame_id = sensor, child_frame_id = landmark name, pose + 6x6 covariance in the sensor frame (unknown dims: -1)
    landmark_factor: In[Odometry]
    corrected_odometry: Out[Odometry]
    # the map -> odometry correction, identity until the first one
    tf: IO[TFMessage]
    # a packed record per keyframe: x, y, z, node_id, ts, qx, qy, qz, qw
    pose_graph: Out[PointCloud2]
    # a packed record per keyframe a solve moved: its pose before, then the delta (post = delta * pre)
    loop_closure_event: Out[PointCloud2]
    # each closure candidate's lifecycle (proposed, gated, accepted, rejected, applied) as packed records
    # with both keyframes' current positions and a confidence, for drawing closure edges
    closure_edges: Out[PointCloud2]
    # debugging numbers as JSON lines, each with "kind" and "ts", for graphing: "pressure" (every pressure_check_frequency-th
    # scan: unresolved_loop_probability, the chance the newest scan overlaps an earlier stretch no closure has tied it to),
    # "closure" (every closure search: each gate's inputs and the outcome), "keyframe" (scan queue, running threads, stage
    # seconds), "solve" (each adopted solve: closures committed / judged / kept, graph error)
    debug_log: Out[String]

    @rpc
    def stop(self) -> None:
        super().stop()
