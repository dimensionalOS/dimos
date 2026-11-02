// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! The pose graph GSC keeps (`GscPgo`'s solve side): started on a background thread whenever closures, fixes or sightings
//! arrive, adopted when it finishes, its verdicts and noise evidence fed back.

use std::thread::JoinHandle;

use crate::big_picture_fit::{
    closure_edges, odometry_edges, solve, solve_log_path, solve_with_priors, LandmarkSighting,
    PositionPrior, Solution, SolveConfig,
};
use crate::gsc_pgo::{ClosureEdgeState, CommittedClosure, GscPgo};
use crate::mat3::{self, Mat3, Vec3};

// ---- the pose graph GSC keeps, re-solved on a background thread whenever closures arrive ----

/// A pose-graph solve running on another thread over the first `keyframes` keyframes (all but keyframe 0 free) and the
/// first `closures_dispatched` committed closures, adopted once it finishes and its hold has passed (`GscPgo::update`)
/// or when waited for (`GscPgo::wait_for_due_solve`, `GscPgo::wait_for_solve`).
pub(crate) struct Solving {
    pub(crate) handle: JoinHandle<(Solution, f64, Vec<u64>)>,
    pub(crate) keyframes: usize,
    pub(crate) schedule: SolveSchedule,
    /// What the solve was handed, only when `GSC_SOLVE_LOG` is set.
    pub(crate) dispatched: Option<SolveDispatch>,
}

/// A solve's inputs for the solve log.
pub(crate) struct SolveDispatch {
    keyframe_time: f64,
    wall_time: f64,
    /// Closures in this solve, and in the solve before it (the rest are new to this one).
    closures: usize,
    closures_before: usize,
}

/// When a solve was dispatched, its estimated cost and when it was seen finished, in scans (`solve_hold_enabled`), and
/// the recorded times of its dispatch scan and of the scan its estimate ends at (once reached).
#[derive(Clone, Copy)]
pub(crate) struct SolveSchedule {
    dispatch_scan: usize,
    estimate_scans: usize,
    finished_scan: Option<usize>,
    dispatch_time: f64,
    pub(crate) due_time: Option<f64>,
    /// It took longer than the data time its estimate spanned (live, it would have been adopted late).
    overshoot: bool,
}

impl SolveSchedule {
    pub(crate) fn due_scan(&self) -> usize {
        self.dispatch_scan + self.estimate_scans
    }
}

fn wall_time() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(f64::NAN, |d| d.as_secs_f64())
}

/// FNV-1a over the first `count` closures' indices and ends, to diff a live solve's input against an offline one.
fn closures_hash(count: usize, closures: &[CommittedClosure]) -> u64 {
    let mut hash = 0xcbf29ce484222325u64;
    for (index, closure) in closures.iter().enumerate().take(count) {
        for value in [
            index as u64,
            closure.source_id as u64,
            closure.target_id as u64,
        ] {
            for byte in value.to_le_bytes() {
                hash = (hash ^ byte as u64).wrapping_mul(0x100000001b3);
            }
        }
    }
    hash
}

/// A float as JSON (NaN and infinities as null).
fn json_number(value: f64) -> String {
    if value.is_finite() {
        format!("{value}")
    } else {
        "null".into()
    }
}

impl GscPgo {
    /// Half the sum of squared whitened residuals of the newest applied solve (how much its
    /// constraints conflict).
    pub fn graph_error(&self) -> f64 {
        self.graph_error
    }
    pub fn committed_closures(&self) -> &[CommittedClosure] {
        &self.committed_closures
    }
    /// Apply the running solve if it has finished and its hold has passed (`solve_hold_enabled`), and start one when
    /// closures have arrived since the last. Never blocks. Returns whether keyframes moved.
    pub fn update(&mut self) -> bool {
        self.track_solve();
        let hold = self.config.solve_hold_enabled;
        let scans = self.scans_seen;
        let ready = self.solving.as_ref().is_some_and(|solving| {
            solving.schedule.finished_scan.is_some()
                && (!hold || scans >= solving.schedule.due_scan())
        });
        self.adopt(ready)
    }
    /// `update`, but waiting for the running solve once its hold has passed, so a replay gives the same graph on any
    /// machine (with the hold off, plain `update`).
    pub fn wait_for_due_solve(&mut self) -> bool {
        self.track_solve();
        let due = self.config.solve_hold_enabled
            && self
                .solving
                .as_ref()
                .is_some_and(|solving| self.scans_seen >= solving.schedule.due_scan());
        if due {
            self.adopt(true)
        } else {
            self.update()
        }
    }
    /// `update`, but waiting for the running solve to finish first, whatever its hold.
    pub fn wait_for_solve(&mut self) -> bool {
        self.adopt(true)
    }
    /// Solves that took longer than the data time their estimate spanned (`SolveSchedule::overshoot`).
    pub fn solve_overshoots(&self) -> usize {
        self.solve_overshoots
    }
    /// Note the scan the running solve was first seen finished at.
    fn track_solve(&mut self) {
        let scans = self.scans_seen;
        if let Some(solving) = self.solving.as_mut() {
            if solving.schedule.finished_scan.is_none() && solving.handle.is_finished() {
                solving.schedule.finished_scan = Some(scans);
            }
        }
    }
    /// An upper bound on a solve's cost in scans over `keyframes` keyframes and every committed closure, from the
    /// newest adopted solve's rejections and the body's recent speed (`solve_hold_enabled`).
    fn solve_estimate_scans(&self, keyframes: usize) -> usize {
        let config = &self.config;
        let closures = self.committed_closures.len() as f64;
        // a closure no solve has judged yet has a NaN weight, so it is not counted
        let rejected = self
            .committed_closures
            .iter()
            .filter(|c| c.gnc_weight <= 0.5)
            .count() as f64;
        let speed = self
            .recent_speed()
            .max(config.solve_estimate_speed_floor_mps);
        (config.solve_estimate_scale
            * (keyframes as f64).powf(config.solve_estimate_keyframe_power)
            * (1.0 + closures).powf(config.solve_estimate_closure_power)
            * (1.0 + rejected).powf(config.solve_estimate_rejected_power)
            * speed.powf(config.solve_estimate_speed_power))
        .ceil() as usize
    }
    /// The running solve's keyframe count and free keyframe count when it started, if one is running.
    pub fn solve_started(&self) -> Option<(usize, usize)> {
        self.solving
            .as_ref()
            .map(|s| (s.keyframes, s.keyframes - 1))
    }
    /// Call `solved` (from the solve's thread) whenever a solve finishes, so a caller can sleep until then.
    pub fn notify_when_solved(&mut self, solved: impl Fn() + Send + Sync + 'static) {
        self.solved = Some(std::sync::Arc::new(solved));
    }
    fn adopt(&mut self, ready: bool) -> bool {
        self.new_closure = false;
        let moved = ready && self.apply_solve();
        if self.solving.is_none() && self.unsolved() {
            self.dispatch_solve();
        }
        moved
    }
    /// Whether a closure or a world position arrived after the newest solve started.
    fn unsolved(&self) -> bool {
        // nothing to solve before the first keyframe (a fix or sighting can arrive before it)
        !self.key_poses.is_empty()
            && (self.committed_closures.len() > self.closures_dispatched
                || self.world_positions.len() > self.world_positions_dispatched
                || self.landmark_sightings.len() > self.landmark_sightings_dispatched
                || self.odometry_error_estimate_rescale)
    }
    /// A measured position of a sensor in fixed world frame `frame` at `time`: the sensor sits at `lever` in the body frame
    /// (its mount, from tf), `position` and `covariance` (m²) are in `frame`. It joins the next solve as a prior on the
    /// keyframe nearest in time (`solve_with_priors`).
    pub fn add_coordinate_factor(
        &mut self,
        time: f64,
        frame: u32,
        lever: Vec3,
        position: Vec3,
        covariance: Mat3,
    ) {
        self.world_positions
            .push((time, frame, lever, position, covariance));
    }
    /// A landmark `landmark` (any id the caller keeps for it: a tag's number, a matched feature, another robot's
    /// landmark) seen at `time` with pose (`rotation`, `position`) in the body frame (through the sensor's mount), with
    /// that pose's 6x6 `information` (`LandmarkSighting`'s order; zero in an unknown dimension). It joins the next solve
    /// as a sighting from the keyframe nearest in time.
    pub fn add_landmark_factor(
        &mut self,
        time: f64,
        landmark: u64,
        rotation: Mat3,
        position: Vec3,
        information: [[f64; 6]; 6],
    ) {
        self.landmark_sightings
            .push((time, landmark, rotation, position, information));
    }
    /// `add_landmark_factor` for a point: `position` with its 3x3 `covariance` (m²; `bearing_and_range` makes one from a
    /// bearing and a rough range), the rotation unknown.
    pub fn add_landmark_point(
        &mut self,
        time: f64,
        landmark: u64,
        position: Vec3,
        covariance: Mat3,
    ) {
        let point = LandmarkSighting::point(0, 0, position, covariance);
        self.add_landmark_factor(time, landmark, point.rotation, position, point.information);
    }
    /// The newest solve's landmarks: id, map rotation (identity for a point) and map position.
    pub fn landmarks(&self) -> &[(u64, Mat3, Vec3)] {
        &self.landmarks
    }
    /// The newest solve's map -> frame alignments (frame id, yaw about z, translation), one per world frame its
    /// positions made observable.
    pub fn world_alignment(&self) -> &[(u32, f64, Vec3)] {
        &self.world_alignment
    }
    /// Each world position as a prior on the keyframe nearest its time, its covariance grown by how far the body moves in
    /// that time gap (the keyframes' own speed there, squared, in each horizontal axis and up); fixes before the first or
    /// after the last keyframe are left for later.
    fn position_priors(&self) -> Vec<PositionPrior> {
        self.world_positions
            .iter()
            .filter_map(|&(time, frame, lever, world, covariance)| {
                let (keyframe, moved) = self.nearest_keyframe(time)?;
                let mut grown = covariance;
                for (axis, row) in grown.iter_mut().enumerate() {
                    row[axis] += moved;
                }
                Some(PositionPrior {
                    keyframe,
                    frame,
                    lever,
                    world,
                    covariance: grown,
                })
            })
            .collect()
    }
    /// The keyframe nearest `time` and how far (squared, m²) the body moves in that gap at the keyframes' own speed there;
    /// None before the first or after the last keyframe (left for later).
    fn nearest_keyframe(&self, time: f64) -> Option<(usize, f64)> {
        let n = self.key_poses.len();
        let time_of = |k: usize| self.key_poses[k].time;
        if n < 2 || time < time_of(0) || time > time_of(n - 1) {
            return None;
        }
        let after = self
            .key_poses
            .partition_point(|k| k.time < time)
            .clamp(1, n - 1);
        let keyframe = if time - time_of(after - 1) <= time_of(after) - time {
            after - 1
        } else {
            after
        };
        let gap = (time_of(after) - time_of(after - 1)).max(f64::EPSILON);
        let speed = mat3::norm(&mat3::sub(
            &self.key_poses[after].translation_local,
            &self.key_poses[after - 1].translation_local,
        )) / gap;
        Some((keyframe, (speed * (time - time_of(keyframe)).abs()).powi(2)))
    }
    /// Each landmark sighting from the keyframe nearest its time, its known position variances grown by how far the body
    /// moves in that gap; landmarks numbered densely in order of first sighting, with their ids.
    fn sightings(&self) -> (Vec<LandmarkSighting>, Vec<u64>) {
        let mut ids: Vec<u64> = Vec::new();
        let sightings = self
            .landmark_sightings
            .iter()
            .filter_map(|&(time, id, rotation, position, information)| {
                let (keyframe, moved) = self.nearest_keyframe(time)?;
                let landmark = ids.iter().position(|&i| i == id).unwrap_or_else(|| {
                    ids.push(id);
                    ids.len() - 1
                });
                let mut grown = crate::landmarks::known_inverse(&information);
                for axis in 3..6 {
                    if grown[axis][axis] > 0.0 {
                        grown[axis][axis] += moved;
                    }
                }
                Some(LandmarkSighting {
                    keyframe,
                    landmark,
                    rotation,
                    position,
                    information: crate::landmarks::known_inverse(&grown),
                })
            })
            .collect();
        (sightings, ids)
    }
    /// Whether the newest adopted solve measured the learned odometry error estimate (`learned_evidence_requested`).
    pub fn learned_odometry_error_estimate_measured(&self) -> bool {
        self.learned_evidence_measured
    }
    /// Wait for the running solve, and solve again until every closure is in; then measure the learned odometry error
    /// estimate once (`learned_odometry_error_estimate_from_closures`). Returns whether keyframes moved.
    pub fn finish(&mut self) -> bool {
        // a closure search still running when the data ends lands now (`closure_search_hold_enabled`)
        self.land_remaining_closure_search();
        let mut moved = self.settle();
        // the learned odometry error estimate, for the next run's config: measured here from the final kept loops (one more
        // solve of the same graph with the evidence on), and during the run only after a solve keeps a longer loop
        if self.config.learned_odometry_error_estimate_from_closures
            && self.committed_closures.iter().any(|c| c.gnc_weight > 0.5)
        {
            self.learned_evidence_requested = true;
            self.odometry_error_estimate_rescale = true;
            moved |= self.settle();
        }
        // the final graph is the solver's last output, whatever the graph change heat
        let keyframes = self.key_poses.len();
        let heat = self.heat_at(keyframes);
        moved |= self.adopt_pending_solve(keyframes, heat);
        moved
    }
    /// Adopt solves until every closure, position and sighting has been through one.
    fn settle(&mut self) -> bool {
        let mut moved = false;
        loop {
            moved |= self.apply_solve();
            if !self.unsolved() {
                return moved;
            }
            self.dispatch_solve();
        }
    }
    /// Start one solve of every committed closure from the odometry, judging each closure afresh (GNC must start from the
    /// dead-reckoned chain, not an estimate earlier closures already bent).
    fn dispatch_solve(&mut self) {
        let n = self.key_poses.len();
        let config = &self.config;
        let closures = closure_edges(self.committed_closures.iter());
        // keyframe 0 pins the map; the rest start from the odometry, riding its correction
        let free: Vec<bool> = (0..n).map(|k| k > 0).collect();
        let pinned = &self.key_poses[0];
        let offset_rotation = mat3::mat_mul(
            &pinned.rotation_global,
            &mat3::transpose(&pinned.rotation_local),
        );
        let offset_translation = mat3::sub(
            &pinned.translation_global,
            &mat3::mat_vec(&offset_rotation, &pinned.translation_local),
        );
        let (mut rotations, mut positions) = (Vec::with_capacity(n), Vec::with_capacity(n));
        rotations.push(pinned.rotation_global);
        positions.push(pinned.translation_global);
        for key_pose in &self.key_poses[1..] {
            rotations.push(mat3::mat_mul(&offset_rotation, &key_pose.rotation_local));
            positions.push(mat3::add(
                &mat3::mat_vec(&offset_rotation, &key_pose.translation_local),
                &offset_translation,
            ));
        }
        // every odometry edge and closure has an end past keyframe 0, so all of them are in
        let solve_error_estimate = self.odometry_error_estimate();
        let odometry = odometry_edges(&self.key_poses, config, solve_error_estimate);
        let priors = self.position_priors();
        let (sightings, landmark_ids) = self.sightings();
        let solve_config = SolveConfig {
            gnc_variance_scale: config.closure_gnc_var_scale,
            gnc_inlier_probability: config.closure_gnc_inlier_probability,
            huber_k: config.closure_robust_huber_k,
            evidence_core: config.odometry_error_estimate_evidence_core,
            // the learned estimate's evidence only in `finish`'s last solve
            learned_evidence: config.learned_odometry_error_estimate_from_closures
                && self.learned_evidence_requested,
        };
        let solved = self.solved.clone();
        let closures_before = self.closures_dispatched;
        let handle = std::thread::Builder::new()
            .name("gsc-pose-graph".into())
            .spawn(move || {
                let started = std::time::Instant::now();
                let solution = if priors.is_empty() && sightings.is_empty() {
                    solve(
                        &rotations,
                        &positions,
                        &free,
                        &odometry,
                        &closures,
                        &solve_config,
                    )
                } else {
                    solve_with_priors(
                        &rotations,
                        &positions,
                        &free,
                        &odometry,
                        &closures,
                        &priors,
                        &sightings,
                        &solve_config,
                    )
                };
                let seconds = started.elapsed().as_secs_f64();
                if let Some(solved) = solved {
                    solved();
                }
                (solution, seconds, landmark_ids)
            })
            .expect("spawn pose graph thread");
        // a solve started only because the noise moved (nothing new since the last) is the one re-solve that evidence gets
        self.odometry_error_estimate_rescaled = self.odometry_error_estimate_rescale
            && self.committed_closures.len() == self.closures_dispatched
            && self.world_positions.len() == self.world_positions_dispatched
            && self.landmark_sightings.len() == self.landmark_sightings_dispatched;
        self.closures_dispatched = self.committed_closures.len();
        self.world_positions_dispatched = self.world_positions.len();
        self.landmark_sightings_dispatched = self.landmark_sightings.len();
        // the ln the residuals are whitened at (the solve's noise, its lower bound or the estimate)
        self.learned_odometry_error_estimate_ln_dispatched =
            self.odometry_error_estimate().map(f64::ln);
        self.odometry_error_estimate_rescale = false;
        let dispatched = solve_log_path().map(|_| SolveDispatch {
            keyframe_time: self.key_poses[n - 1].time,
            wall_time: wall_time(),
            closures: self.closures_dispatched,
            closures_before,
        });
        self.solving = Some(Solving {
            handle,
            keyframes: n,
            schedule: SolveSchedule {
                dispatch_scan: self.scans_seen,
                estimate_scans: self.solve_estimate_scans(n),
                finished_scan: None,
                dispatch_time: self.scan_time,
                due_time: None,
                overshoot: false,
            },
            dispatched,
        });
    }
    /// Wait for the running solve and adopt it: its poses, the correction the keyframes since then
    /// ride on, and its closure verdicts. Returns whether there was one.
    fn apply_solve(&mut self) -> bool {
        let Some(solving) = self.solving.take() else {
            return false;
        };
        let (solution, seconds, landmark_ids) =
            solving.handle.join().expect("pose graph solve panicked");
        // a solve waited for was finished at the scan it was waited at; one slower than the data its estimate spanned (a
        // replay outrunning real time waits on others too) overshot: the estimate was not an upper bound
        let schedule = SolveSchedule {
            finished_scan: Some(solving.schedule.finished_scan.unwrap_or(self.scans_seen)),
            overshoot: solving
                .schedule
                .due_time
                .is_some_and(|due| seconds > due - solving.schedule.dispatch_time),
            ..solving.schedule
        };
        if schedule.overshoot {
            self.solve_overshoots += 1;
            eprintln!(
                "gsc_pgo: SOLVE OVERSHOOT #{}: the solve over {} keyframes took {seconds:.2}s, past the {:.2}s of data its estimate of {} scans spanned",
                self.solve_overshoots,
                solving.keyframes,
                schedule.due_time.unwrap_or(f64::NAN) - schedule.dispatch_time,
                schedule.estimate_scans
            );
        }
        self.timing.solve_s += seconds;
        let dispatched_over = solving.keyframes;
        self.solve_records.push((
            dispatched_over,
            self.key_poses[dispatched_over - 1].time,
            seconds,
        ));
        // a solve that stalled short of a minimum would bend the map wrong: the map keeps the previous solve (a real loop
        // closure can raise the cost a hundredfold, so the cost itself is no test)
        let previous = self.graph_error;
        if !solution.converged {
            let reason = "lm_not_converged";
            if let Some(dispatched) = &solving.dispatched {
                self.log_rejected_solve(
                    solving.keyframes,
                    dispatched,
                    &schedule,
                    &solution,
                    reason,
                    previous,
                    seconds,
                );
            }
            return false;
        }
        let finished = FinishedSolve {
            solution,
            seconds,
            landmark_ids,
            keyframes: solving.keyframes,
            schedule,
            dispatched: solving.dispatched,
            arrived_keyframes: self.key_poses.len(),
        };
        if !self.config.graph_change_heat_enabled {
            return self.adopt_solution(finished, None);
        }
        // the newest finished solve is the one waiting; an older one still waiting is superseded
        let arrived = self.key_poses.len();
        let heat = self.heat_at(arrived);
        if let Some(older) = self.pending_solve.replace(finished) {
            self.log_heat_event("superseded", &older, heat);
        }
        if self.release_pending_solve(arrived) {
            return true;
        }
        if let Some(waiting) = &self.pending_solve {
            self.log_heat_event("held", waiting, heat);
        }
        false
    }
    /// Adopt a finished solve: its poses, the correction the keyframes since then ride on, and its closure verdicts;
    /// `heat`, the graph change heat before it, its cost and after it (for the solve log). Returns true.
    pub(crate) fn adopt_solution(
        &mut self,
        finished: FinishedSolve,
        heat: Option<(f64, f64, f64)>,
    ) -> bool {
        let FinishedSolve {
            solution,
            seconds,
            landmark_ids,
            keyframes,
            schedule,
            dispatched,
            arrived_keyframes,
        } = finished;
        self.graph_version += 1;
        if !landmark_ids.is_empty() {
            self.landmarks = landmark_ids
                .into_iter()
                .zip(solution.landmark_rotations.iter().zip(&solution.landmarks))
                .map(|(id, (&r, &p))| (id, r, p))
                .collect();
        }
        if !solution.world_alignment.is_empty() {
            self.world_alignment = solution.world_alignment.clone();
        }
        let n = keyframes;
        // the largest move any keyframe makes from the previous map to this solve's (only when logging)
        let largest_move = dispatched.as_ref().map(|_| {
            self.key_poses[..n]
                .iter()
                .zip(&solution.positions)
                .map(|(key_pose, position)| {
                    mat3::norm(&mat3::sub(position, &key_pose.translation_global))
                })
                .enumerate()
                .fold(
                    (0, 0.0),
                    |best, (k, d)| if d > best.1 { (k, d) } else { best },
                )
        });
        let previous_kept: Vec<bool> = if dispatched.is_some() {
            self.committed_closures
                .iter()
                .map(|c| c.gnc_weight > 0.5)
                .collect()
        } else {
            Vec::new()
        };
        self.solved_relative_covariance =
            Some((n - 1, solution.relative_position_covariance.clone()))
                .filter(|(_, covariance)| covariance.len() == n);
        for (key_pose, (rotation, position)) in self.key_poses[..n]
            .iter_mut()
            .zip(solution.rotations.iter().zip(&solution.positions))
        {
            key_pose.rotation_global = *rotation;
            key_pose.translation_global = *position;
        }
        // keyframes made since the solve started ride on its newest keyframe's correction
        let newest = &self.key_poses[n - 1];
        self.rotation_offset = mat3::mat_mul(
            &newest.rotation_global,
            &mat3::transpose(&newest.rotation_local),
        );
        self.translation_offset = mat3::sub(
            &newest.translation_global,
            &mat3::mat_vec(&self.rotation_offset, &newest.translation_local),
        );
        for key_pose in &mut self.key_poses[n..] {
            key_pose.rotation_global =
                mat3::mat_mul(&self.rotation_offset, &key_pose.rotation_local);
            key_pose.translation_global = mat3::add(
                &mat3::mat_vec(&self.rotation_offset, &key_pose.translation_local),
                &self.translation_offset,
            );
        }
        // the solve judged every closure dispatched to it; one committed while it ran was not in it, so it stays unjudged
        for (index, weight) in solution.closure_weights.iter().copied().enumerate() {
            let previous = self.committed_closures[index].gnc_weight;
            self.committed_closures[index].gnc_weight = weight;
            if previous == weight {
                continue;
            }
            let (source, target) = (
                self.committed_closures[index].source_id,
                self.committed_closures[index].target_id,
            );
            // a closure committed directly (`commit_closure`, no search) has no search confidence or yank recorded
            let (confidence, yank_stdevs) = (
                self.closure_fit_confidence
                    .get(index)
                    .copied()
                    .unwrap_or(f64::NAN),
                self.closure_yank_stdevs
                    .get(index)
                    .copied()
                    .unwrap_or(f64::NAN),
            );
            if weight > 0.5 {
                self.push_edge(
                    source,
                    target,
                    ClosureEdgeState::Accepted,
                    confidence,
                    yank_stdevs,
                );
                self.push_edge(
                    source,
                    target,
                    ClosureEdgeState::Applied,
                    confidence,
                    yank_stdevs,
                );
            } else {
                self.push_edge(source, target, ClosureEdgeState::Rejected, 0.0, yank_stdevs);
            }
        }
        self.graph_error = solution.error;
        if let (Some(dispatched), Some((largest_move_keyframe, largest_move))) =
            (&dispatched, largest_move)
        {
            self.log_solve(
                n,
                dispatched,
                &schedule,
                &solution,
                &previous_kept,
                largest_move_keyframe,
                largest_move,
                seconds,
                &heat_fields(arrived_keyframes, heat),
            );
        }
        self.update_learned_odometry_error_estimate(solution.odometry_error_estimate_evidence);
        self.learned_evidence_measured = solution.odometry_error_estimate_evidence.is_some();
        if self.learned_evidence_measured {
            self.learned_evidence_requested = false;
        }
        // a loop longer than any kept before is new information on the odometry's error: measure the learned estimate
        // with the next solve (logged only, never a decision)
        let longest = self
            .committed_closures
            .iter()
            .filter(|c| c.gnc_weight > 0.5)
            .map(|c| c.source_id.abs_diff(c.target_id))
            .max()
            .unwrap_or(0);
        if longest > self.learned_evidence_gap {
            self.learned_evidence_gap = longest;
            self.learned_evidence_requested = true;
        }
        // `GSC_LEARN_LOG=<path>`: one JSON line per adopted solve (keyframes, kept closure indices, learned odometry error estimate,
        // and the solve's evidence per source as [sum, redundancy]), for tracing the noise learner against ground truth offline
        if let Ok(path) = std::env::var("GSC_LEARN_LOG") {
            use std::io::Write;
            let kept: Vec<usize> = self
                .committed_closures
                .iter()
                .enumerate()
                .filter(|(_, c)| c.gnc_weight > 0.5)
                .map(|(i, _)| i)
                .collect();
            let line = format!(
                "{{\"keyframes\":{},\"committed\":{},\"kept\":{:?},\"noise\":{:?},\"evidence\":{:?}}}\n",
                n,
                self.committed_closures.len(),
                kept,
                self.learned_odometry_error_estimate(),
                solution
                    .odometry_error_estimate_evidence
                    .map(|e| e.map(|(sum, r)| [sum, r]))
                    .unwrap_or([[f64::NAN; 2]; 6])
            );
            if let Ok(mut file) = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(path)
            {
                let _ = file.write_all(line.as_bytes());
            }
        }
        true
    }
    /// One `GSC_SOLVE_LOG` line for an adopted solve (`solve_log_path`).
    #[allow(clippy::too_many_arguments)]
    fn log_solve(
        &self,
        n: usize,
        dispatched: &SolveDispatch,
        schedule: &SolveSchedule,
        solution: &Solution,
        previous_kept: &[bool],
        largest_move_keyframe: usize,
        largest_move: f64,
        seconds: f64,
        heat: &str,
    ) {
        use std::fmt::Write as _;
        let Some(trace) = &solution.trace else {
            return;
        };
        let weights = &trace.gnc_weights;
        // final GNC weights in [0, 1e-4), [1e-4, .25), [.25, .5), [.5, .75), [.75, 1 - 1e-4), [1 - 1e-4, 1]
        let mut histogram = [0usize; 6];
        for &w in weights {
            let bin = [1e-4, 0.25, 0.5, 0.75, 1.0 - 1e-4]
                .iter()
                .take_while(|&&edge| w >= edge)
                .count();
            histogram[bin] += 1;
        }
        // closures new to this solve, or kept before and rejected now (or the reverse)
        let mut changed = String::new();
        for (index, &weight) in weights.iter().enumerate() {
            let was = previous_kept.get(index).copied();
            let kept = solution.closure_weights[index] > 0.5;
            let new = index >= dispatched.closures_before;
            if new || was != Some(kept) {
                let closure = &self.committed_closures[index];
                let _ = write!(
                    changed,
                    "{}{{\"index\":{index},\"source\":{},\"target\":{},\"weight\":{},\"kept\":{kept},\"was\":\"{}\"}}",
                    if changed.is_empty() { "" } else { "," },
                    closure.source_id,
                    closure.target_id,
                    json_number(weight),
                    if new { "new" } else if was == Some(true) { "kept" } else { "rejected" },
                );
            }
        }
        let line = format!(
            "{{{}{heat},\"weight_histogram\":{:?},\"largest_move_m\":{},\"largest_move_keyframe\":{largest_move_keyframe},\"changed\":[{changed}]}}\n",
            self.solve_fields(n, dispatched, schedule, solution, seconds),
            histogram,
            json_number(largest_move),
        );
        append_solve_log(&line);
    }
    /// A `GSC_SOLVE_LOG` line for a solve that was not adopted (its last Levenberg-Marquardt pass did not converge), with
    /// why and the previous adopted solve's final cost.
    fn log_rejected_solve(
        &self,
        n: usize,
        dispatched: &SolveDispatch,
        schedule: &SolveSchedule,
        solution: &Solution,
        reason: &str,
        previous_error: f64,
        seconds: f64,
    ) {
        let line = format!(
            "{{\"rejected_solve\":\"{reason}\",{},\"error\":{},\"previous_error\":{}}}\n",
            self.solve_fields(n, dispatched, schedule, solution, seconds),
            json_number(solution.error),
            json_number(previous_error),
        );
        append_solve_log(&line);
    }
    /// The solve log fields every line has: when it was dispatched and finished, its schedule in scans (the scan count
    /// now as `adopted_scan`, the data seconds its estimate spanned once reached), its closures, and its GNC's path.
    fn solve_fields(
        &self,
        n: usize,
        dispatched: &SolveDispatch,
        schedule: &SolveSchedule,
        solution: &Solution,
        seconds: f64,
    ) -> String {
        let kept = solution
            .closure_weights
            .iter()
            .filter(|&&w| w > 0.5)
            .count();
        let trace = solution.trace.as_ref();
        format!(
            "\"dispatch_keyframes\":{},\"dispatch_time\":{},\"dispatch_wall\":{},\"adopt_keyframes\":{},\"adopt_time\":{},\"adopt_wall\":{},\"solve_s\":{},\"estimate_scans\":{},\"estimate_data_s\":{},\"dispatch_scan\":{},\"finished_scan\":{},\"adopted_scan\":{},\"overshoot\":{},\"closures\":{},\"closures_new\":{},\"closures_hash\":\"{:016x}\",\"kept\":{kept},\"converged\":{},\"gnc_iterations\":{},\"gnc_mu\":{},\"gnc_stop\":\"{}\",\"initial_cost\":[{}],\"final_cost\":[{}]",
            n,
            json_number(dispatched.keyframe_time),
            json_number(dispatched.wall_time),
            self.key_poses.len(),
            json_number(self.key_poses.last().map_or(f64::NAN, |k| k.time)),
            json_number(wall_time()),
            json_number(seconds),
            schedule.estimate_scans,
            json_number(schedule.due_time.map_or(f64::NAN, |due| due - schedule.dispatch_time)),
            schedule.dispatch_scan,
            schedule.finished_scan.map_or("null".into(), |scan| scan.to_string()),
            self.scans_seen,
            schedule.overshoot,
            dispatched.closures,
            dispatched.closures - dispatched.closures_before.min(dispatched.closures),
            closures_hash(dispatched.closures, &self.committed_closures),
            solution.converged,
            trace.map_or(0, |t| t.gnc_iterations),
            json_number(trace.map_or(f64::NAN, |t| t.gnc_mu)),
            trace.map_or("", |t| t.gnc_stop),
            trace.map_or([f64::NAN; 3], |t| t.initial_cost).map(json_number).join(","),
            trace.map_or([f64::NAN; 3], |t| t.final_cost).map(json_number).join(","),
        )
    }
}

/// A finished solve's result and what it was dispatched over, waiting to be adopted (`graph_change_heat_enabled`).
pub(crate) struct FinishedSolve {
    solution: Solution,
    seconds: f64,
    landmark_ids: Vec<u64>,
    keyframes: usize,
    schedule: SolveSchedule,
    dispatched: Option<SolveDispatch>,
    /// The keyframe count when it finished.
    arrived_keyframes: usize,
}

/// The solve log's graph change heat fields for an adopted solve: when it finished, and the heat before, its cost and
/// the heat after (none with the heat off).
fn heat_fields(arrived_keyframes: usize, heat: Option<(f64, f64, f64)>) -> String {
    match heat {
        Some((before, cost, after)) => format!(
            ",\"arrived_keyframes\":{arrived_keyframes},\"heat_before\":{},\"heat_cost\":{},\"heat_after\":{}",
            json_number(before),
            json_number(cost),
            json_number(after)
        ),
        None => String::new(),
    }
}

impl GscPgo {
    /// The graph change heat decayed to keyframe count `keyframes` (`graph_change_heat_decay` per keyframe).
    pub(crate) fn heat_at(&self, keyframes: usize) -> f64 {
        self.heat
            * self
                .config
                .graph_change_heat_decay
                .powi(keyframes.saturating_sub(self.heat_keyframes) as i32)
    }
    /// Adopt the waiting solve if the heat at keyframe count `keyframes` is at or under `graph_change_heat_threshold`,
    /// adding its cost to the heat. Returns whether it was adopted.
    pub(crate) fn release_pending_solve(&mut self, keyframes: usize) -> bool {
        if self.pending_solve.is_none() {
            return false;
        }
        let heat = self.heat_at(keyframes);
        if heat > self.config.graph_change_heat_threshold {
            return false;
        }
        self.adopt_pending_solve(keyframes, heat)
    }
    /// Adopt the waiting solve whatever the heat (`finish`, so the final graph is the solver's last output).
    pub(crate) fn adopt_pending_solve(&mut self, keyframes: usize, heat: f64) -> bool {
        let Some(waiting) = self.pending_solve.take() else {
            return false;
        };
        let cost = self.graph_change_cost(&waiting.solution, waiting.keyframes);
        self.heat = heat + cost;
        self.heat_keyframes = keyframes;
        self.adopt_solution(waiting, Some((heat, cost, self.heat)))
    }
    /// What adopting `solution` (solved over the first `solved` keyframes) costs in graph change heat: the largest
    /// keyframe move from the accepted graph to the new one (the keyframes after `solved` riding its newest's
    /// correction), in standard deviations of the stated odometry error accumulated along the path from the start (xy,
    /// z and yaw combined; 1 m of travel at least), curbed (`curbed_yank`, drift range `closure_yank_drift_range`),
    /// above `graph_change_heat_floor`.
    fn graph_change_cost(&self, solution: &Solution, solved: usize) -> f64 {
        let config = &self.config;
        let newest = &self.key_poses[solved - 1];
        let ride_rotation = mat3::mat_mul(
            &solution.rotations[solved - 1],
            &mat3::transpose(&newest.rotation_local),
        );
        let ride_translation = mat3::sub(
            &solution.positions[solved - 1],
            &mat3::mat_vec(&ride_rotation, &newest.translation_local),
        );
        let yaw = |r: &Mat3| r[1][0].atan2(r[0][0]);
        let (mut travelled, mut turned, mut largest) = (0.0, 0.0, 0.0f64);
        for (k, key_pose) in self.key_poses.iter().enumerate() {
            if k > 0 {
                let before = &self.key_poses[k - 1];
                travelled += mat3::norm(&mat3::sub(
                    &key_pose.translation_local,
                    &before.translation_local,
                ));
                turned += mat3::angular_distance(&key_pose.rotation_local, &before.rotation_local);
            }
            let spread = |per_sqrt_m: f64, per_sqrt_rad: f64| {
                (per_sqrt_m * per_sqrt_m * (travelled + 1.0) + per_sqrt_rad * per_sqrt_rad * turned)
                    .sqrt()
            };
            let (rotation, position) = if k < solved {
                (solution.rotations[k], solution.positions[k])
            } else {
                (
                    mat3::mat_mul(&ride_rotation, &key_pose.rotation_local),
                    mat3::add(
                        &mat3::mat_vec(&ride_rotation, &key_pose.translation_local),
                        &ride_translation,
                    ),
                )
            };
            let moved = mat3::sub(&position, &key_pose.translation_global);
            let turn = (yaw(&rotation) - yaw(&key_pose.rotation_global) + std::f64::consts::PI)
                .rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI;
            let z = ((moved[0].hypot(moved[1])
                / spread(
                    config.odometry_error_estimate_xy_per_sqrt_m,
                    config.odometry_error_estimate_xy_per_sqrt_rad,
                ))
            .powi(2)
                + (moved[2]
                    / spread(
                        config.odometry_error_estimate_z_per_sqrt_m,
                        config.odometry_error_estimate_z_per_sqrt_rad,
                    ))
                .powi(2)
                + (turn
                    / spread(
                        config.odometry_error_estimate_yaw_per_sqrt_m,
                        config.odometry_error_estimate_yaw_per_sqrt_rad,
                    ))
                .powi(2))
            .sqrt();
            largest = largest.max(z);
        }
        (curbed_yank(largest, config.closure_yank_drift_range) - config.graph_change_heat_floor)
            .max(0.0)
    }
    /// A `GSC_SOLVE_LOG` line for a solve the graph change heat held or superseded, with the heat then.
    fn log_heat_event(&self, event: &str, finished: &FinishedSolve, heat: f64) {
        let Some(dispatched) = &finished.dispatched else {
            return;
        };
        let line = format!(
            "{{\"{event}_solve\":true,{},\"arrived_keyframes\":{},\"heat\":{},\"heat_threshold\":{}}}\n",
            self.solve_fields(
                finished.keyframes,
                dispatched,
                &finished.schedule,
                &finished.solution,
                finished.seconds
            ),
            finished.arrived_keyframes,
            json_number(heat),
            json_number(self.config.graph_change_heat_threshold),
        );
        append_solve_log(&line);
    }
}

/// A yank's cost in yank units when the odometry may drift up to `drift_range` times its stated error estimate: about
/// z^2 / 2 for a small yank, about 3 ln z for a large one (`proportional_confidence::yank_evidence`), summed in log
/// space so a yank far past the range keeps growing instead of overflowing.
fn curbed_yank(z: f64, drift_range: f64) -> f64 {
    const STEPS: usize = 64;
    let top = drift_range.ln();
    let log_sum = |z: f64| {
        let terms: Vec<f64> = (0..=STEPS)
            .map(|i| {
                let ln_s = top * i as f64 / STEPS as f64;
                let weight: f64 = if i == 0 || i == STEPS { 0.5 } else { 1.0 };
                weight.ln() - 3.0 * ln_s - z * z / (2.0 * (2.0 * ln_s).exp())
            })
            .collect();
        let most = terms.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        most + terms.iter().map(|t| (t - most).exp()).sum::<f64>().ln()
    };
    log_sum(0.0) - log_sum(z)
}

/// Append one line to the `GSC_SOLVE_LOG` file.
fn append_solve_log(line: &str) {
    let Some(path) = solve_log_path() else {
        return;
    };
    if let Ok(mut file) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
    {
        use std::io::Write;
        let _ = file.write_all(line.as_bytes());
    }
}
