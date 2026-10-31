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
    closure_edges, odometry_edges, solve, solve_with_priors, LandmarkSighting, PositionPrior,
    Solution, SolveConfig,
};
use crate::gsc_pgo::{ClosureEdgeState, CommittedClosure, GscPgo};
use crate::mat3::{self, Mat3, Vec3};

// ---- the pose graph GSC keeps, re-solved on a background thread whenever closures arrive ----

/// A pose-graph solve running on another thread over the first `keyframes` keyframes (all but keyframe 0 free) and the
/// first `closures_dispatched` committed closures, adopted once it finishes (`GscPgo::update`) or when waited for
/// (`GscPgo::wait_for_solve`).
pub(crate) struct Solving {
    pub(crate) handle: JoinHandle<(Solution, f64, Vec<u64>)>,
    pub(crate) keyframes: usize,
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
    /// Apply the running solve if it has finished, and start one when closures have arrived since the
    /// last. Never blocks. Returns whether keyframes moved.
    pub fn update(&mut self) -> bool {
        let finished = self
            .solving
            .as_ref()
            .is_some_and(|solving| solving.handle.is_finished());
        self.adopt(finished)
    }
    /// `update`, but waiting for the running solve to finish first.
    pub fn wait_for_solve(&mut self) -> bool {
        self.adopt(true)
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
                || self.odometry_noise_rescale)
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
    /// Wait for the running solve, and solve again until every closure is in. Returns whether keyframes moved.
    pub fn finish(&mut self) -> bool {
        let mut moved = false;
        loop {
            moved |= self.apply_solve();
            if !self.unsolved() {
                return moved;
            }
            self.dispatch_solve();
        }
    }
    /// Start a solve of every committed closure from the odometry (GNC must start from the dead-reckoned chain,
    /// not an estimate earlier closures already bent).
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
        let solve_noise = self.odometry_noise();
        let odometry = odometry_edges(&self.key_poses, config, solve_noise);
        let priors = self.position_priors();
        let (sightings, landmark_ids) = self.sightings();
        let solve_config = SolveConfig {
            gnc_variance_scale: config.closure_gnc_var_scale,
            gnc_inlier_probability: config.closure_gnc_inlier_probability,
            huber_k: config.closure_robust_huber_k,
            evidence_core: config.odometry_noise_evidence_core,
            committed: if config.closure_commitments {
                self.committed_closures
                    .iter()
                    .map(|c| c.gnc_weight > 0.5)
                    .collect()
            } else {
                Vec::new()
            },
        };
        let solved = self.solved.clone();
        let handle = std::thread::Builder::new()
            .name("gsc-pose-graph".into())
            .spawn(move || {
                let started = std::time::Instant::now();
                let run = |solve_config: &SolveConfig| {
                    if priors.is_empty() && sightings.is_empty() {
                        solve(
                            &rotations,
                            &positions,
                            &free,
                            &odometry,
                            &closures,
                            solve_config,
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
                            solve_config,
                        )
                    }
                };
                let held = run(&solve_config);
                // the committed closures stand unless a cold solve finds a larger group that rejects one of them
                let solution = if solve_config.committed.iter().any(|&c| c) {
                    let cold = run(&SolveConfig {
                        committed: Vec::new(),
                        ..solve_config.clone()
                    });
                    let count =
                        |s: &Solution| s.closure_weights.iter().filter(|&&w| w > 0.5).count();
                    let contradicts = solve_config
                        .committed
                        .iter()
                        .zip(&cold.closure_weights)
                        .any(|(&c, &w)| c && w < 0.5);
                    if count(&cold) > count(&held) && contradicts {
                        cold
                    } else {
                        held
                    }
                } else {
                    held
                };
                let seconds = started.elapsed().as_secs_f64();
                if let Some(solved) = solved {
                    solved();
                }
                (solution, seconds, landmark_ids)
            })
            .expect("spawn pose graph thread");
        // a solve started only because the noise moved (nothing new since the last) is the one re-solve that evidence gets
        self.odometry_noise_rescaled = self.odometry_noise_rescale
            && self.committed_closures.len() == self.closures_dispatched
            && self.world_positions.len() == self.world_positions_dispatched
            && self.landmark_sightings.len() == self.landmark_sightings_dispatched;
        self.closures_dispatched = self.committed_closures.len();
        self.world_positions_dispatched = self.world_positions.len();
        self.landmark_sightings_dispatched = self.landmark_sightings.len();
        // the ln the residuals are whitened at (the solve's noise, its lower bound or the estimate)
        self.odometry_noise_ln_dispatched = self.odometry_noise().map(f64::ln);
        self.odometry_noise_rescale = false;
        self.solving = Some(Solving {
            handle,
            keyframes: n,
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
        if !landmark_ids.is_empty() {
            self.landmarks = landmark_ids
                .into_iter()
                .zip(solution.landmark_rotations.iter().zip(&solution.landmarks))
                .map(|(id, (&r, &p))| (id, r, p))
                .collect();
        }
        self.timing.solve_s += seconds;
        if !solution.world_alignment.is_empty() {
            self.world_alignment = solution.world_alignment.clone();
        }
        let n = solving.keyframes;
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
        self.update_odometry_noise_scale(solution.odometry_noise_evidence);
        // `GSC_LEARN_LOG=<path>`: one JSON line per adopted solve (keyframes, kept closure indices, learned noise,
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
                self.odometry_noise_estimate(),
                solution
                    .odometry_noise_evidence
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
}
