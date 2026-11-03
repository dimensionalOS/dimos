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
use crate::gsc_pgo::{ClosureEdgeState, GscPgo};
use crate::mat3::{self, Mat3, Vec3};

// ---- the pose graph GSC keeps, re-solved on a background thread whenever closures arrive ----

/// A background pose-graph solve over the first `keyframes` keyframes (all but 0 free) and `closures_dispatched` closures.
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

/// When a solve was dispatched, its estimated cost and when it was seen finished (scans).
#[derive(Clone, Copy)]
pub(crate) struct SolveSchedule {
    dispatch_scan: usize,
    estimate_scans: usize,
    /// The estimate before rounding up (scans).
    estimate_raw_scans: f64,
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
fn closures_hash(count: usize, closures: &crate::store::Closures) -> u64 {
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
    /// Half the sum of squared whitened residuals of the newest applied solve (how much its constraints conflict).
    pub fn graph_error(&self) -> f64 {
        self.graph_error
    }
    pub fn committed_closures(&self) -> &crate::store::Closures {
        &self.committed_closures
    }
    /// Non-blocking: adopt a finished solve past its hold, start one if due; true if moved.
    pub fn update(&mut self) -> bool {
        self.track_solve();
        let hold = self.config.deterministic();
        let scans = self.scans_seen;
        let ready = self.solving.as_ref().is_some_and(|solving| {
            solving.schedule.finished_scan.is_some()
                && (!hold || scans >= solving.schedule.due_scan())
        });
        self.adopt(ready)
    }
    /// `update`, but waiting for the running solve once its hold passes, so a replay gives the same graph on any machine.
    pub fn wait_for_due_solve(&mut self) -> bool {
        self.track_solve();
        let due = self.config.deterministic()
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
    /// Upper bound on a solve's cost in scans (raw, held) from its node count and closures (committed indices) and rejected count.
    fn solve_estimate_scans(&self, keyframes: usize, closure_index: &[usize]) -> (f64, usize) {
        let config = &self.config;
        let closures = closure_index.len() as f64;
        // a closure no solve has judged yet has a NaN weight, so it is not counted
        let rejected = closure_index
            .iter()
            .filter(|&&i| self.committed_closures.weight(i) <= 0.5)
            .count() as f64;
        let estimate_raw_scans = config.deterministic_config.solve_scans_scale
            * (keyframes as f64).powf(config.deterministic_config.solve_keyframe_power)
            * (1.0 + closures).powf(config.deterministic_config.solve_closure_power)
            * (1.0 + rejected).powf(config.deterministic_config.solve_rejected_power);
        (estimate_raw_scans, crate::closure_search_schedule::hold_scans(estimate_raw_scans))
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
        let mut moved = std::mem::take(&mut self.adopted_since_update) | self.step_adoption(false);
        moved |= ready && self.apply_solve();
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
                || self.evidence_solve_due)
    }
    /// A sensor's position in world frame `frame` at `time` (mount `lever`, covariance m²): a prior on the nearest keyframe.
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
    /// Landmark `landmark` (any caller id) seen at `time`: body-frame pose and 6x6 `information` (0 = unknown dimension).
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
    /// The newest solve's landmarks: id, map rotation (identity for a point) and map position.
    pub fn landmarks(&self) -> &[(u64, Mat3, Vec3)] {
        &self.landmarks
    }
    /// The newest solve's map -> frame alignments (frame id, yaw about z, translation), one per observable world frame.
    pub fn world_alignment(&self) -> &[(u32, f64, Vec3)] {
        &self.world_alignment
    }
    /// World positions as priors on their nearest keyframes, covariance grown by the gap's motion; out-of-range ones wait.
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
    /// The keyframe nearest `time` and the body's squared motion (m²) over that gap; None outside the keyframes' time span.
    fn nearest_keyframe(&self, time: f64) -> Option<(usize, f64)> {
        let n = self.key_poses.len();
        let time_of = |k: usize| self.key_poses.fixed(k).time;
        if n < 2 || time < time_of(0) || time > time_of(n - 1) {
            return None;
        }
        // the first keyframe at or after `time` (keyframe times rise), a binary search
        let (mut lo, mut hi) = (0, n);
        while lo < hi {
            let mid = (lo + hi) / 2;
            if time_of(mid) < time {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        let after = lo.clamp(1, n - 1);
        let keyframe = if time - time_of(after - 1) <= time_of(after) - time {
            after - 1
        } else {
            after
        };
        let gap = (time_of(after) - time_of(after - 1)).max(f64::EPSILON);
        let speed = mat3::norm(&mat3::sub(
            &self.key_poses.fixed(after).translation_local,
            &self.key_poses.fixed(after - 1).translation_local,
        )) / gap;
        Some((keyframe, (speed * (time - time_of(keyframe)).abs()).powi(2)))
    }
    /// Sightings from their nearest keyframes (variance grown by the gap's motion), landmarks numbered densely, with ids.
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
                for (axis, row) in grown.iter_mut().enumerate().skip(3) {
                    if row[axis] > 0.0 {
                        row[axis] += moved;
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
    /// Solve until every closure is in, then measure the learned estimate (`learned_odometry_error_estimate_from_closures`).
    pub fn finish(&mut self) -> bool {
        // a closure search still running lands now; from here on the debounce and merge run inline
        self.land_remaining_closure_search();
        let mut moved = false;
        while self.adoption_busy() {
            moved |= self.step_adoption(true);
        }
        self.adoption_inline = true;
        moved |= self.settle();
        // the learned estimate for the next run's config: one more solve of the final graph with the evidence on
        if self.config.learned_odometry_error_estimate_from_closures
            && self.committed_closures.iter().any(|c| c.gnc_weight > 0.5)
        {
            self.learned_evidence_requested = true;
            self.evidence_solve_due = true;
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
    /// Start one solve of every committed closure from the odometry, so GNC judges each afresh from the dead-reckoned chain.
    fn dispatch_solve(&mut self) {
        let n = self.key_poses.len();
        let config = &self.config;
        let closure_count = self.committed_closures.len();
        // simplified (`graph_simplify`): the solve's nodes are the kept keyframes then the frontier; else every keyframe
        let simplified = self.simplifying();
        let node_keyframes: Vec<usize> = if simplified {
            let simplify = &self.simplify;
            (0..simplify.kept.len())
                .map(|i| simplify.kept[i])
                .chain(simplify.decided()..n)
                .collect()
        } else {
            (0..n).collect()
        };
        let m = node_keyframes.len();
        let (closure_index, closures): (Vec<usize>, Vec<_>) = if simplified {
            self.simplify
                .closures(&self.committed_closures, closure_count, config)
                .into_iter()
                .unzip()
        } else {
            ((0..closure_count).collect(), closure_edges(self.committed_closures.iter()))
        };
        // keyframe 0 pins the map; the rest start from the odometry, riding its correction
        let free: Vec<bool> = (0..m).map(|k| k > 0).collect();
        let pinned = &self.key_poses.get(0);
        let offset_rotation = mat3::mat_mul(
            &pinned.rotation_global,
            &mat3::transpose(&pinned.rotation_local),
        );
        let offset_translation = mat3::sub(
            &pinned.translation_global,
            &mat3::mat_vec(&offset_rotation, &pinned.translation_local),
        );
        let (mut rotations, mut positions) = (Vec::with_capacity(m), Vec::with_capacity(m));
        rotations.push(pinned.rotation_global);
        positions.push(pinned.translation_global);
        for &k in &node_keyframes[1..] {
            let key_pose = self.key_poses.fixed(k);
            rotations.push(mat3::mat_mul(&offset_rotation, &key_pose.rotation_local));
            positions.push(mat3::add(
                &mat3::mat_vec(&offset_rotation, &key_pose.translation_local),
                &offset_translation,
            ));
        }
        // every odometry edge and closure has an end past keyframe 0, so all of them are in
        let solve_error_estimate = self.odometry_error_estimate();
        let (odometry, priors, sightings, landmark_ids) = if simplified {
            let odometry = self
                .simplify
                .odometry(&self.key_poses, n)
                .iter()
                .map(|e| e.edge(config, solve_error_estimate))
                .collect();
            // a prior on a removed keyframe moves to its node, its lever carried through the offset
            let priors = self
                .position_priors()
                .into_iter()
                .map(|prior| {
                    let (node, offset) = self.simplify.place(prior.keyframe);
                    PositionPrior {
                        keyframe: node,
                        lever: mat3::add(&mat3::mat_vec(&offset.0, &prior.lever), &offset.1),
                        ..prior
                    }
                })
                .collect();
            // a sighting moves to its keyframe's node only when that keyframe is the node (its information is body-frame)
            let (sightings, landmark_ids) = self.sightings();
            let sightings = sightings
                .into_iter()
                .filter_map(|sighting| {
                    let (node, offset) = self.simplify.place(sighting.keyframe);
                    (offset.1 == [0.0; 3] && offset.0 == mat3::identity())
                        .then_some(LandmarkSighting { keyframe: node, ..sighting })
                })
                .collect();
            (odometry, priors, sightings, landmark_ids)
        } else {
            let (sightings, landmark_ids) = self.sightings();
            (
                odometry_edges(&self.key_poses, config, solve_error_estimate),
                self.position_priors(),
                sightings,
                landmark_ids,
            )
        };
        let solve_config = SolveConfig {
            gnc_variance_scale: config.closure_gnc_var_scale,
            gnc_inlier_probability: config.closure_gnc_inlier_probability,
            huber_k: config.closure_robust_huber_k,
            evidence_core: config.odometry_error_estimate_evidence_core,
            // the learned estimate's evidence only in `finish`'s last solve
            learned_evidence: config.learned_odometry_error_estimate_from_closures
                && self.learned_evidence_requested,
            // or from the accepted graph (newer keyframes riding its correction), where it already fits
            seed_start: if config.solve_seed_from_accepted {
                node_keyframes.iter().map(|&k| self.key_poses.global(k)).collect()
            } else {
                Vec::new()
            },
            seed_closure_weights: if config.solve_seed_gnc_verdicts {
                closure_index.iter().map(|&i| self.committed_closures.weight(i)).collect()
            } else {
                Vec::new()
            },
        };
        let (estimate_raw_scans, estimate_scans) = self.solve_estimate_scans(m, &closure_index);
        // a simplified solve's nodes expand back to every keyframe (and its closures to every committed one) on its thread
        let expansion = simplified.then(|| crate::graph_simplify::Expansion {
            placements: self.simplify.placements.clone(),
            key_poses: self.key_poses.clone(),
            kept: self.simplify.kept.len(),
            keyframes: n,
            closure_index,
            closures: self.committed_closures.clone(),
            closure_count,
        });
        self.simplify.last_solve = (m, closures.len());
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
                let solution = match &expansion {
                    Some(expansion) => expansion.expand(solution),
                    None => solution,
                };
                let seconds = started.elapsed().as_secs_f64();
                if let Some(solved) = solved {
                    solved();
                }
                (solution, seconds, landmark_ids)
            })
            .expect("spawn pose graph thread");
        self.closures_dispatched = self.committed_closures.len();
        self.world_positions_dispatched = self.world_positions.len();
        self.landmark_sightings_dispatched = self.landmark_sightings.len();
        self.evidence_solve_due = false;
        let dispatched = solve_log_path().map(|_| SolveDispatch {
            keyframe_time: self.key_poses.fixed(n - 1).time,
            wall_time: wall_time(),
            closures: self.closures_dispatched,
            closures_before,
        });
        self.solving = Some(Solving {
            handle,
            keyframes: n,
            schedule: SolveSchedule {
                dispatch_scan: self.scans_seen,
                estimate_scans,
                estimate_raw_scans,
                finished_scan: None,
                dispatch_time: self.scan_time,
                due_time: None,
                overshoot: false,
            },
            dispatched,
        });
    }
    /// Wait for the running solve and adopt it (poses, ride correction, closure verdicts). Returns whether there was one.
    fn apply_solve(&mut self) -> bool {
        let Some(solving) = self.solving.take() else {
            return false;
        };
        let joined = std::time::Instant::now();
        let (solution, seconds, landmark_ids) =
            solving.handle.join().expect("pose graph solve panicked");
        self.solve_join_wait_s += joined.elapsed().as_secs_f64();
        // a waited-for solve finished at the waiting scan; one slower than its estimate's data span overshot
        let schedule = SolveSchedule {
            finished_scan: Some(solving.schedule.finished_scan.unwrap_or(self.scans_seen)),
            overshoot: solving
                .schedule
                .due_time
                .is_some_and(|due| seconds > due - solving.schedule.dispatch_time),
            ..solving.schedule
        };
        self.note_overshoot("solve", schedule.overshoot);
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
        self.record_job_ratio("solve", seconds, solving.schedule.estimate_raw_scans);
        let dispatched_over = solving.keyframes;
        self.solve_records.push((
            dispatched_over,
            self.key_poses.fixed(dispatched_over - 1).time,
            seconds,
        ));
        // an unconverged solve keeps the previous map (cost is no test: a real loop can raise it a hundredfold)
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
        // the debounce and the merge are their own jobs (`adoption_schedule`), except at the end of the data
        if !self.adoption_inline {
            self.offer_debounce(finished);
            return false;
        }
        if !self.config.graph_change_heat_enabled {
            return self.adopt_solution(finished, None, None);
        }
        let cost = graph_change_cost_on(&self.key_poses, &self.config, &finished.solution, finished.keyframes);
        self.debounced(finished, cost, true)
    }
    /// Adopt a finished solve; `heat` = graph change heat before, its cost, after (for the solve log). Returns true.
    pub(crate) fn adopt_solution(
        &mut self,
        finished: FinishedSolve,
        heat: Option<(f64, f64, f64)>,
        prepared: Option<crate::adoption_schedule::MergePrepared>,
    ) -> bool {
        let FinishedSolve {
            mut solution,
            seconds,
            landmark_ids,
            keyframes,
            schedule,
            dispatched,
            arrived_keyframes,
            ..
        } = finished;
        self.graph_version += 1;
        self.adopted_solves += 1;
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
            (0..n)
                .zip(&solution.positions)
                .map(|(k, position)| mat3::norm(&mat3::sub(position, &self.key_poses.position(k))))
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
        // moved, not copied: the solve's covariances are not read after this
        self.solved_relative_covariance =
            Some((n - 1, std::sync::Arc::new(std::mem::take(&mut solution.relative_position_covariance))))
                .filter(|(_, covariance)| covariance.len() == n);
        // the solve's poses swapped in; newer keyframes ride its newest keyframe's correction (`KeyFrames::global`)
        let newest = self.key_poses.fixed(n - 1);
        let (newest_rotation, newest_translation) = (solution.rotations[n - 1], solution.positions[n - 1]);
        self.rotation_offset = mat3::mat_mul(
            &newest_rotation,
            &mat3::transpose(&newest.rotation_local),
        );
        self.translation_offset = mat3::sub(
            &newest_translation,
            &mat3::mat_vec(&self.rotation_offset, &newest.translation_local),
        );
        let solved = std::sync::Arc::new(crate::store::SolvedPoses {
            rotations: std::mem::take(&mut solution.rotations),
            positions: std::mem::take(&mut solution.positions),
        });
        self.key_poses.adopt(solved, self.rotation_offset, self.translation_offset);
        // the solve judged every closure dispatched to it; one committed while it ran was not in it, so it stays unjudged
        let changed: Vec<(usize, f64)> = match &prepared {
            Some(prepared) => prepared.changed.clone(),
            None => solution
                .closure_weights
                .iter()
                .copied()
                .enumerate()
                .filter(|&(index, weight)| self.committed_closures.weight(index) != weight)
                .collect(),
        };
        for (index, weight) in changed {
            // a closure committed directly (`commit_closure`, no search) has no search confidence or yank recorded
            let fixed = *self.committed_closures.fixed(index);
            let (source, target) = (fixed.source_id, fixed.target_id);
            let (confidence, yank_stdevs) = (fixed.fit_confidence, fixed.yank_stdevs);
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
        // the solve's closure weights swapped in (its thread built them): a closure committed while it ran stays unjudged
        let evidence = solution.odometry_error_estimate_evidence;
        self.committed_closures
            .adopt(std::sync::Arc::new(std::mem::take(&mut solution.closure_weights)));
        self.simplify_note_adoption();
        self.update_learned_odometry_error_estimate(evidence);
        self.learned_evidence_measured = solution.odometry_error_estimate_evidence.is_some();
        if self.learned_evidence_measured {
            self.learned_evidence_requested = false;
        }
        // a loop longer than any kept before: measure the learned estimate with the next solve (logged only)
        let longest = match &prepared {
            Some(prepared) => prepared.longest_kept,
            None => self
                .committed_closures
                .iter()
                .filter(|c| c.gnc_weight > 0.5)
                .map(|c| c.source_id.abs_diff(c.target_id))
                .max()
                .unwrap_or(0),
        };
        if longest > self.learned_evidence_gap {
            self.learned_evidence_gap = longest;
            self.learned_evidence_requested = true;
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
                let closure = self.committed_closures.get(index);
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
    /// A `GSC_SOLVE_LOG` line for a solve not adopted (LM did not converge), with why and the previous final cost.
    #[allow(clippy::too_many_arguments)]
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
    /// The fields every solve log line has: dispatch/finish times, schedule in scans, closures and the GNC's path.
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
    pub(crate) solution: Solution,
    seconds: f64,
    landmark_ids: Vec<u64>,
    pub(crate) keyframes: usize,
    schedule: SolveSchedule,
    dispatched: Option<SolveDispatch>,
    /// The keyframe count when it finished.
    arrived_keyframes: usize,
}

/// The solve log's graph change heat fields: when it finished, heat before, its cost, heat after (none with the heat off).
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
    /// Adopt the waiting solve if the heat at `keyframes` is <= `graph_change_heat_threshold`, adding its cost.
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
    /// Adopt the waiting solve whatever the heat (`finish`); its cost joins the heat and it goes to the merge.
    pub(crate) fn adopt_pending_solve(&mut self, keyframes: usize, heat: f64) -> bool {
        let Some((waiting, cost)) = self.pending_solve.take() else {
            return false;
        };
        self.heat = heat + cost;
        self.heat_keyframes = keyframes;
        let heat = Some((heat, cost, self.heat));
        if self.adoption_inline {
            return self.adopt_solution(waiting, heat, None);
        }
        self.offer_merge(waiting, heat);
        false
    }
    /// A debounced solve and its heat cost: adopted now if the heat allows, else held as the newest waiting one.
    pub(crate) fn debounced(&mut self, finished: FinishedSolve, cost: f64, inline: bool) -> bool {
        let arrived = self.key_poses.len();
        let heat = self.heat_at(arrived);
        if let Some((older, _)) = self.pending_solve.replace((finished, cost)) {
            self.log_heat_event("superseded", &older, heat);
        }
        if heat <= self.config.graph_change_heat_threshold {
            return self.adopt_pending_solve(arrived, heat) && inline;
        }
        if let Some((waiting, _)) = &self.pending_solve {
            self.log_heat_event("held", waiting, heat);
        }
        false
    }
    /// A `GSC_SOLVE_LOG` line for a solve the graph change heat held or superseded, with the heat then.
    pub(crate) fn log_heat_event(&self, event: &str, finished: &FinishedSolve, heat: f64) {
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

/// Yank cost (yank units) with drift up to `drift_range` x stated error: ~z²/2 small, ~3 ln z large, in log space.
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

/// Graph change heat scan state: odometry travel/turn so far, previous odometry pose, largest move so far (odometry sds).
#[derive(Clone, Default)]
pub(crate) struct ChangeScan {
    travelled: f64,
    turned: f64,
    previous_local: Option<(Mat3, Vec3)>,
    largest: f64,
}

impl ChangeScan {
    /// The next keyframe: its odometry pose, its accepted pose and its new one.
    fn step(
        &mut self,
        config: &crate::gsc_pgo::Config,
        (rotation_local, translation_local): (&Mat3, &Vec3),
        (rotation_global, translation_global): (&Mat3, &Vec3),
        (rotation, position): (Mat3, Vec3),
    ) {
        if let Some((before_rotation, before_translation)) = &self.previous_local {
            self.travelled += mat3::norm(&mat3::sub(translation_local, before_translation));
            self.turned += mat3::angular_distance(rotation_local, before_rotation);
        }
        self.previous_local = Some((*rotation_local, *translation_local));
        let (travelled, turned) = (self.travelled, self.turned);
        let spread = |per_sqrt_m: f64, per_sqrt_rad: f64| {
            (per_sqrt_m * per_sqrt_m * (travelled + 1.0) + per_sqrt_rad * per_sqrt_rad * turned).sqrt()
        };
        let yaw = |r: &Mat3| r[1][0].atan2(r[0][0]);
        let moved = mat3::sub(&position, translation_global);
        let turn = (yaw(&rotation) - yaw(rotation_global) + std::f64::consts::PI)
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
        self.largest = self.largest.max(z);
    }
}

/// Heat cost of adopting `solution` over `keys`: largest move in accumulated-odometry sds, `curbed_yank`, minus the floor.
pub(crate) fn graph_change_cost_on(
    keys: &crate::store::KeyFrames,
    config: &crate::gsc_pgo::Config,
    solution: &Solution,
    solved: usize,
) -> f64 {
    let mut scan = ChangeScan::default();
    for (k, key_pose) in keys.range(0..solved).enumerate() {
        scan.step(
            config,
            (&key_pose.rotation_local, &key_pose.translation_local),
            (&key_pose.rotation_global, &key_pose.translation_global),
            (solution.rotations[k], solution.positions[k]),
        );
    }
    let newest = keys.fixed(solved - 1);
    let ride_rotation = mat3::mat_mul(
        &solution.rotations[solved - 1],
        &mat3::transpose(&newest.rotation_local),
    );
    let ride_translation = mat3::sub(
        &solution.positions[solved - 1],
        &mat3::mat_vec(&ride_rotation, &newest.translation_local),
    );
    for key_pose in keys.range(solved..keys.len()) {
        scan.step(
            config,
            (&key_pose.rotation_local, &key_pose.translation_local),
            (&key_pose.rotation_global, &key_pose.translation_global),
            (
                mat3::mat_mul(&ride_rotation, &key_pose.rotation_local),
                mat3::add(
                    &mat3::mat_vec(&ride_rotation, &key_pose.translation_local),
                    &ride_translation,
                ),
            ),
        );
    }
    (curbed_yank(scan.largest, config.closure_yank_drift_range) - config.graph_change_heat_floor).max(0.0)
}
