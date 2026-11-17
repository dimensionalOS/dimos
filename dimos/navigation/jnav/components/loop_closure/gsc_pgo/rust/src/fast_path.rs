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


//! Solver fast path (`solver_fast_path_enabled`): the closures GNC has not judged yet, published before the GNC solve that
//! judges them lands. One idea: the published map is GNC's graph plus a least squares over the keyframes nearest the robot,
//! everything else held exactly where the published map has it.
//!
//! - Window: the keyframes the published map puts nearest the newest one, as many as `solver_fast_path_budget_scans`
//!   affords at `fast_path_scans_per_free_keyframe` (keyframe 0 held, as GNC holds it), ranked by distance.
//! - Taper: the nearer `1 - solver_fast_path_taper_fraction` of the window is free; each farther one is pulled to its
//!   published pose (position and rotation) with the covariance the closure search judges a closure's yank by between it
//!   and the robot (the newest GNC solve's, plus odometry since), times (1 - t) / t at taper depth t: free where the taper
//!   starts, as loose as the map's own uncertainty halfway, fixed at the window's edge.
//! - Graph: the window, its odometry, the held keyframes its odometry and closures reach (fixed).
//! - Closures: the kept ones, the ones the published map already holds, and every pending one (unjudged, not refused)
//!   with an end in the window: a job reapplies everything at once.
//! - Gate (GNC's own): every closure in the window within GNC's cutoff at its scaled variance; else the worst new one is
//!   refused (left to GNC) and the rest stay pending.
//! - Publish: the window at the solve, newer keyframes riding the newest one; no other keyframe moves.
//!
//! A logical-execution-time job (one worker, one job out), held to its estimate in deterministic mode. GNC never reads
//! what it publishes; an adopted GNC solve drops it, and the next job reapplies the closures GNC has not judged.

use std::collections::HashMap;
use std::sync::Arc;

use crate::adoption_schedule::JobSlot;
use crate::big_picture_fit::{closure_edges, odometry_edges_ending, solve_fast_path, Edge, FastSolve, SolveConfig};
use crate::closure_search_schedule::{hold_scans, SearchPool};
use crate::gsc_pgo::{Config, GscPgo};
use crate::mat3::{self, Mat3, Vec3};
use crate::store::{KeyFrames, SolvedPoses};

/// What the worker returns: the window's poses, each closure's residual over GNC's cutoff (problem order), and seconds.
struct FastPathResult {
    rotations: Vec<Mat3>,
    positions: Vec<Vec3>,
    over_cutoff: Vec<f64>,
    converged: bool,
    seconds: f64,
}

/// A dispatched fast path job.
pub struct FastPathJob {
    slot: Arc<JobSlot<FastPathResult>>,
    dispatch_scan: usize,
    dispatch_seen: usize,
    dispatch_time: f64,
    estimate_raw_scans: f64,
    /// GNC solves adopted at dispatch: its result is stale once another has been.
    adopted_solves: u64,
    /// The window (the result's order), the keyframes then, and the window's closures with which of them are new.
    free: Vec<usize>,
    keyframes: usize,
    closures: Vec<usize>,
    new: Vec<bool>,
    /// It was due but unfinished on a scan that could not wait (an overshoot).
    late: bool,
}

/// One landed fast path job (`GscPgo::fast_path_records`).
#[derive(Clone, Debug)]
#[cfg_attr(feature = "native", derive(serde::Serialize))]
pub struct FastPathRecord {
    pub scan: usize,
    pub time: f64,
    pub dispatch_scan: usize,
    /// "adopted", "stale" (a GNC solve landed first) or "over_cutoff" (a closure in the window past GNC's cutoff).
    pub outcome: &'static str,
    /// The new closures it applied, and over_cutoff the one it refused.
    pub closures: Vec<usize>,
    pub refused: Option<usize>,
    pub free: usize,
    pub seconds: f64,
    /// The largest closure residual over GNC's cutoff, whether LM converged, and adopted the largest keyframe move (m).
    pub worst_over_cutoff: f64,
    pub converged: bool,
    pub max_move_m: f64,
}

/// A committed closure's latency (`GscPgo::closure_latency`): scan and data time committed, fast path adopted, GNC judged.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "native", derive(serde::Serialize))]
pub struct ClosureLatency {
    pub commit: (usize, f64),
    pub fast_path: Option<(usize, f64)>,
    pub gnc: Option<(usize, f64)>,
    pub gnc_kept: Option<bool>,
}

impl ClosureLatency {
    pub(crate) fn committed(scan: usize, time: f64) -> ClosureLatency {
        ClosureLatency {
            commit: (scan, time),
            fast_path: None,
            gnc: None,
            gnc_kept: None,
        }
    }
}

impl GscPgo {
    pub fn fast_path_records(&self) -> &[FastPathRecord] {
        &self.fast_path_records
    }
    pub fn closure_latency(&self) -> &[ClosureLatency] {
        &self.closure_latency
    }
    pub fn fast_path_overshoots(&self) -> usize {
        self.fast_path_overshoots
    }
    /// Land a due job, then dispatch one if a closure is pending; true if the published keyframes moved.
    pub(crate) fn step_fast_path(&mut self) -> bool {
        if !self.config.solver_fast_path_enabled {
            return false;
        }
        let moved = self.land_fast_path(false);
        // the published graph takes keyframes made since the correction landed
        if let Some((solved, rotation, translation)) = &self.fast_path_overlay {
            let mut published = self.key_poses.clone();
            published.adopt(solved.clone(), *rotation, *translation);
            self.fast_path_published = Some(published);
        }
        if self.fast_path_job.is_none() {
            self.dispatch_fast_path();
        }
        moved
    }
    /// An adopted GNC solve replaced every pose: the published correction is dropped and the closures it held that GNC
    /// left unjudged are pending again; stamp latencies.
    pub(crate) fn fast_path_superseded(&mut self, kept: &[f64]) {
        self.fast_path_overlay = None;
        self.fast_path_published = None;
        self.fast_path_accepted.clear();
        let now = (self.scans_seen, self.newest_scan_time());
        for (latency, &weight) in self.closure_latency.iter_mut().zip(kept) {
            if latency.gnc.is_none() {
                latency.gnc = Some(now);
                latency.gnc_kept = Some(weight > 0.5);
            }
        }
    }
    /// The window: the `budget` keyframes the published map puts nearest the newest one, keyframe 0 held (nearest first).
    fn fast_path_window(&self) -> Vec<usize> {
        let published = self.key_poses();
        let n = published.len();
        let affordable =
            self.config.solver_fast_path_budget_scans / self.config.deterministic_config.fast_path_scans_per_free_keyframe;
        let budget = if affordable.is_finite() { affordable.max(0.0) as usize } else { n };
        let center = published.position(n - 1);
        let mut near: Vec<(f64, usize)> =
            (1..n).map(|k| (mat3::norm(&mat3::sub(&published.position(k), &center)), k)).collect();
        if near.len() > budget {
            near.select_nth_unstable_by(budget, |a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
            near.truncate(budget);
        }
        near.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        near.into_iter().map(|(_, k)| k).collect()
    }
    fn dispatch_fast_path(&mut self) {
        let n = self.key_poses.len();
        let judged = self.committed_closures.judged();
        let pending = |i: &usize| !self.fast_path_accepted.contains(i) && !self.fast_path_refused.contains(i);
        if n < 2 || !(judged..self.committed_closures.len()).any(|i| pending(&i)) {
            return;
        }
        let free = self.fast_path_window();
        let mut is_free = vec![false; n];
        for &k in &free {
            is_free[k] = true;
        }
        let touches = |i: usize| {
            let c = self.committed_closures.fixed(i);
            c.source_id.max(c.target_id) < n && (is_free[c.source_id] || is_free[c.target_id])
        };
        // kept, already published, and pending closures with an end in the window
        let (closures, new): (Vec<usize>, Vec<bool>) = (0..self.committed_closures.len())
            .filter(|&i| touches(i))
            .filter_map(|i| {
                if i < judged {
                    (self.committed_closures.weight(i) > 0.5).then_some((i, false))
                } else if self.fast_path_accepted.contains(&i) {
                    Some((i, false))
                } else {
                    pending(&i).then_some((i, true))
                }
            })
            .unzip();
        if !new.iter().any(|&new| new) {
            return;
        }
        let selected: Vec<_> = closures.iter().map(|&i| self.committed_closures.get(i)).collect();
        let published = self.key_poses().clone();
        let noise = self.odometry_error_estimate();
        let covariance = self.solved_relative_covariance.clone();
        let estimate_raw_scans = self.config.deterministic_config.fast_path_scans_per_free_keyframe * free.len() as f64;
        let config = self.config.clone();
        let window = free.clone();
        let slot = Arc::new(JobSlot::<FastPathResult>::default());
        let finish = slot.clone();
        self.fast_path_pool
            .get_or_insert_with(|| SearchPool::new(1, "gsc-fast-path"))
            .send(Box::new(move || {
                let started = std::time::Instant::now();
                let graph = window_graph(&published, &config, noise, covariance, &window, selected);
                let fast = solve_window(&graph, &config);
                let count = window.len();
                *finish.done.lock().expect("fast path slot") = Some(FastPathResult {
                    rotations: fast.rotations[..count].to_vec(),
                    positions: fast.positions[..count].to_vec(),
                    over_cutoff: fast.over_cutoff,
                    converged: fast.converged,
                    seconds: started.elapsed().as_secs_f64(),
                });
                finish.changed.notify_all();
            }));
        self.fast_path_job = Some(FastPathJob {
            slot,
            dispatch_scan: self.advancing_scans,
            dispatch_seen: self.scans_seen,
            dispatch_time: self.newest_scan_time(),
            estimate_raw_scans,
            adopted_solves: self.adopted_solves,
            free,
            keyframes: n,
            closures,
            new,
            late: false,
        });
    }
    /// Land the job once its hold has passed (`now`: whatever its hold, waiting for it); true if it was adopted.
    pub(crate) fn land_fast_path(&mut self, now: bool) -> bool {
        let Some(job) = &self.fast_path_job else {
            return false;
        };
        let deterministic = self.config.deterministic();
        if deterministic && !now && self.advancing_scans < job.dispatch_scan + hold_scans(job.estimate_raw_scans) {
            return false;
        }
        let spanned = self.newest_scan_time() - job.dispatch_time;
        let mut done = job.slot.done.lock().expect("fast path slot");
        if done.is_none() {
            // fast mode lands it once finished; deterministic waits in a lockstep replay, else it lands late
            if !now && !(deterministic && self.wait_for_due_jobs) {
                drop(done);
                if let Some(job) = self.fast_path_job.as_mut() {
                    job.late = deterministic;
                }
                return false;
            }
            let started = std::time::Instant::now();
            done = job.slot.changed.wait_while(done, |done| done.is_none()).expect("fast path slot");
            self.closure_search_wait_s += started.elapsed().as_secs_f64();
        }
        let result = done.take().expect("finished");
        drop(done);
        let job = self.fast_path_job.take().expect("checked above");
        let overshoot = deterministic && !now && (result.seconds > spanned || job.late);
        self.note_overshoot("fast_path", overshoot);
        if overshoot {
            self.fast_path_overshoots += 1;
        }
        self.record_job_ratio("fast_path", result.seconds, job.estimate_raw_scans);
        let applied: Vec<usize> = job.closures.iter().zip(&job.new).filter(|(_, &new)| new).map(|(&i, _)| i).collect();
        let worst = result.over_cutoff.iter().copied().fold(0.0, f64::max);
        let mut record = FastPathRecord {
            scan: self.scans_seen,
            time: self.newest_scan_time(),
            dispatch_scan: job.dispatch_seen,
            outcome: "adopted",
            closures: applied.clone(),
            refused: None,
            free: job.free.len(),
            seconds: result.seconds,
            worst_over_cutoff: worst,
            converged: result.converged,
            max_move_m: 0.0,
        };
        let adopted = if self.adopted_solves != job.adopted_solves {
            // GNC replaced the graph it corrected: its closures are still pending, the next job redoes them
            record.outcome = "stale";
            false
        } else if worst > 1.0 {
            // GNC would weigh a closure here: the worst new one waits for GNC, the rest are retried
            let refused = job
                .closures
                .iter()
                .zip(&job.new)
                .zip(&result.over_cutoff)
                .filter(|((_, &new), _)| new)
                .max_by(|a, b| a.1.total_cmp(b.1))
                .map(|((&i, _), _)| i)
                .expect("a new closure");
            self.fast_path_refused.insert(refused);
            record.outcome = "over_cutoff";
            record.refused = Some(refused);
            false
        } else {
            record.max_move_m = self.publish_fast_path(&job, &result);
            let stamp = (self.scans_seen, self.newest_scan_time());
            for &i in &applied {
                self.fast_path_accepted.insert(i);
                self.closure_latency[i].fast_path.get_or_insert(stamp);
            }
            true
        };
        self.fast_path_records.push(record);
        adopted
    }
    /// Publish a passed job: the window at the solve, newer keyframes riding the newest; returns the largest move (m).
    fn publish_fast_path(&mut self, job: &FastPathJob, result: &FastPathResult) -> f64 {
        let n = job.keyframes;
        let published = self.key_poses();
        let (mut rotations, mut positions): (Vec<Mat3>, Vec<Vec3>) = (0..n).map(|k| published.global(k)).unzip();
        let before_positions = positions.clone();
        for (i, &k) in job.free.iter().enumerate() {
            (rotations[k], positions[k]) = (result.rotations[i], result.positions[i]);
        }
        let moved = (0..n)
            .map(|k| mat3::norm(&mat3::sub(&positions[k], &before_positions[k])))
            .fold(0.0, f64::max);
        let newest = self.key_poses.fixed(n - 1);
        let rotation = mat3::mat_mul(&rotations[n - 1], &mat3::transpose(&newest.rotation_local));
        let translation = mat3::sub(&positions[n - 1], &mat3::mat_vec(&rotation, &newest.translation_local));
        let solved = Arc::new(SolvedPoses { rotations, positions });
        let mut graph = self.key_poses.clone();
        graph.adopt(solved.clone(), rotation, translation);
        self.fast_path_overlay = Some((solved, rotation, translation));
        self.fast_path_published = Some(graph);
        moved
    }
}

/// GNC's scaled closure variances and cutoff, no evidence, no seeds.
fn solve_config(config: &Config) -> SolveConfig {
    SolveConfig {
        gnc_variance_scale: config.closure_gnc_var_scale,
        gnc_inlier_probability: config.closure_gnc_inlier_probability,
        huber_k: config.closure_robust_huber_k,
        evidence_core: false,
        learned_evidence: false,
        seed_closure_weights: Vec::new(),
        seed_start: Vec::new(),
        planar: config.planar_pose_graph,
        landmark_warm_start_xy_variance_scale: config.landmark_warm_start_xy_variance_scale,
        landmark_warm_start_xy_variance_ladder: config.landmark_warm_start_xy_variance_ladder,
    }
}

/// The window's problem: nodes (the window nearest first, then the held keyframes it reaches), poses, edges, and per
/// window keyframe the variances of its pull to its published pose before the taper scales them.
#[derive(Clone)]
#[cfg_attr(feature = "native", derive(serde::Serialize, serde::Deserialize))]
pub struct WindowGraph {
    pub keyframes: Vec<usize>,
    pub rotations: Vec<Mat3>,
    pub positions: Vec<Vec3>,
    pub odometry: Vec<Edge>,
    pub closures: Vec<Edge>,
    pub pull: Vec<[f64; 6]>,
}

/// The window, its odometry steps and the held keyframes they and its closures reach; each window keyframe's pull is the
/// closure search's position covariance between it and the robot and its odometry's rotation variance since, on its axes.
fn window_graph(
    published: &KeyFrames,
    config: &Config,
    noise: [f64; 4],
    covariance: Option<(usize, Arc<Vec<Mat3>>)>,
    window: &[usize],
    selected: Vec<crate::gsc_pgo::CommittedClosure>,
) -> WindowGraph {
    let n = published.len();
    let mut node = window.to_vec();
    let mut slot: HashMap<usize, usize> = window.iter().enumerate().map(|(i, &k)| (k, i)).collect();
    let mut index_of = |k: usize, node: &mut Vec<usize>| -> usize {
        *slot.entry(k).or_insert_with(|| {
            node.push(k);
            node.len() - 1
        })
    };
    let mut closures = closure_edges(selected.into_iter());
    for e in &mut closures {
        e.from = index_of(e.from, &mut node);
        e.to = index_of(e.to, &mut node);
    }
    let mut in_window = vec![false; n];
    for &k in window {
        in_window[k] = true;
    }
    // each window keyframe's step in, and its step out when the next keyframe is held
    let mut odometry = Vec::new();
    for &k in window {
        for b in [k, k + 1] {
            if b == k + 1 && (b >= n || in_window[b]) {
                continue;
            }
            let mut edge = odometry_edges_ending(published, config, noise, b..b + 1).remove(0);
            (edge.from, edge.to) = (index_of(b - 1, &mut node), index_of(b, &mut node));
            odometry.push(edge);
        }
    }
    let sums = crate::loop_overlap::odometry_error_estimate_sums(published, config, noise);
    let solved = covariance.as_ref().map(|(tip, covariance)| (*tip, covariance.as_slice()));
    let newest = n - 1;
    let pull = window
        .iter()
        .map(|&k| {
            let (r, _) = published.global(k);
            let local = |world: &Mat3| mat3::mat_mul(&mat3::mat_mul(&mat3::transpose(&r), world), &r);
            let position = local(&crate::loop_overlap::relative_covariance(published, &sums, solved, k, newest));
            let rotation = local(&mat3::sub_mat(&sums[newest][1], &sums[k][1]));
            [rotation[0][0], rotation[1][1], rotation[2][2], position[0][0], position[1][1], position[2][2]]
        })
        .collect();
    let (rotations, positions): (Vec<Mat3>, Vec<Vec3>) = node.iter().map(|&k| published.global(k)).unzip();
    WindowGraph {
        keyframes: node,
        rotations,
        positions,
        odometry,
        closures,
        pull,
    }
}

/// Solve a window: the nearer `1 - solver_fast_path_taper_fraction` free, each taper keyframe pulled to its published pose
/// with its pull times (1 - t) / t at taper depth t in (0, 1), the held keyframes fixed.
pub fn solve_window(graph: &WindowGraph, config: &Config) -> FastSolve {
    let count = graph.pull.len();
    let fraction = config.solver_fast_path_taper_fraction.clamp(0.0, 1.0);
    let free_count = count - (fraction * count as f64).round() as usize;
    let (mut rotations, mut positions) = (graph.rotations.clone(), graph.positions.clone());
    let mut free: Vec<bool> = (0..rotations.len()).map(|i| i < count).collect();
    let mut odometry = graph.odometry.clone();
    for r in free_count..count {
        let t = (r - free_count + 1) as f64 / (count - free_count + 1) as f64;
        // the pull is an edge from a fixed copy of the keyframe at its published pose
        odometry.push(Edge {
            from: rotations.len(),
            to: r,
            rotation: mat3::identity(),
            translation: [0.0; 3],
            variances: graph.pull[r].map(|v| v.max(f64::MIN_POSITIVE) * (1.0 - t) / t),
        });
        rotations.push(graph.rotations[r]);
        positions.push(graph.positions[r]);
        free.push(false);
    }
    solve_fast_path(
        &rotations,
        &positions,
        &free,
        &odometry,
        &graph.closures,
        &solve_config(config),
        config.solver_fast_path_max_lm_steps,
    )
}
