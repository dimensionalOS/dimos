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

//! Solver fast path (`solver_fast_path_enabled`): a recent closure's local least-squares correction, adopted until a GNC solve replaces it.

use std::sync::Arc;

use crate::adoption_schedule::JobSlot;
use crate::big_picture_fit::{closure_edges, odometry_edges_ending, solve_fast_path, SolveConfig};
use crate::closure_search_schedule::{hold_scans, SearchPool};
use crate::gsc_pgo::GscPgo;
use crate::mat3::{self, Mat3, Vec3};
use crate::pose_graph_solving::graph_change_cost_on;
use crate::store::SolvedPoses;

/// What the fast path's worker returns: the first `keyframes` global poses, heat cost, seconds, and the GNC cutoff check
/// and convergence (`FastSolve`).
struct FastPathResult {
    solved: SolvedPoses,
    converged: bool,
    cost: f64,
    seconds: f64,
    worst_over_cutoff: f64,
    cost_rise_over_cutoff: f64,
}

/// A dispatched fast path job.
pub struct FastPathJob {
    slot: Arc<JobSlot<FastPathResult>>,
    dispatch_scan: usize,
    /// `scans_seen` at dispatch (`dispatch_scan` counts advancing scans), for its record.
    dispatch_seen: usize,
    dispatch_time: f64,
    estimate_raw_scans: f64,
    /// GNC solves adopted at dispatch: its result is stale once another has been.
    adopted_solves: u64,
    /// Committed closures at dispatch, the closures it applies (unjudged, small), its first keyframe (held) and keyframes.
    closures: usize,
    applied: Vec<usize>,
    first: usize,
    keyframes: usize,
    /// It was due but unfinished on a scan that could not wait (an overshoot).
    late: bool,
}

/// One landed fast path job (`GscPgo::fast_path_records`): when, what, and the window's positions around it.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "native", derive(serde::Serialize))]
pub struct FastPathRecord {
    pub scan: usize,
    pub time: f64,
    pub dispatch_scan: usize,
    /// "adopted", "stale" (a GNC solve landed first), "over_cutoff" (GNC would weigh a closure) or "heat_held".
    pub outcome: &'static str,
    pub closures: Vec<usize>,
    pub first_keyframe: usize,
    pub keyframes: usize,
    pub seconds: f64,
    pub heat_cost: f64,
    /// `FastSolve::worst_over_cutoff`, `cost_rise_over_cutoff` and `converged`.
    pub worst_over_cutoff: f64,
    pub converged: bool,
    pub cost_rise_over_cutoff: f64,
    /// Adopted only: keyframes first..keyframes' times and positions before it, after it, and after the GNC solve judging it.
    pub times: Vec<f64>,
    pub before: Vec<Vec3>,
    pub after: Vec<Vec3>,
    pub gnc: Vec<Vec3>,
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
    /// Land a due fast path job, then dispatch one if a small closure is unjudged and unreflected; true if keyframes moved.
    pub(crate) fn step_fast_path(&mut self) -> bool {
        if !self.config.solver_fast_path_enabled {
            return false;
        }
        let moved = self.land_fast_path(false);
        if self.fast_path_job.is_none() {
            self.dispatch_fast_path();
        }
        moved
    }
    /// An adopted GNC solve replaced every pose: the graph reflects only judged closures; stamp latencies and records.
    pub(crate) fn fast_path_superseded(&mut self, kept: &[f64]) {
        self.fast_path_reflected = 0;
        let now = (self.scans_seen, self.newest_scan_time());
        for (latency, &weight) in self.closure_latency.iter_mut().zip(kept) {
            if latency.gnc.is_none() {
                latency.gnc = Some(now);
                latency.gnc_kept = Some(weight > 0.5);
            }
        }
        let judged = kept.len();
        for record in &mut self.fast_path_records {
            let covered = record.closures.iter().all(|&i| i < judged);
            if record.outcome == "adopted" && record.gnc.is_empty() && covered {
                record.gnc = (record.first_keyframe..record.keyframes)
                    .map(|k| self.key_poses.position(k))
                    .collect();
            }
        }
    }
    /// The closures past the judged ones whose older end is within `solver_fast_path_max_keyframes` of the newest keyframe.
    fn small_unjudged(&self) -> Vec<usize> {
        let n = self.key_poses.len();
        let max = self.config.solver_fast_path_max_keyframes;
        (self.committed_closures.judged()..self.committed_closures.len())
            .filter(|&i| {
                let c = self.committed_closures.fixed(i);
                c.source_id.max(c.target_id) < n && n - c.source_id.min(c.target_id) <= max
            })
            .collect()
    }
    fn dispatch_fast_path(&mut self) {
        let total = self.committed_closures.len();
        let unreflected = self
            .fast_path_reflected
            .max(self.committed_closures.judged());
        if total <= unreflected {
            return;
        }
        let applied = self.small_unjudged();
        if !applied.iter().any(|&i| i >= unreflected) {
            self.fast_path_reflected = total;
            return;
        }
        let n = self.key_poses.len();
        let older = |i: usize| {
            let c = self.committed_closures.fixed(i);
            c.source_id.min(c.target_id)
        };
        let first = applied
            .iter()
            .map(|&i| older(i))
            .min()
            .expect("one applied");
        // the window first..n (first held), then each held keyframe a kept closure ties it to
        let mut node: Vec<usize> = (first..n).collect();
        let mut slot = std::collections::HashMap::new();
        let mut index_of = |k: usize, node: &mut Vec<usize>| -> usize {
            if k >= first {
                return k - first;
            }
            *slot.entry(k).or_insert_with(|| {
                node.push(k);
                node.len() - 1
            })
        };
        // the applied closures (new to the graph) and every kept closure reaching into the window
        let (selected, new): (Vec<crate::gsc_pgo::CommittedClosure>, Vec<bool>) = self
            .committed_closures
            .iter()
            .enumerate()
            .map(|(i, c)| (c, applied.contains(&i)))
            .filter(|&(c, new)| new || (c.is_kept() && c.source_id.max(c.target_id) >= first))
            .unzip();
        let mut closures = closure_edges(selected.into_iter());
        for e in &mut closures {
            e.from = index_of(e.from, &mut node);
            e.to = index_of(e.to, &mut node);
        }
        let mut odometry = odometry_edges_ending(
            &self.key_poses,
            &self.config,
            self.odometry_error_estimate(),
            first + 1..n,
        );
        for e in &mut odometry {
            e.from -= first;
            e.to -= first;
        }
        let (rotations, positions): (Vec<Mat3>, Vec<Vec3>) =
            node.iter().map(|&k| self.key_poses.global(k)).unzip();
        let free: Vec<bool> = node.iter().map(|&k| k > first).collect();
        let dc = &self.config.deterministic_config;
        let estimate_raw_scans = dc.fast_path_scans_fixed_overhead
            + dc.fast_path_scans_per_window_keyframe * (n - first) as f64
            + dc.fast_path_scans_per_keyframe * n as f64;
        let keys = self.key_poses.clone();
        let config = self.config.clone();
        let slot = Arc::new(JobSlot::<FastPathResult>::default());
        let finish = slot.clone();
        self.fast_path_pool
            .get_or_insert_with(|| SearchPool::new(1, "gsc-fast-path"))
            .send(Box::new(move || {
                let started = std::time::Instant::now();
                let solve_config = SolveConfig {
                    gnc_variance_scale: config.closure_gnc_var_scale,
                    gnc_inlier_probability: config.closure_gnc_inlier_probability,
                    huber_k: config.closure_robust_huber_k,
                    evidence_core: false,
                    learned_evidence: false,
                    seed_closure_weights: Vec::new(),
                    seed_start: Vec::new(),
                    planar: config.planar_pose_graph,
                    landmark_warm_start_xy_variance_scale: config
                        .landmark_warm_start_xy_variance_scale,
                    landmark_warm_start_xy_variance_ladder: config
                        .landmark_warm_start_xy_variance_ladder,
                };
                let steps = config.solver_fast_path_max_lm_steps;
                let fast = solve_fast_path(
                    &rotations,
                    &positions,
                    &free,
                    &odometry,
                    &closures,
                    &new,
                    &solve_config,
                    steps,
                );
                let (window_rotations, window_positions) = (fast.rotations, fast.positions);
                let (mut rotations, mut positions): (Vec<Mat3>, Vec<Vec3>) =
                    (0..first).map(|k| keys.global(k)).unzip();
                rotations.extend_from_slice(&window_rotations[..n - first]);
                positions.extend_from_slice(&window_positions[..n - first]);
                let solved = SolvedPoses {
                    rotations,
                    positions,
                };
                let cost = if config.graph_change_heat_enabled && fast.worst_over_cutoff <= 1.0 {
                    graph_change_cost_on(&keys, &config, (&solved.rotations, &solved.positions), n)
                } else {
                    0.0
                };
                *finish.done.lock().expect("fast path slot") = Some(FastPathResult {
                    solved,
                    converged: fast.converged,
                    cost,
                    seconds: started.elapsed().as_secs_f64(),
                    worst_over_cutoff: fast.worst_over_cutoff,
                    cost_rise_over_cutoff: fast.cost_rise_over_cutoff,
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
            closures: total,
            applied,
            first,
            keyframes: n,
            late: false,
        });
    }
    /// Land the job once its hold has passed (`now`: whatever its hold, waiting for it); true if it was adopted.
    pub(crate) fn land_fast_path(&mut self, now: bool) -> bool {
        let Some(job) = &self.fast_path_job else {
            return false;
        };
        let deterministic = self.config.deterministic();
        if deterministic
            && !now
            && self.advancing_scans < job.dispatch_scan + hold_scans(job.estimate_raw_scans)
        {
            return false;
        }
        let spanned = self.newest_scan_time() - job.dispatch_time;
        let mut done = job.slot.done.lock().expect("fast path slot");
        if done.is_none() {
            // not finished: it lands late (deterministic) or once it finishes (fast)
            if !now {
                drop(done);
                if let Some(job) = self.fast_path_job.as_mut() {
                    job.late = deterministic;
                }
                return false;
            }
            done = job
                .slot
                .changed
                .wait_while(done, |done| done.is_none())
                .expect("fast path slot");
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
        let mut record = FastPathRecord {
            scan: self.scans_seen,
            time: self.newest_scan_time(),
            dispatch_scan: job.dispatch_seen,
            outcome: "adopted",
            closures: job.applied.clone(),
            first_keyframe: job.first,
            keyframes: job.keyframes,
            seconds: result.seconds,
            heat_cost: result.cost,
            worst_over_cutoff: result.worst_over_cutoff,
            converged: result.converged,
            cost_rise_over_cutoff: result.cost_rise_over_cutoff,
            times: Vec::new(),
            before: Vec::new(),
            after: Vec::new(),
            gnc: Vec::new(),
        };
        let heat = self.heat_at(self.key_poses.len());
        let adopted = match result.solved {
            // a GNC solve adopted since replaced the graph it corrected; the next step redoes it on the new one
            _ if self.adopted_solves != job.adopted_solves => {
                record.outcome = "stale";
                false
            }
            // GNC would weigh a closure (it might reject it): leave the loop to GNC
            _ if result.worst_over_cutoff > 1.0 => {
                record.outcome = "over_cutoff";
                false
            }
            _ if self.config.graph_change_heat_enabled
                && heat > self.config.graph_change_heat_threshold =>
            {
                record.outcome = "heat_held";
                false
            }
            solved => {
                if self.config.graph_change_heat_enabled {
                    self.heat = heat + result.cost;
                    self.heat_keyframes = self.key_poses.len();
                }
                record.times = (job.first..job.keyframes)
                    .map(|k| self.key_poses.fixed(k).time)
                    .collect();
                record.before = (job.first..job.keyframes)
                    .map(|k| self.key_poses.position(k))
                    .collect();
                record.after = solved.positions[job.first..].to_vec();
                self.adopt_fast_path(solved, job.keyframes);
                let stamp = (self.scans_seen, self.newest_scan_time());
                for &i in &job.applied {
                    self.closure_latency[i].fast_path.get_or_insert(stamp);
                }
                true
            }
        };
        // a stale result is redone; the rest are not retried
        if record.outcome != "stale" {
            self.fast_path_reflected = job.closures;
        }
        self.fast_path_records.push(record);
        adopted
    }
    /// Swap in the fast path's first `n` poses; newer keyframes ride its newest one's correction (as `adopt_solution`).
    fn adopt_fast_path(&mut self, solved: SolvedPoses, n: usize) {
        let newest = self.key_poses.fixed(n - 1);
        self.rotation_offset = mat3::mat_mul(
            &solved.rotations[n - 1],
            &mat3::transpose(&newest.rotation_local),
        );
        self.translation_offset = mat3::sub(
            &solved.positions[n - 1],
            &mat3::mat_vec(&self.rotation_offset, &newest.translation_local),
        );
        self.key_poses.adopt(
            Arc::new(solved),
            self.rotation_offset,
            self.translation_offset,
        );
        self.graph_version += 1;
    }
}
