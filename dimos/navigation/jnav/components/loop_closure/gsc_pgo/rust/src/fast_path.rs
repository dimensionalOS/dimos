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
//! judges them lands. One idea: closures that agree move the robot's own recent trail as one block onto the map they
//! close against; every other keyframe stays exactly where the published map has it. No odometry weight is used, so a
//! wrong odometry error estimate cannot make it publish more or less.
//!
//! - Trail: the newest keyframe and the ones before it, back while they stay among the keyframes the published map puts
//!   nearest the newest (as many as `solver_fast_path_budget_scans` affords at `fast_path_scans_per_free_keyframe`).
//! - Closures: each with one end on the trail and the other off it: the published or GNC-kept ones (the newest trail
//!   keyframe one ties is the anchor, where the trail already agrees with the map; else the trail's oldest keyframe) and
//!   every one GNC has not judged (one an earlier job dropped rides along again). A job runs when a closure no job has
//!   dropped arrives.
//! - Shift: the one rigid motion (a turn about the vertical and a translation) that best takes the unjudged closures'
//!   trail ends to where their closures put them, bent in along the trail from the anchor to the oldest such end and in
//!   full beyond it.
//! - Gate (GNC's own rule): while a closure is past GNC's cutoff at its scaled variance, drop the unjudged one furthest past
//!   it and fit again; at least two unjudged closures must agree (a rigid motion fits any one exactly, so one alone is no
//!   evidence: it waits for GNC); and no trail step may change length by more than `solver_fast_path_max_stretch` of itself.
//! - Publish: the trail shifted, newer keyframes riding the newest one; no other keyframe moves.
//!
//! A logical-execution-time job (one worker, one job out), held to its estimate in deterministic mode. GNC never reads
//! what it publishes; an adopted GNC solve drops it, and the next job reapplies the closures GNC has not judged.

use std::sync::Arc;

use crate::adoption_schedule::JobSlot;
use crate::big_picture_fit::chi2_inverse_6dof;
use crate::closure_search_schedule::{hold_scans, SearchPool};
use crate::gsc_pgo::{Config, GscPgo};
use crate::mat3::{self, Mat3, Vec3};
use crate::store::SolvedPoses;

/// What the worker returns: the block shift and seconds.
struct FastPathResult {
    shift: BlockShift,
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
    /// The trail (the result's order), the keyframes then, and the closures between the trail and the rest of the map with
    /// which of them GNC has not judged.
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
    /// "adopted", "stale" (a GNC solve landed first), "over_cutoff" (no unjudged closure agrees within GNC's cutoff) or
    /// "stretched" (the shift would stretch a trail step past `solver_fast_path_max_stretch`).
    pub outcome: &'static str,
    /// The unjudged closures it applied, and the first one it dropped.
    pub closures: Vec<usize>,
    pub refused: Option<usize>,
    pub free: usize,
    pub seconds: f64,
    /// The largest closure residual over GNC's cutoff, the largest trail step stretch (of its length), and adopted the
    /// largest keyframe move (m).
    pub worst_over_cutoff: f64,
    pub stretch: f64,
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
    /// The window: the `budget` keyframes the published map puts nearest the newest one, keyframe 0 held (newest first).
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
        let mut window: Vec<usize> = near.into_iter().map(|(_, k)| k).collect();
        window.sort_unstable_by(|a, b| b.cmp(a));
        window
    }
    fn dispatch_fast_path(&mut self) {
        let n = self.key_poses.len();
        let judged = self.committed_closures.judged();
        let pending = |i: &usize| !self.fast_path_accepted.contains(i) && !self.fast_path_refused.contains(i);
        if n < 2 || !(judged..self.committed_closures.len()).any(|i| pending(&i)) {
            return;
        }
        // the trail: the newest keyframe back while it stays in the window
        let mut in_window = vec![false; n];
        for k in self.fast_path_window() {
            in_window[k] = true;
        }
        in_window[n - 1] = true;
        let mut first = n - 1;
        while first > 1 && in_window[first - 1] {
            first -= 1;
        }
        let free: Vec<usize> = (first..n).collect();
        let on_trail = |k: usize| k >= first && k < n;
        // published, kept and unjudged closures with one end on the trail
        let (closures, new): (Vec<usize>, Vec<bool>) = (0..self.committed_closures.len())
            .filter(|&i| {
                let c = self.committed_closures.fixed(i);
                c.source_id.max(c.target_id) < n && on_trail(c.source_id) != on_trail(c.target_id)
            })
            .filter_map(|i| {
                if i < judged {
                    (self.committed_closures.weight(i) > 0.5).then_some((i, false))
                } else {
                    Some((i, !self.fast_path_accepted.contains(&i)))
                }
            })
            .unzip();
        // an unjudged closure not between the trail and the rest of the map is not the trail's to move: it waits for GNC
        // (it rides along again if a later job's trail reaches it), so it does not start a job every scan
        let elsewhere: Vec<usize> =
            (judged..self.committed_closures.len()).filter(|i| pending(i) && !closures.contains(i)).collect();
        self.fast_path_refused.extend(elsewhere);
        if !new.iter().any(|&new| new) {
            return;
        }
        let published = self.key_poses();
        let ends: Vec<TrailClosure> = closures
            .iter()
            .zip(&new)
            .map(|(&i, &new)| {
                let c = self.committed_closures.get(i);
                let variance = c.score.max(1e-6);
                if on_trail(c.source_id) {
                    // source = target * closure
                    let (r, p) = published.global(c.target_id);
                    let rotation = mat3::mat_mul(&r, &c.rotation_offset);
                    let position = mat3::add(&p, &mat3::mat_vec(&r, &c.translation_offset));
                    TrailClosure { keyframe: c.source_id, rotation, position, variance, new }
                } else {
                    // target = source * closure^-1
                    let (r, p) = published.global(c.source_id);
                    let rotation = mat3::mat_mul(&r, &mat3::transpose(&c.rotation_offset));
                    let position = mat3::sub(&p, &mat3::mat_vec(&rotation, &c.translation_offset));
                    TrailClosure { keyframe: c.target_id, rotation, position, variance, new }
                }
            })
            .collect();
        let (rotations, positions): (Vec<Mat3>, Vec<Vec3>) = free.iter().map(|&k| published.global(k)).unzip();
        let estimate_raw_scans = self.config.deterministic_config.fast_path_scans_per_free_keyframe * free.len() as f64;
        let config = self.config.clone();
        let trail = free.clone();
        let slot = Arc::new(JobSlot::<FastPathResult>::default());
        let finish = slot.clone();
        self.fast_path_pool
            .get_or_insert_with(|| SearchPool::new(1, "gsc-fast-path"))
            .send(Box::new(move || {
                let started = std::time::Instant::now();
                let shift = block_shift(&trail, &rotations, &positions, &ends, &config);
                *finish.done.lock().expect("fast path slot") =
                    Some(FastPathResult { shift, seconds: started.elapsed().as_secs_f64() });
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
        let shift = &result.shift;
        let applied: Vec<usize> = (0..job.closures.len())
            .filter(|p| job.new[*p] && !shift.dropped.contains(p))
            .map(|p| job.closures[p])
            .collect();
        let worst = shift.over_cutoff.iter().copied().fold(0.0, f64::max);
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
            stretch: shift.stretch,
            max_move_m: 0.0,
        };
        let adopted = if self.adopted_solves != job.adopted_solves {
            // GNC replaced the graph it corrected: its closures are still pending, the next job redoes them
            record.outcome = "stale";
            false
        } else if worst > 1.0 || applied.is_empty() || !shift.within_stretch {
            // nothing unjudged agrees, or agreeing would stretch the trail: they wait for GNC or a later job
            for (p, &new) in job.new.iter().enumerate() {
                if new {
                    self.fast_path_refused.insert(job.closures[p]);
                }
            }
            record.outcome = if worst > 1.0 || applied.is_empty() { "over_cutoff" } else { "stretched" };
            record.refused = shift.dropped.first().map(|&p| job.closures[p]);
            false
        } else {
            for &p in &shift.dropped {
                self.fast_path_refused.insert(job.closures[p]);
            }
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
            (rotations[k], positions[k]) = (result.shift.rotations[i], result.shift.positions[i]);
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

/// A closure between the trail and the rest of the map: its trail keyframe, where the closure puts that keyframe (from its
/// other end's published pose), its variance, and whether GNC has yet to judge it.
#[derive(Clone, Debug)]
pub struct TrailClosure {
    pub keyframe: usize,
    pub rotation: Mat3,
    pub position: Vec3,
    pub variance: f64,
    pub new: bool,
}

/// A block shift: the trail's new poses, each closure it kept with its residual over GNC's cutoff, the unjudged closures it
/// dropped (positions in the closure list), the largest trail step stretch, and whether that is within
/// `solver_fast_path_max_stretch`.
#[derive(Clone, Debug)]
pub struct BlockShift {
    pub rotations: Vec<Mat3>,
    pub positions: Vec<Vec3>,
    pub over_cutoff: Vec<f64>,
    pub dropped: Vec<usize>,
    pub stretch: f64,
    pub within_stretch: bool,
}

/// The heading of a rotation about the vertical.
fn yaw(r: &Mat3) -> f64 {
    r[1][0].atan2(r[0][0])
}

/// Shift `trail` (consecutive keyframes ending at the newest, at `rotations` / `positions`) by the rigid motion its unjudged
/// closures agree on (module docs), dropping by GNC's rule the unjudged closure furthest past the cutoff until the rest fit.
pub fn block_shift(trail: &[usize], rotations: &[Mat3], positions: &[Vec3], closures: &[TrailClosure], config: &Config) -> BlockShift {
    let at = |k: usize| k - trail[0];
    let cutoff = chi2_inverse_6dof(config.closure_gnc_inlier_probability);
    let anchor = closures.iter().filter(|c| !c.new).map(|c| c.keyframe).max().unwrap_or(trail[0]);
    // unjudged closures at or behind the anchor say nothing about the trail past it
    let mut dropped: Vec<usize> = (0..closures.len()).filter(|&p| closures[p].new && closures[p].keyframe <= anchor).collect();
    let mut kept: Vec<usize> = (0..closures.len()).filter(|p| !dropped.contains(p)).collect();
    let mut path = vec![0.0; trail.len()];
    for i in 1..trail.len() {
        path[i] = path[i - 1] + mat3::norm(&mat3::sub(&positions[i], &positions[i - 1]));
    }
    let refuse = |dropped: Vec<usize>, over: Vec<f64>| BlockShift {
        rotations: rotations.to_vec(),
        positions: positions.to_vec(),
        over_cutoff: over,
        dropped,
        stretch: 0.0,
        within_stretch: false,
    };
    loop {
        // a rigid motion fits any one closure exactly: only two or more can agree (or not)
        let news: Vec<usize> = kept.iter().copied().filter(|&p| closures[p].new).collect();
        if news.len() < 2 {
            dropped.extend(news);
            return refuse(dropped, vec![f64::INFINITY]);
        }
        let first = news.iter().map(|&p| closures[p].keyframe).min().expect("a new closure");
        let span = (path[at(first)] - path[at(anchor)]).max(1e-6);
        // the motion's turn and translation: from the unjudged closures' trail ends to where their closures put them
        let count = news.len() as f64;
        let mean = |f: &dyn Fn(usize) -> Vec3| -> Vec3 {
            let sum = news.iter().fold([0.0; 3], |s, &p| mat3::add(&s, &f(p)));
            mat3::scale(&sum, 1.0 / count)
        };
        let pc = mean(&|p| positions[at(closures[p].keyframe)]);
        let qc = mean(&|p| closures[p].position);
        let (mut h00, mut h01, mut h10, mut h11) = (0.0, 0.0, 0.0, 0.0);
        for &p in &news {
            let a = mat3::sub(&positions[at(closures[p].keyframe)], &pc);
            let b = mat3::sub(&closures[p].position, &qc);
            (h00, h01, h10, h11) = (h00 + a[0] * b[0], h01 + a[0] * b[1], h10 + a[1] * b[0], h11 + a[1] * b[1]);
        }
        let turn = (h01 - h10).atan2(h00 + h11);
        let slide = mat3::sub(&qc, &pc);
        // bent in from the anchor to the oldest unjudged end, in full beyond: turned about the ends' centroid, slid
        let moved = |i: usize| -> (Mat3, Vec3) {
            let w = if trail[i] <= anchor { 0.0 } else { ((path[i] - path[at(anchor)]) / span).min(1.0) };
            let r = mat3::rot_z(w * turn);
            let p = mat3::add(&mat3::add(&mat3::mat_vec(&r, &mat3::sub(&positions[i], &pc)), &pc), &mat3::scale(&slide, w));
            (mat3::mat_mul(&r, &rotations[i]), p)
        };
        let over: Vec<f64> = kept
            .iter()
            .map(|&p| {
                let c = &closures[p];
                let (r, x) = moved(at(c.keyframe));
                let d = mat3::sub(&x, &c.position);
                let turned = yaw(&mat3::mat_mul(&mat3::transpose(&c.rotation), &r));
                (mat3::norm(&d).powi(2) + turned * turned) / (c.variance * config.closure_gnc_var_scale) / cutoff
            })
            .collect();
        if over.iter().all(|&o| o <= 1.0) {
            let (shifted_rotations, shifted): (Vec<Mat3>, Vec<Vec3>) = (0..trail.len()).map(moved).unzip();
            let stretch = (1..trail.len())
                .map(|i| {
                    let before = path[i] - path[i - 1];
                    let after = mat3::norm(&mat3::sub(&shifted[i], &shifted[i - 1]));
                    (after - before).abs() / before.max(1e-3)
                })
                .fold(0.0, f64::max);
            return BlockShift {
                rotations: shifted_rotations,
                positions: shifted,
                over_cutoff: over,
                dropped,
                stretch,
                within_stretch: stretch <= config.solver_fast_path_max_stretch,
            };
        }
        let worst = kept
            .iter()
            .zip(&over)
            .filter(|(&p, &o)| closures[p].new && o > 1.0)
            .max_by(|a, b| a.1.total_cmp(b.1))
            .map(|(&p, _)| p);
        match worst {
            Some(p) if news.len() > 2 => {
                dropped.push(p);
                kept.retain(|&q| q != p);
            }
            // an already published or kept closure no longer fits, or the last unjudged one does not
            _ => {
                dropped.extend(news);
                return refuse(dropped, over);
            }
        }
    }
}
