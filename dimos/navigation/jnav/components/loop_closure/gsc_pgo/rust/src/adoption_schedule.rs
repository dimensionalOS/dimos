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

//! A finished solve's way into the graph, two logical-execution-time jobs after the solve (which is only the GNC solve):
//!
//! - Debounce (`debounce_scans_per_keyframe`): the graph change heat's cost of the solve, measured off the scan thread on
//!   a snapshot of the accepted graph (`store`, O(1) to take) and the keyframes made since the solve started; at its
//!   landing the heat check adopts it or holds it as the newest waiting solve (an older one superseded). A solve that
//!   finishes while a debounce runs waits as the newest pending one.
//! - Merge (`merge_scans_per_keyframe`): prepares the adoption off the scan thread (the closure verdicts that changed
//!   against the weights now, the longest kept loop); at its landing the scan thread swaps the new graph version in (the
//!   solve's pose and weight vectors, moved into `Arc`s) and does the O(changed) bookkeeping. One adoption waiting
//!   for a busy merge is the newest (an older one superseded).
//!
//! Each lands at its dispatch + `ceil(estimate)` advancing scans in deterministic mode (a lockstep replay waiting for it
//! then), or as soon as it finishes in fast mode. At the end of the data both run inline.

use crate::closure_search_schedule::{hold_scans, SearchPool};
use crate::gsc_pgo::GscPgo;
use crate::pose_graph_solving::{graph_change_cost_on, FinishedSolve};
use std::sync::{Arc, Condvar, Mutex};

/// A job's result once it has finished.
pub struct JobSlot<T> {
    done: Mutex<Option<T>>,
    changed: Condvar,
}

impl<T> Default for JobSlot<T> {
    fn default() -> Self {
        JobSlot {
            done: Mutex::new(None),
            changed: Condvar::new(),
        }
    }
}

/// A dispatched debounce or merge job: its result slot, the advancing scan it was dispatched at, its estimate (scans,
/// before rounding up) and the keyframes it was sized by.
pub struct AdoptionJob<T> {
    slot: Arc<JobSlot<T>>,
    dispatch_scan: usize,
    estimate_raw_scans: f64,
}

/// What the debounce measured: the solve, its graph change heat cost and the seconds it took.
pub struct Debounced {
    finished: FinishedSolve,
    cost: f64,
    seconds: f64,
}

/// What the merge prepared: the solve, its heat (before, cost, after), the closures whose verdict changes (index, new
/// weight), the longest kept loop (keyframes between its ends) and the seconds it took.
pub struct Merged {
    finished: FinishedSolve,
    heat: Option<(f64, f64, f64)>,
    pub(crate) prepared: MergePrepared,
    seconds: f64,
}

/// The merge's off-thread preparation, read by `GscPgo::adopt_solution`.
pub struct MergePrepared {
    pub(crate) changed: Vec<(usize, f64)>,
    pub(crate) longest_kept: usize,
}

impl GscPgo {
    /// Hand a finished solve to the debounce: dispatched at once if it is free, else the newest pending one.
    pub(crate) fn offer_debounce(&mut self, finished: FinishedSolve) {
        if self.debounce_job.is_some() {
            if let Some(older) = self.debounce_waiting.replace(finished) {
                let heat = self.heat_at(self.key_poses.len());
                self.log_heat_event("superseded", &older, heat);
            }
            return;
        }
        self.dispatch_debounce(finished);
    }
    fn dispatch_debounce(&mut self, finished: FinishedSolve) {
        let keys = self.key_poses.clone();
        let config = self.config.clone();
        let estimate_raw_scans =
            self.config.deterministic_config.debounce_scans_per_keyframe * keys.len() as f64;
        let slot = Arc::new(JobSlot::<Debounced>::default());
        let finish = slot.clone();
        self.adoption_pool
            .get_or_insert_with(|| SearchPool::new(1, "gsc-debounce"))
            .send(Box::new(move || {
                let started = std::time::Instant::now();
                let cost = if config.graph_change_heat_enabled {
                    graph_change_cost_on(&keys, &config, &finished.solution, finished.keyframes)
                } else {
                    0.0
                };
                let seconds = started.elapsed().as_secs_f64();
                *finish.done.lock().expect("debounce slot") = Some(Debounced {
                    finished,
                    cost,
                    seconds,
                });
                finish.changed.notify_all();
            }));
        self.debounce_job = Some(AdoptionJob {
            slot,
            dispatch_scan: self.advancing_scans,
            estimate_raw_scans,
        });
    }
    /// Hand an adopted solve to the merge: dispatched at once if it is free, else the newest waiting one.
    pub(crate) fn offer_merge(&mut self, finished: FinishedSolve, heat: Option<(f64, f64, f64)>) {
        if self.merge_job.is_some() {
            if let Some((older, _)) = self.merge_waiting.replace((finished, heat)) {
                let now = self.heat_at(self.key_poses.len());
                self.log_heat_event("superseded", &older, now);
            }
            return;
        }
        self.dispatch_merge(finished, heat);
    }
    fn dispatch_merge(&mut self, finished: FinishedSolve, heat: Option<(f64, f64, f64)>) {
        let closures = self.committed_closures.clone();
        let estimate_raw_scans =
            self.config.deterministic_config.merge_scans_per_keyframe * finished.keyframes as f64;
        let slot = Arc::new(JobSlot::<Merged>::default());
        let finish = slot.clone();
        self.merge_pool
            .get_or_insert_with(|| SearchPool::new(1, "gsc-merge"))
            .send(Box::new(move || {
                let started = std::time::Instant::now();
                let weights = &finished.solution.closure_weights;
                let changed = weights
                    .iter()
                    .copied()
                    .enumerate()
                    .filter(|&(i, weight)| closures.weight(i) != weight)
                    .collect();
                let longest_kept = weights
                    .iter()
                    .enumerate()
                    .filter(|(_, &weight)| weight > 0.5)
                    .map(|(i, _)| {
                        let fixed = closures.fixed(i);
                        fixed.source_id.abs_diff(fixed.target_id)
                    })
                    .max()
                    .unwrap_or(0);
                let seconds = started.elapsed().as_secs_f64();
                *finish.done.lock().expect("merge slot") = Some(Merged {
                    finished,
                    heat,
                    prepared: MergePrepared {
                        changed,
                        longest_kept,
                    },
                    seconds,
                });
                finish.changed.notify_all();
            }));
        self.merge_job = Some(AdoptionJob {
            slot,
            dispatch_scan: self.advancing_scans,
            estimate_raw_scans,
        });
    }
    /// Land the debounce and the merge once their holds have passed (deterministic) or they have finished (fast),
    /// waiting for one due when `wait_for_due_jobs` or `now` (the end of the data, holds ignored). Returns whether a merge
    /// moved the keyframes.
    pub(crate) fn step_adoption(&mut self, now: bool) -> bool {
        let mut moved = false;
        if let Some(debounced) = self.land_adoption_job::<Debounced>(true, now) {
            self.change_cost_calls
                .push((debounced.finished.keyframes, debounced.seconds));
            let estimate = self.config.deterministic_config.debounce_scans_per_keyframe
                * self.key_poses.len() as f64;
            self.record_job_ratio("debounce", debounced.seconds, estimate);
            self.debounced(debounced.finished, debounced.cost, false);
            if let Some(waiting) = self.debounce_waiting.take() {
                self.dispatch_debounce(waiting);
            }
        }
        if let Some(merged) = self.land_adoption_job::<Merged>(false, now) {
            let estimate = self.config.deterministic_config.merge_scans_per_keyframe
                * merged.finished.keyframes as f64;
            self.record_job_ratio("merge", merged.seconds, estimate);
            moved |= self.adopt_solution(merged.finished, merged.heat, Some(merged.prepared));
            if let Some((waiting, heat)) = self.merge_waiting.take() {
                self.dispatch_merge(waiting, heat);
            }
        }
        moved
    }
    /// Whether a debounce or a merge is out or waiting.
    pub(crate) fn adoption_busy(&self) -> bool {
        self.debounce_job.is_some()
            || self.debounce_waiting.is_some()
            || self.merge_job.is_some()
            || self.merge_waiting.is_some()
    }
    fn land_adoption_job<T: AdoptionKind>(&mut self, debounce: bool, now: bool) -> Option<T> {
        let deterministic = self.config.deterministic();
        let advancing = self.advancing_scans;
        let wait = self.wait_for_due_jobs || now;
        let job = T::job(self, debounce)?;
        let due = advancing >= job.dispatch_scan + hold_scans(job.estimate_raw_scans, &self.config);
        if deterministic && !due && !now {
            return None;
        }
        let mut done = job.slot.done.lock().expect("adoption slot");
        if done.is_none() {
            if !(now || (deterministic && wait)) {
                return None;
            }
            let started = std::time::Instant::now();
            done = job
                .slot
                .changed
                .wait_while(done, |done| done.is_none())
                .expect("adoption slot");
            let waited = started.elapsed().as_secs_f64();
            let result = done.take();
            drop(done);
            self.closure_search_wait_s += waited;
            T::clear(self, debounce);
            return result;
        }
        let result = done.take();
        drop(done);
        T::clear(self, debounce);
        result
    }
}

/// The debounce's and the merge's results, each kept in its own job field.
pub(crate) trait AdoptionKind: Sized {
    fn job(pgo: &GscPgo, debounce: bool) -> Option<&AdoptionJob<Self>>;
    fn clear(pgo: &mut GscPgo, debounce: bool);
}

impl AdoptionKind for Debounced {
    fn job(pgo: &GscPgo, _: bool) -> Option<&AdoptionJob<Self>> {
        pgo.debounce_job.as_ref()
    }
    fn clear(pgo: &mut GscPgo, _: bool) {
        pgo.debounce_job = None;
    }
}

impl AdoptionKind for Merged {
    fn job(pgo: &GscPgo, _: bool) -> Option<&AdoptionJob<Self>> {
        pgo.merge_job.as_ref()
    }
    fn clear(pgo: &mut GscPgo, _: bool) {
        pgo.merge_job = None;
    }
}
