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

//! The loop-overlap pressure as a logical-execution-time job (`Config::pressure_let_enabled`): where the scan thread
//! would recompute it (every `pressure_check_frequency`-th scan once the graph has changed), it asks for a pressure job
//! instead. One worker, logically one job at a time: a job starts when the worker is free (at once, or at the landing of
//! the one before), with a snapshot of what the pressure reads taken then, so a request made while the worker is busy
//! waits as the one pending request (a newer request replaces it: the pressure is a state estimate, and the newest
//! inputs are the ones worth computing). Its result lands at its dispatch + estimate (advancing scans and the data-time
//! floor, as the closure searches); everything that reads the pressure reads the newest landed value.

use crate::closure_search_schedule::SearchPool;
use crate::gsc_pgo::GscPgo;
use crate::loop_overlap::LoopOverlap;
use std::sync::{Arc, Condvar, Mutex};

/// A pressure job's outcome (the pressure, None before a sensor range, and its seconds) once it has finished.
#[derive(Default)]
pub struct PressureSlot {
    done: Mutex<Option<(Option<LoopOverlap>, f64)>>,
    changed: Condvar,
}

/// A dispatched pressure job and its schedule: the advancing-scan count and recorded time at its dispatch, its estimate
/// (seconds) and the scan period it is counted in, and the keyframes and committed closures it read.
pub struct PressureJob {
    slot: Arc<PressureSlot>,
    dispatch_scan: usize,
    dispatch_time: f64,
    estimate_raw_scans: f64,
    keyframes: usize,
    closures: usize,
}

impl PressureJob {
    fn estimate_scans(&self, config: &crate::gsc_pgo::Config) -> usize {
        crate::closure_search_schedule::hold_scans(self.estimate_raw_scans, config)
    }
}

impl GscPgo {
    /// The scan thread's pressure step: land a due job, take a request when the graph has changed on a check scan, and
    /// start the pending request when the worker is free (`pressure_let_enabled`).
    pub(crate) fn step_pressure(&mut self, check_due: bool) {
        self.land_pressure();
        if check_due && self.pressure_version != Some(self.graph_version) {
            self.pressure_version = Some(self.graph_version);
            self.pressure_requested = true;
        }
        if self.pressure_requested && self.pressure_job.is_none() {
            self.pressure_requested = false;
            self.dispatch_pressure();
        }
    }
    /// Per pressure computation: the keyframes and committed closures it read and its seconds (inline or as a job), and
    /// the pressure jobs that ran past the data time their estimate spanned.
    pub fn pressure_calls(&self) -> &[(usize, usize, f64)] {
        &self.pressure_calls
    }
    pub fn pressure_overshoots(&self) -> usize {
        self.pressure_overshoots
    }
    /// Per adopted solve, the graph change heat cost's part measured on its solve's thread (keyframes, seconds), and the
    /// solves whose cost the scan thread recomputed because a solve was adopted while they ran.
    pub fn change_cost_calls(&self) -> (&[(usize, f64)], usize) {
        (&self.change_cost_calls, self.graph_change_cost_recomputed)
    }
    /// Seconds the scan thread blocked waiting on jobs (due closure searches and pressure jobs, and solves it joined):
    /// a lockstep replay's waits, left out of the scan thread's work.
    pub fn thread_waits_s(&self) -> f64 {
        self.closure_search_wait_s + self.solve_join_wait_s
    }
    /// A lockstep replay waits for a due job that has not finished (pressure jobs here; the closure searches and solves
    /// are waited for by their own calls), so its results do not depend on the machine's speed.
    pub fn set_wait_for_due_jobs(&mut self, wait: bool) {
        self.wait_for_due_jobs = wait;
    }
    fn dispatch_pressure(&mut self) {
        let snapshot = self.overlap_snapshot();
        let (keyframes, closures) = snapshot.size();
        let config = &self.config;
        let estimate_raw_scans = config.deterministic_config.pressure_scans_fixed_overhead
            + config.deterministic_config.pressure_scans_per_keyframe * keyframes as f64
            + config.deterministic_config.pressure_scans_per_closure * closures as f64;
        let slot = Arc::new(PressureSlot::default());
        let finish = slot.clone();
        self.pressure_pool
            .get_or_insert_with(|| SearchPool::new(1, "gsc-pressure"))
            .send(Box::new(move || {
                let started = std::time::Instant::now();
                let overlap = snapshot.inputs().loop_overlap();
                let seconds = started.elapsed().as_secs_f64();
                *finish.done.lock().expect("pressure slot") = Some((overlap, seconds));
                finish.changed.notify_all();
            }));
        self.pressure_job = Some(PressureJob {
            slot,
            dispatch_scan: self.advancing_scans,
            dispatch_time: self.newest_scan_time(),
            estimate_raw_scans,
            keyframes,
            closures,
        });
    }
    /// Land the pressure job once its hold has passed (its scans and the data time its seconds span), waiting for it
    /// then when `wait_for_due_jobs`; a run that does not wait lands it once it finishes (late: an overshoot).
    fn land_pressure(&mut self) {
        let Some(job) = &self.pressure_job else {
            return;
        };
        let spanned = self.newest_scan_time() - job.dispatch_time;
        let deterministic = self.config.deterministic();
        if deterministic && self.advancing_scans < job.dispatch_scan + job.estimate_scans(&self.config) {
            return;
        }
        let mut done = job.slot.done.lock().expect("pressure slot");
        // fast mode: it lands once it has finished, never waited for
        if done.is_none() && !deterministic {
            return;
        }
        if done.is_none() {
            if !self.wait_for_due_jobs {
                job.slot.changed.notify_all();
                drop(done);
                self.pressure_late = true;
                return;
            }
            let started = std::time::Instant::now();
            done = job
                .slot
                .changed
                .wait_while(done, |done| done.is_none())
                .expect("pressure slot");
            self.closure_search_wait_s += started.elapsed().as_secs_f64();
        }
        let (overlap, seconds) = done.take().expect("finished");
        drop(done);
        let job = self.pressure_job.take().expect("checked above");
        // its work ran past the data time its hold spanned (first reached at the scan it became due), or it landed late
        let overshoot = deterministic && (seconds > spanned || std::mem::take(&mut self.pressure_late));
        if overshoot {
            self.pressure_overshoots += 1;
            eprintln!(
                "gsc_pgo: PRESSURE OVERSHOOT #{}: over {} keyframes and {} closures it took {seconds:.4}s, past the {spanned:.4}s of data its estimate of {:.4} vs ({} scans) spanned",
                self.pressure_overshoots,
                job.keyframes,
                job.closures,
                job.estimate_raw_scans,
                job.estimate_scans(&self.config),
            );
        }
        self.pressure_calls.push((job.keyframes, job.closures, seconds));
        self.record_job_ratio("pressure", seconds, job.estimate_raw_scans);
        if let Some(overlap) = overlap {
            self.pressure = Some((self.scans_seen, overlap));
        }
    }
}
