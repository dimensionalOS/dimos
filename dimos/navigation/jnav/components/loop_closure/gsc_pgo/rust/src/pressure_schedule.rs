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

//! Loop-overlap pressure as a logical-execution-time job: one worker, one pending request (newest replaces it); a result lands at dispatch + estimate, and readers see the newest landed value.

use crate::closure_search_schedule::SearchPool;
use crate::gsc_pgo::GscPgo;
use crate::loop_overlap::LoopOverlap;
use std::sync::Arc;

/// A pressure job's outcome: the pressure (None before a sensor range) and its seconds.
type PressureSlot = crate::adoption_schedule::JobSlot<(Option<LoopOverlap>, f64)>;

/// A dispatched pressure job: advancing scans and data time at dispatch, estimate (raw scans).
pub struct PressureJob {
    slot: Arc<PressureSlot>,
    dispatch_scan: usize,
    dispatch_time: f64,
    estimate_raw_scans: f64,
}

impl PressureJob {
    fn estimate_scans(&self) -> usize {
        crate::closure_search_schedule::hold_scans(self.estimate_raw_scans)
    }
}

impl GscPgo {
    /// Land a due job, request one if the graph changed on a check scan, dispatch it when the worker is free.
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
        });
    }
    /// Land the job once its hold has passed (one still running then lands late, as an overshoot).
    fn land_pressure(&mut self) {
        let Some(job) = &self.pressure_job else {
            return;
        };
        let spanned = self.newest_scan_time() - job.dispatch_time;
        let deterministic = self.config.deterministic();
        if deterministic && self.advancing_scans < job.dispatch_scan + job.estimate_scans() {
            return;
        }
        let mut done = job.slot.done.lock().expect("pressure slot");
        // fast mode: it lands once it has finished, never waited for
        if done.is_none() && !deterministic {
            return;
        }
        if done.is_none() {
            job.slot.changed.notify_all();
            drop(done);
            self.pressure_late = true;
            return;
        }
        let (overlap, seconds) = done.take().expect("finished");
        drop(done);
        let job = self.pressure_job.take().expect("checked above");
        // overshoot: its work ran past the data time its hold spanned, or it landed late
        let overshoot =
            deterministic && (seconds > spanned || std::mem::take(&mut self.pressure_late));
        self.note_overshoot("pressure", overshoot);
        self.record_job_ratio("pressure", seconds, job.estimate_raw_scans);
        if let Some(overlap) = overlap {
            self.pressure = Some((self.scans_seen, overlap));
        }
    }
}
