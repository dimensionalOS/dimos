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

//! Closure search as two stages of logical-execution-time jobs: scan-context buffer -> fast match -> (if uncertain) verification buffer -> rounds. Jobs land at dispatch + ceil(estimate) scans (deterministic) or on finish (fast).

use crate::gsc_pgo::{ClosureSearch, ClosureSearchResult, GscPgo, SearchStage};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Condvar, Mutex};

/// Extension rounds started, their summed estimates (scans), and the outcome once finished.
#[derive(Default)]
struct ProgressState {
    rounds: usize,
    round_scans: f64,
    stage: Option<SearchStage>,
}

/// A search's progress, shared with the thread running it.
#[derive(Default)]
pub struct SearchProgress {
    state: Mutex<ProgressState>,
    changed: Condvar,
}

impl SearchProgress {
    pub(crate) fn round_started(&self, estimate_scans: f64) {
        let mut state = self.state.lock().expect("search progress");
        state.rounds += 1;
        state.round_scans += estimate_scans;
        drop(state);
        self.changed.notify_all();
    }
    fn rounds(&self) -> (usize, f64) {
        let state = self.state.lock().expect("search progress");
        (state.rounds, state.round_scans)
    }
    fn finish(&self, stage: SearchStage) {
        self.state.lock().expect("search progress").stage = Some(stage);
        self.changed.notify_all();
    }
    /// (rounds started, finished).
    fn snapshot(&self) -> (usize, bool) {
        let state = self.state.lock().expect("search progress");
        (state.rounds, state.stage.is_some())
    }
    fn take_stage(&self) -> Option<SearchStage> {
        self.state.lock().expect("search progress").stage.take()
    }
    /// Block until the progress differs from `seen`.
    fn wait_past(&self, seen: (usize, bool)) {
        let state = self.state.lock().expect("search progress");
        let _unchanged = self
            .changed
            .wait_while(state, |now| (now.rounds, now.stage.is_some()) == seen)
            .expect("search progress");
    }
}

pub(crate) type Work = Box<dyn FnOnce() + Send>;

/// Threads that take work in the order it was sent.
pub struct SearchPool {
    jobs: Sender<Work>,
    _workers: Vec<std::thread::JoinHandle<()>>,
}

impl SearchPool {
    pub(crate) fn new(workers: usize, name: &str) -> SearchPool {
        let (jobs, queue) = channel::<Work>();
        let queue: Arc<Mutex<Receiver<Work>>> = Arc::new(Mutex::new(queue));
        let workers = (0..workers.max(1))
            .map(|i| {
                let queue = queue.clone();
                std::thread::Builder::new()
                    .name(format!("{name}-{i}"))
                    .spawn(move || loop {
                        let next = queue.lock().expect("search queue").recv();
                        let Ok(work) = next else {
                            return;
                        };
                        work();
                    })
                    .expect("spawn closure search worker")
            })
            .collect();
        SearchPool {
            jobs,
            _workers: workers,
        }
    }
    pub(crate) fn send(&self, work: Work) {
        self.jobs
            .send(work)
            .expect("closure search workers stopped");
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum JobKind {
    FastMatch,
    Verification,
}

/// A dispatched job: dispatch number, advancing scans and data time at dispatch, raw estimate, earlier stage's seconds.
pub struct SearchJob {
    kind: JobKind,
    progress: Arc<SearchProgress>,
    order: u64,
    dispatch_scan: usize,
    /// `scans_seen` at dispatch, for diagnostics.
    dispatch_scans_seen: usize,
    dispatch_time: f64,
    estimate_raw_scans: f64,
    earlier_s: f64,
    /// Its hold passed while still running in a non-waiting run (lands late).
    late: std::cell::Cell<bool>,
}

impl SearchJob {
    /// Raw estimate (scans) including verification rounds started; `rounds` is the count the caller read.
    fn estimate_raw_scans(&self, rounds: usize) -> f64 {
        let (counted, round_scans) = self.progress.rounds();
        // newer rounds only raise the hold, so reading the newer sum is safe
        let round_scans = if counted >= rounds { round_scans } else { 0.0 };
        match self.kind {
            JobKind::FastMatch => self.estimate_raw_scans,
            JobKind::Verification => self.estimate_raw_scans + round_scans,
        }
    }
    fn estimate_scans(&self, rounds: usize) -> usize {
        hold_scans(self.estimate_raw_scans(rounds))
    }
}

impl GscPgo {
    /// Per keyframe: land due jobs, buffer this keyframe's proposal, and fill free workers.
    pub fn step_closure_search(&mut self) {
        self.land_due_closure_searches(false);
        self.propose_closure();
        self.buffer_waiting_proposal();
        self.fill_workers();
    }
    /// End of data: run and land every outstanding and buffered job, waiting for each.
    pub fn land_remaining_closure_search(&mut self) {
        loop {
            if !self.closure_searches.is_empty() {
                self.land_due_closure_searches(true);
            }
            self.fill_workers();
            if self.closure_searches.is_empty() {
                return;
            }
        }
    }
    /// Whether a closure search is running, held or buffered.
    pub fn closure_search_running(&self) -> bool {
        !self.closure_searches.is_empty()
            || !self.sc_buffer.is_empty()
            || !self.verification_buffer.is_empty()
    }
    fn jobs_of(&self, kind: JobKind) -> usize {
        self.closure_searches
            .iter()
            .filter(|job| job.kind == kind)
            .count()
    }
    /// Move the waiting proposal (new match or a rejected search's next alternative) into the scan-context buffer.
    fn buffer_waiting_proposal(&mut self) {
        if let Some(proposal) = self.waiting_proposal.take() {
            self.sc_arrivals += 1;
            self.sc_buffer.push((proposal, self.sc_arrivals));
        }
    }
    /// Free workers take the best buffered entries, then each buffer drops its worst past its size.
    fn fill_workers(&mut self) {
        while self.jobs_of(JobKind::FastMatch) < self.config.fast_match_workers
            && !self.sc_buffer.is_empty()
        {
            // lowest scan-context distance, ties by arrival
            let best = (0..self.sc_buffer.len())
                .min_by(|&a, &b| {
                    let (pa, oa) = (&self.sc_buffer[a].0, self.sc_buffer[a].1);
                    let (pb, ob) = (&self.sc_buffer[b].0, self.sc_buffer[b].1);
                    pa.similarity.total_cmp(&pb.similarity).then(oa.cmp(&ob))
                })
                .expect("not empty");
            let (proposal, _) = self.sc_buffer.remove(best);
            self.waiting_proposal = Some(proposal);
            self.dispatch_closure_search(JobKind::FastMatch);
        }
        while self.sc_buffer.len() > self.config.sc_buffer_size {
            let worst = (0..self.sc_buffer.len())
                .max_by(|&a, &b| {
                    let (pa, oa) = (&self.sc_buffer[a].0, self.sc_buffer[a].1);
                    let (pb, ob) = (&self.sc_buffer[b].0, self.sc_buffer[b].1);
                    pa.similarity.total_cmp(&pb.similarity).then(oa.cmp(&ob))
                })
                .expect("not empty");
            let (proposal, _) = self.sc_buffer.remove(worst);
            let mut diag = proposal.diag;
            diag.outcome = "sc_dropped";
            self.record_search(diag);
        }
        while self.jobs_of(JobKind::Verification) < self.config.verification_workers
            && !self.verification_buffer.is_empty()
        {
            let best = self.verification_rank(true);
            let (pending, order) = self.verification_buffer.remove(best);
            self.dispatch_verification(pending, order);
        }
        while self.verification_buffer.len() > self.config.verification_buffer_slots {
            let worst = self.verification_rank(false);
            let (pending, _) = self.verification_buffer.remove(worst);
            self.finish_closure_search(ClosureSearchResult::dropped(*pending));
        }
    }
    /// Index of the best (or worst) verification entry by confidence ratio, earlier dispatch winning ties.
    fn verification_rank(&self, best: bool) -> usize {
        let key = |i: usize| {
            (
                self.verification_buffer[i].0.ratio,
                std::cmp::Reverse(self.verification_buffer[i].1),
            )
        };
        let cmp = |a: &usize, b: &usize| {
            let (ka, kb) = (key(*a), key(*b));
            ka.0.total_cmp(&kb.0).then(ka.1.cmp(&kb.1))
        };
        if best {
            (0..self.verification_buffer.len())
                .max_by(cmp)
                .expect("not empty")
        } else {
            (0..self.verification_buffer.len())
                .min_by(cmp)
                .expect("not empty")
        }
    }
    fn dispatch_closure_search(&mut self, kind: JobKind) {
        let Some(mut search) = self.begin_closure_search() else {
            return;
        };
        let progress = Arc::new(SearchProgress::default());
        search.progress = Some(progress.clone());
        let (estimate_raw_scans, source_points, target_points) =
            self.fast_match_estimate_raw_scans(&search);
        search.diag.dispatch_source_points = source_points;
        search.diag.dispatch_target_points = target_points;
        let finish = progress.clone();
        let workers = self.config.fast_match_workers;
        self.search_pool
            .get_or_insert_with(|| SearchPool::new(workers, "gsc-fast-match"))
            .send(Box::new(move || finish.finish(search.run_stage())));
        self.search_order += 1;
        let order = self.search_order;
        self.push_job(kind, progress, order, estimate_raw_scans, 0.0);
    }
    /// `order`: its fast match's dispatch number.
    fn dispatch_verification(
        &mut self,
        mut pending: Box<crate::gsc_pgo::PendingExtension>,
        order: u64,
    ) {
        let progress = Arc::new(SearchProgress::default());
        pending.search.progress = Some(progress.clone());
        pending.diag.verification_scan_index_dispatched = self.scans_seen;
        let earlier_s = pending.fast_s;
        let finish = progress.clone();
        let workers = self.config.verification_workers;
        self.verification_pool
            .get_or_insert_with(|| SearchPool::new(workers, "gsc-verification"))
            .send(Box::new(move || {
                finish.finish(SearchStage::Done(pending.run()))
            }));
        let estimate_raw_scans = self
            .config
            .deterministic_config
            .verification_scans_fixed_overhead;
        self.push_job(
            JobKind::Verification,
            progress,
            order,
            estimate_raw_scans,
            earlier_s,
        );
    }
    fn push_job(
        &mut self,
        kind: JobKind,
        progress: Arc<SearchProgress>,
        order: u64,
        estimate_raw_scans: f64,
        earlier_s: f64,
    ) {
        self.closure_searches.push_back(SearchJob {
            kind,
            progress,
            order,
            dispatch_scan: self.advancing_scans,
            dispatch_scans_seen: self.scans_seen,
            dispatch_time: self.newest_scan_time(),
            estimate_raw_scans,
            earlier_s,
            late: std::cell::Cell::new(false),
        });
    }
    /// Whether `job`'s hold has passed with `rounds` rounds started.
    fn hold_passed(&self, job: &SearchJob, rounds: usize) -> bool {
        self.advancing_scans >= job.dispatch_scan + job.estimate_scans(rounds)
    }
    /// Land finished jobs whose hold passed in due order (`now`: all, blocking on unfinished); returns count.
    fn land_due_closure_searches(&mut self, now: bool) -> usize {
        let hold = self.config.deterministic() && !now;
        // (due scan, dispatch number, kind)
        let mut due: Vec<(usize, u64, u8)> = Vec::new();
        for job in &self.closure_searches {
            loop {
                let seen = job.progress.snapshot();
                let (rounds, done) = seen;
                if hold && !self.hold_passed(job, rounds) {
                    break;
                }
                if done {
                    due.push((
                        job.dispatch_scan + job.estimate_scans(rounds),
                        job.order,
                        job.kind as u8,
                    ));
                    break;
                }
                if !now {
                    job.late.set(hold);
                    break;
                }
                job.progress.wait_past(seen);
            }
        }
        due.sort();
        for &(_, order, kind) in &due {
            let index = self
                .closure_searches
                .iter()
                .position(|job| job.order == order && job.kind as u8 == kind)
                .expect("a due job is out");
            let job = self
                .closure_searches
                .remove(index)
                .expect("index found above");
            self.land_closure_search(job);
            // a rejected search's next alternative joins the scan-context buffer as its result lands
            self.buffer_waiting_proposal();
        }
        self.trim_search_scan_times();
        due.len()
    }
    /// Apply a finished job: record schedule and overshoot, then finish it or buffer it for verification.
    fn land_closure_search(&mut self, job: SearchJob) {
        let (rounds, _) = job.progress.snapshot();
        let estimate_scans = job.estimate_scans(rounds);
        let estimate_raw_scans = job.estimate_raw_scans(rounds);
        let mut stage = job
            .progress
            .take_stage()
            .expect("a landing job has finished");
        let handed_on_s = match &stage {
            SearchStage::Extend(pending) => Some(pending.fast_s),
            SearchStage::Done(_) => None,
        };
        let diag = match &mut stage {
            SearchStage::Done(result) => &mut result.diag,
            SearchStage::Extend(pending) => &mut pending.diag,
        };
        // this job's own seconds (lookup + search, up to a hand-off)
        let lookup = if job.kind == JobKind::Verification || !diag.cost_lookup_s.is_finite() {
            0.0
        } else {
            diag.cost_lookup_s
        };
        let seconds = lookup + handed_on_s.unwrap_or(diag.cost_search_s - job.earlier_s);
        // data time from dispatch to the hold's end scan (None if data ended first)
        let due = (job.dispatch_scan + estimate_scans).saturating_sub(self.search_scan_base);
        let spanned = self
            .search_scan_times
            .get(due)
            .map(|time| time - job.dispatch_time);
        self.record_job_ratio(
            match job.kind {
                JobKind::Verification => "verification",
                JobKind::FastMatch => "fast_match",
            },
            seconds,
            estimate_raw_scans,
        );
        // ran past the data time its hold spanned, or was still running when due
        let overshoot = self.config.deterministic()
            && (spanned.is_some_and(|spanned| seconds > spanned) || job.late.get());
        if job.kind == JobKind::Verification {
            diag.verification_estimate_raw_scans = estimate_raw_scans;
            diag.verification_estimate_scans = estimate_scans;
            diag.verification_overshoot = overshoot;
        } else {
            diag.scan_index_dispatched = job.dispatch_scans_seen;
            diag.estimate_raw_scans = estimate_raw_scans;
            diag.estimate_scans = estimate_scans;
            diag.search_overshoot = overshoot;
        }
        self.note_overshoot(
            match job.kind {
                JobKind::Verification => "verification",
                JobKind::FastMatch => "fast_match",
            },
            overshoot,
        );
        match stage {
            SearchStage::Done(result) => self.finish_closure_search(result),
            SearchStage::Extend(pending) => self.verification_buffer.push((pending, job.order)),
        }
    }
    /// Drop the scan times older than every out job's dispatch.
    fn trim_search_scan_times(&mut self) {
        let Some(oldest) = self
            .closure_searches
            .iter()
            .map(|job| job.dispatch_scan)
            .min()
        else {
            self.search_scan_times.clear();
            return;
        };
        while self.search_scan_base <= oldest && !self.search_scan_times.is_empty() {
            self.search_scan_times.pop_front();
            self.search_scan_base += 1;
        }
    }
    /// Fast-match estimate (raw scans) at dispatch, with the source and target points it counted.
    fn fast_match_estimate_raw_scans(&self, search: &ClosureSearch) -> (f64, usize, usize) {
        let config = &self.config;
        let resolution = config.submap_resolution;
        let points = |k: usize| {
            search
                .key_poses
                .get(k)
                .body_cloud
                .as_ref()
                .map_or(0, |body| {
                    self.keyframe_clouds
                        .get(k, body)
                        .voxelized(resolution)
                        .len()
                })
        };
        let (source_idx, target_idx) = (search.diag.source_id, search.target_idx);
        let half_range = config.target_half_range(1);
        let source_points = points(source_idx);
        let target_points = (target_idx.saturating_sub(half_range)
            ..=(target_idx + half_range).min(search.key_poses.len() - 1))
            .map(points)
            .sum::<usize>();
        let (source, target) = (source_points.max(1) as f64, target_points.max(2) as f64);
        // lookup grows with the map (none for a retry)
        let lookup = if search.diag.cost_lookup_s > 0.0 {
            config.deterministic_config.fast_match_scans_per_keyframe
                * search.diag.map_keyframes as f64
        } else {
            0.0
        };
        let scans = config.deterministic_config.fast_match_scans_fixed_overhead
            + lookup
            + config.deterministic_config.fast_match_scans_per_point * (source + target)
            + config
                .deterministic_config
                .fast_match_scans_per_source_point
                * source;
        (scans, source_points, target_points)
    }
    /// Newest advancing scan's data time (-inf before any).
    pub(crate) fn newest_scan_time(&self) -> f64 {
        self.newest_advancing_time.unwrap_or(f64::NEG_INFINITY)
    }
    /// Record measured scans (seconds x scan rate) / raw estimate for `runtime_profile`; diagnostics only.
    pub(crate) fn record_job_ratio(
        &mut self,
        kind: &'static str,
        seconds: f64,
        estimate_raw_scans: f64,
    ) {
        let rate = match self.first_advancing_time {
            Some(first) if self.newest_scan_time() > first => {
                (self.advancing_scans - 1) as f64 / (self.newest_scan_time() - first)
            }
            _ => f64::NAN,
        };
        if estimate_raw_scans > 0.0 && (seconds * rate).is_finite() {
            self.job_ratios
                .push((kind, seconds * rate / estimate_raw_scans));
        }
    }
    /// `deterministic_config` with each LET's coefficients scaled by its p99 measured/estimated ratio (diagnostics).
    pub fn suggested_deterministic_config(&self) -> crate::gsc_pgo::DeterministicConfig {
        let mut suggested = self.config.deterministic_config.clone();
        for (_, ratio, _, coefficients) in self.runtime_profile() {
            if !ratio.is_finite() {
                continue;
            }
            for (field, value) in coefficients {
                let slot = match field {
                    "fast_match_scans_fixed_overhead" => {
                        &mut suggested.fast_match_scans_fixed_overhead
                    }
                    "fast_match_scans_per_keyframe" => &mut suggested.fast_match_scans_per_keyframe,
                    "fast_match_scans_per_point" => &mut suggested.fast_match_scans_per_point,
                    "fast_match_scans_per_source_point" => {
                        &mut suggested.fast_match_scans_per_source_point
                    }
                    "verification_scans_fixed_overhead" => {
                        &mut suggested.verification_scans_fixed_overhead
                    }
                    "verification_scans_per_round" => &mut suggested.verification_scans_per_round,
                    "verification_scans_per_round_point" => {
                        &mut suggested.verification_scans_per_round_point
                    }
                    "solve_scans_scale" => &mut suggested.solve_scans_scale,
                    "solve_closure_free_scans_scale" => {
                        &mut suggested.solve_closure_free_scans_scale
                    }
                    "debounce_scans_per_keyframe" => &mut suggested.debounce_scans_per_keyframe,
                    "merge_scans_per_keyframe" => &mut suggested.merge_scans_per_keyframe,
                    "pressure_scans_fixed_overhead" => &mut suggested.pressure_scans_fixed_overhead,
                    "pressure_scans_per_keyframe" => &mut suggested.pressure_scans_per_keyframe,
                    "pressure_scans_per_closure" => &mut suggested.pressure_scans_per_closure,
                    "fast_path_scans_fixed_overhead" => {
                        &mut suggested.fast_path_scans_fixed_overhead
                    }
                    "fast_path_scans_per_window_keyframe" => {
                        &mut suggested.fast_path_scans_per_window_keyframe
                    }
                    "fast_path_scans_per_keyframe" => &mut suggested.fast_path_scans_per_keyframe,
                    _ => continue,
                };
                *slot = value;
            }
        }
        suggested
    }
    pub fn runtime_profile(&self) -> Vec<RuntimeProfileRow> {
        let percentile = |mut values: Vec<f64>| -> f64 {
            values.retain(|v| v.is_finite());
            if values.is_empty() {
                return f64::NAN;
            }
            values.sort_by(f64::total_cmp);
            values[((values.len() - 1) as f64 * 0.99).round() as usize]
        };
        let c = &self.config;
        let groups: [ProfileGroup; 7] = [
            (
                "fast_match",
                vec![
                    (
                        "fast_match_scans_fixed_overhead",
                        c.deterministic_config.fast_match_scans_fixed_overhead,
                    ),
                    (
                        "fast_match_scans_per_keyframe",
                        c.deterministic_config.fast_match_scans_per_keyframe,
                    ),
                    (
                        "fast_match_scans_per_point",
                        c.deterministic_config.fast_match_scans_per_point,
                    ),
                    (
                        "fast_match_scans_per_source_point",
                        c.deterministic_config.fast_match_scans_per_source_point,
                    ),
                ],
            ),
            (
                "verification",
                vec![
                    (
                        "verification_scans_fixed_overhead",
                        c.deterministic_config.verification_scans_fixed_overhead,
                    ),
                    (
                        "verification_scans_per_round",
                        c.deterministic_config.verification_scans_per_round,
                    ),
                    (
                        "verification_scans_per_round_point",
                        c.deterministic_config.verification_scans_per_round_point,
                    ),
                ],
            ),
            (
                "solve",
                vec![
                    (
                        "solve_scans_scale",
                        c.deterministic_config.solve_scans_scale,
                    ),
                    (
                        "solve_closure_free_scans_scale",
                        c.deterministic_config.solve_closure_free_scans_scale,
                    ),
                ],
            ),
            (
                "debounce",
                vec![(
                    "debounce_scans_per_keyframe",
                    c.deterministic_config.debounce_scans_per_keyframe,
                )],
            ),
            (
                "merge",
                vec![(
                    "merge_scans_per_keyframe",
                    c.deterministic_config.merge_scans_per_keyframe,
                )],
            ),
            (
                "pressure",
                vec![
                    (
                        "pressure_scans_fixed_overhead",
                        c.deterministic_config.pressure_scans_fixed_overhead,
                    ),
                    (
                        "pressure_scans_per_keyframe",
                        c.deterministic_config.pressure_scans_per_keyframe,
                    ),
                    (
                        "pressure_scans_per_closure",
                        c.deterministic_config.pressure_scans_per_closure,
                    ),
                ],
            ),
            (
                "fast_path",
                vec![
                    (
                        "fast_path_scans_fixed_overhead",
                        c.deterministic_config.fast_path_scans_fixed_overhead,
                    ),
                    (
                        "fast_path_scans_per_window_keyframe",
                        c.deterministic_config.fast_path_scans_per_window_keyframe,
                    ),
                    (
                        "fast_path_scans_per_keyframe",
                        c.deterministic_config.fast_path_scans_per_keyframe,
                    ),
                ],
            ),
        ];
        groups
            .into_iter()
            .map(|(name, coefficients)| {
                let ratios: Vec<f64> = self
                    .job_ratios
                    .iter()
                    .filter(|(k, _)| *k == name)
                    .map(|&(_, ratio)| ratio)
                    .collect();
                let count = ratios.len();
                let ratio = percentile(ratios);
                let suggested = coefficients
                    .into_iter()
                    .map(|(field, value)| (field, value * ratio))
                    .collect();
                (name, ratio, count, suggested)
            })
            .collect()
    }
}

/// Scans a job holds: its estimate rounded up, at least one.
pub(crate) fn hold_scans(estimate_raw_scans: f64) -> usize {
    if estimate_raw_scans.is_finite() {
        (estimate_raw_scans.ceil() as usize).max(1)
    } else {
        1
    }
}

/// `runtime_profile` row: job kind, p99 measured/estimated ratio, job count, suggested coefficients.
pub type RuntimeProfileRow = (&'static str, f64, usize, Vec<(&'static str, f64)>);
/// Job kind and its coefficients.
type ProfileGroup = (&'static str, Vec<(&'static str, f64)>);
