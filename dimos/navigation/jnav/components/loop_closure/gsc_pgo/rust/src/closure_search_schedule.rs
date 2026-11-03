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

//! The closure search, a two-stage pipeline of logical-execution-time jobs on worker pools: every scan-context candidate
//! and retry alternative waits in a priority buffer (`sc_buffer_size`, the most similar first, ties by arrival; the
//! least similar dropped when it overflows); each fast-match worker (`fast_match_workers`, one job at a time logically)
//! takes the best as it frees, at its job's landing. A fast match decides a clear pass or fail; a match inside the
//! uncertainty range enters the verification buffer at its landing (`verification_buffer_slots`, the highest single-scan
//! confidence ratio first, ties by dispatch; the lowest dropped when it overflows), for a verification worker
//! (`verification_workers`, one job at a time logically) to take as it frees: its extension rounds and their yank
//! verdict, one job that counts each round's estimate as the round starts (each round's continue-or-stop reads only the
//! job's own inputs). A job reads only its snapshot (taken at its dispatch), so what an in-flight job finds never
//! reaches another until it lands.
//!
//! Deterministic mode (`Config::mode`): a job lands at its dispatch + `ceil(estimate)` advancing scans, the due ones in
//! due-scan order (ties by dispatch order), so which jobs run and when their results land does not depend on the
//! machine's speed. Fast mode: a job lands as soon as it has finished.

use crate::gsc_pgo::{ClosureSearch, ClosureSearchResult, GscPgo, SearchStage};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Condvar, Mutex};

/// What a search has reported: the extension rounds it has started, and its outcome once it has finished.
#[derive(Default)]
struct ProgressState {
    rounds: usize,
    /// The extension rounds' estimates (scans), each counted as its round starts.
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
    /// The rounds started and their estimates' sum (scans).
    fn rounds(&self) -> (usize, f64) {
        let state = self.state.lock().expect("search progress");
        (state.rounds, state.round_scans)
    }
    fn finish(&self, stage: SearchStage) {
        self.state.lock().expect("search progress").stage = Some(stage);
        self.changed.notify_all();
    }
    /// The rounds started and whether it has finished.
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
        self.jobs.send(work).expect("closure search workers stopped");
    }
}

/// What a job runs: a fast match or a verification.
#[derive(Clone, Copy, PartialEq, Eq)]
enum JobKind {
    FastMatch,
    Verification,
}

/// A dispatched job and its schedule: its dispatch number, the advancing-scan count (`GscPgo::advancing_scans`) and
/// recorded time at its dispatch, its estimate at dispatch (scans, before rounding up; each extension round adds
/// `verification_scans_per_round` as it starts), and the seconds
/// of the search before this job (a verification's fast match), left out of its own.
pub struct SearchJob {
    kind: JobKind,
    progress: Arc<SearchProgress>,
    order: u64,
    dispatch_scan: usize,
    /// `scans_seen` at dispatch, for the diagnostics (whose scan indices count every scan).
    dispatch_scans_seen: usize,
    dispatch_time: f64,
    estimate_raw_scans: f64,
    earlier_s: f64,
    /// Its hold passed while it was still running, in a run that does not wait (live: it lands late).
    late: std::cell::Cell<bool>,
}

impl SearchJob {
    /// The estimate (scans, before rounding up) with the extension rounds started so far (each round's own estimate,
    /// counted as it starts: `verification_scans_per_round` + `verification_scans_per_round_point` x its source and
    /// target points); `rounds` is the count those were read at.
    fn estimate_raw_scans(&self, rounds: usize, config: &crate::gsc_pgo::Config) -> f64 {
        let (counted, round_scans) = self.progress.rounds();
        // rounds started after `rounds` was read only raise the hold, never lower it, so reading the newer sum is safe
        let round_scans = if counted >= rounds { round_scans } else { 0.0 };
        let _ = config;
        match self.kind {
            JobKind::FastMatch => self.estimate_raw_scans,
            JobKind::Verification => self.estimate_raw_scans + round_scans,
        }
    }
    /// The scans after dispatch it holds (`hold_scans`).
    fn estimate_scans(&self, rounds: usize, config: &crate::gsc_pgo::Config) -> usize {
        hold_scans(self.estimate_raw_scans(rounds, config), config)
    }
}

impl GscPgo {
    /// One keyframe's closure search: land the jobs that may land (deterministic: their holds passed, a lockstep replay
    /// waiting for one due then when `wait`; fast: finished), then propose this keyframe's match into the scan-context
    /// buffer and fill the free workers from the buffers. Every proposal is buffered whatever the load (under a compute
    /// shortage jobs overshoot, or the buffers drop their least promising).
    pub fn step_closure_search(&mut self, wait: bool) {
        self.land_due_closure_searches(wait, false);
        self.propose_closure();
        self.buffer_waiting_proposal();
        self.fill_workers();
    }
    /// Land every job still out and run every buffered match whatever its estimate, waiting for each (the end of the
    /// data).
    pub fn land_remaining_closure_search(&mut self) {
        loop {
            if !self.closure_searches.is_empty() {
                self.land_due_closure_searches(true, true);
            }
            self.fill_workers();
            if self.closure_searches.is_empty() {
                return;
            }
        }
    }
    /// Searches (whole, or fast matches) and verifications that ran past the data time their estimate spanned, the
    /// seconds a lockstep replay waited on jobs whose estimate had passed, and what the two-stage buffers dropped.
    pub fn closure_search_overshoots(&self) -> usize {
        self.closure_search_overshoots
    }
    pub fn verification_overshoots(&self) -> usize {
        self.verification_overshoots
    }
    pub fn closure_search_wait_s(&self) -> f64 {
        self.closure_search_wait_s
    }
    pub fn buffer_drops(&self) -> (usize, usize) {
        (self.sc_dropped, self.verification_dropped)
    }
    /// Whether a closure search is running, held for its estimate or buffered.
    pub fn closure_search_running(&self) -> bool {
        !self.closure_searches.is_empty()
            || !self.sc_buffer.is_empty()
            || !self.verification_buffer.is_empty()
    }
    fn jobs_of(&self, kind: JobKind) -> usize {
        self.closure_searches.iter().filter(|job| job.kind == kind).count()
    }
    /// The two-stage search's waiting proposal (this keyframe's match, or a rejected search's next alternative) goes into
    /// the scan-context buffer.
    fn buffer_waiting_proposal(&mut self) {
        if let Some(proposal) = self.waiting_proposal.take() {
            self.sc_arrivals += 1;
            self.sc_buffer.push((proposal, self.sc_arrivals));
        }
    }
    /// The two-stage search's free workers take the best buffered entries, then each buffer drops its least promising
    /// past its size.
    fn fill_workers(&mut self) {
        while self.jobs_of(JobKind::FastMatch) < self.config.fast_match_workers && !self.sc_buffer.is_empty() {
            // the most similar candidate (the lowest scan-context distance), ties by arrival
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
            self.sc_dropped += 1;
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
            self.verification_dropped += 1;
            self.finish_closure_search(ClosureSearchResult::dropped(*pending));
        }
    }
    /// The verification buffer's most (`best`) or least promising entry: by the single scan's confidence ratio, ties
    /// by dispatch (the earlier first).
    fn verification_rank(&self, best: bool) -> usize {
        let key = |i: usize| (self.verification_buffer[i].0.ratio, std::cmp::Reverse(self.verification_buffer[i].1));
        let cmp = |a: &usize, b: &usize| {
            let (ka, kb) = (key(*a), key(*b));
            ka.0.total_cmp(&kb.0).then(ka.1.cmp(&kb.1))
        };
        if best {
            (0..self.verification_buffer.len()).max_by(cmp).expect("not empty")
        } else {
            (0..self.verification_buffer.len()).min_by(cmp).expect("not empty")
        }
    }
    /// Dispatch the waiting proposal's fast match onto the fast-match pool.
    fn dispatch_closure_search(&mut self, kind: JobKind) {
        let Some(mut search) = self.begin_closure_search() else {
            return;
        };
        let progress = Arc::new(SearchProgress::default());
        search.progress = Some(progress.clone());
        let (estimate_raw_scans, source_points, target_points) = self.fast_match_estimate_raw_scans(&search);
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
    /// Dispatch a buffered match's verification onto the verification pool (`order`: its fast match's dispatch number).
    fn dispatch_verification(&mut self, mut pending: Box<crate::gsc_pgo::PendingExtension>, order: u64) {
        let progress = Arc::new(SearchProgress::default());
        pending.search.progress = Some(progress.clone());
        pending.diag.verification_scan_index_dispatched = self.scans_seen;
        let earlier_s = pending.fast_s;
        let finish = progress.clone();
        let workers = self.config.verification_workers;
        self.verification_pool
            .get_or_insert_with(|| SearchPool::new(workers, "gsc-verification"))
            .send(Box::new(move || finish.finish(SearchStage::Done(pending.run()))));
        let estimate_raw_scans = self.config.deterministic_config.verification_scans_fixed_overhead;
        self.push_job(JobKind::Verification, progress, order, estimate_raw_scans, earlier_s);
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
    /// Whether `job`'s hold has passed with `rounds` rounds started: its scans (`hold_scans`).
    fn hold_passed(&self, job: &SearchJob, rounds: usize) -> bool {
        self.advancing_scans >= job.dispatch_scan + job.estimate_scans(rounds, &self.config)
    }
    /// Land the jobs whose hold has passed (all, when `now`) and that have finished, waiting for one to finish or report
    /// another extension round when `wait`, in due order (ties by dispatch order, a fast match before its own
    /// verification never being out at once). Returns how many landed.
    fn land_due_closure_searches(&mut self, wait: bool, now: bool) -> usize {
        let hold = self.config.deterministic() && !now;
        // each finished job whose hold has passed, by its due scan, dispatch number and kind
        let mut due: Vec<(usize, u64, u8)> = Vec::new();
        let mut waited_s = 0.0;
        for job in &self.closure_searches {
            loop {
                let seen = job.progress.snapshot();
                let (rounds, done) = seen;
                if hold && !self.hold_passed(job, rounds) {
                    break;
                }
                if done {
                    due.push((
                        job.dispatch_scan + job.estimate_scans(rounds, &self.config),
                        job.order,
                        job.kind as u8,
                    ));
                    break;
                }
                if !wait {
                    job.late.set(hold);
                    break;
                }
                // the replay ran ahead of the job: wait for it to finish or start another round (which moves its hold)
                let started = std::time::Instant::now();
                job.progress.wait_past(seen);
                waited_s += started.elapsed().as_secs_f64();
            }
        }
        self.closure_search_wait_s += waited_s;
        due.sort();
        for &(_, order, kind) in &due {
            let index = self
                .closure_searches
                .iter()
                .position(|job| job.order == order && job.kind as u8 == kind)
                .expect("a due job is out");
            let job = self.closure_searches.remove(index).expect("index found above");
            self.land_closure_search(job);
            // a rejected search's next alternative joins the scan-context buffer as its result lands
            self.buffer_waiting_proposal();
        }
        self.trim_search_scan_times();
        due.len()
    }
    /// Apply a finished job: its schedule into its diagnostics, an overshoot logged and counted, and its result (or, for
    /// a fast match inside the uncertainty range, the match into the verification buffer).
    fn land_closure_search(&mut self, job: SearchJob) {
        let (rounds, _) = job.progress.snapshot();
        let estimate_scans = job.estimate_scans(rounds, &self.config);
        let estimate_raw_scans = job.estimate_raw_scans(rounds, &self.config);
        let stage = job.progress.take_stage().expect("a landing job has finished");
        let mut stage = stage;
        let handed_on_s = match &stage {
            SearchStage::Extend(pending) => Some(pending.fast_s),
            SearchStage::Done(_) => None,
        };
        let diag = match &mut stage {
            SearchStage::Done(result) => &mut result.diag,
            SearchStage::Extend(pending) => &mut pending.diag,
        };
        // this job's own seconds: the lookup and the search (a fast match's, inside the uncertainty range, until its hand-off)
        let lookup = if job.kind == JobKind::Verification || !diag.cost_lookup_s.is_finite() {
            0.0
        } else {
            diag.cost_lookup_s
        };
        let seconds = lookup + handed_on_s.unwrap_or(diag.cost_search_s - job.earlier_s);
        // the data time from dispatch to the scan its hold ended at (none when the data ended first): advancing scan n is
        // `search_scan_times[n - search_scan_base]`
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
        // its own work ran past the data time its hold spanned, or (live) it was still running, queued or not, when due
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
        if overshoot {
            let count = if job.kind == JobKind::Verification {
                self.verification_overshoots += 1;
                self.verification_overshoots
            } else {
                self.closure_search_overshoots += 1;
                self.closure_search_overshoots
            };
            eprintln!(
                "gsc_pgo: CLOSURE {} OVERSHOOT #{count}: keyframe {} onto {} took {seconds:.3}s, past the {:.3}s of data its estimate of {estimate_raw_scans:.3} vs ({estimate_scans} scans) spanned",
                match job.kind {
                    JobKind::Verification => "VERIFICATION",
                    JobKind::FastMatch => "FAST MATCH",
                },
                diag.source_id,
                diag.target_id,
                spanned.unwrap_or(f64::NAN),
            );
        }
        match stage {
            SearchStage::Done(result) => self.finish_closure_search(result),
            // inside the uncertainty range: waits for a verification worker, ranked by its confidence ratio
            SearchStage::Extend(pending) => self.verification_buffer.push((pending, job.order)),
        }
    }
    /// Drop the scan times older than every out job's dispatch.
    fn trim_search_scan_times(&mut self) {
        let Some(oldest) = self.closure_searches.iter().map(|job| job.dispatch_scan).min() else {
            self.search_scan_times.clear();
            return;
        };
        // the first time kept is advancing scan oldest + 1
        while self.search_scan_base <= oldest && !self.search_scan_times.is_empty() {
            self.search_scan_times.pop_front();
            self.search_scan_base += 1;
        }
    }
    /// The search's estimated cost at dispatch (seconds): the scan-context lookup (none for a retry), building the
    /// source scan and single target submap, the degeneracy check, the single ICP at its most iterations, and the gates.
    /// Returns it with the source and target points it counted.
    fn fast_match_estimate_raw_scans(&self, search: &ClosureSearch) -> (f64, usize, usize) {
        let config = &self.config;
        let resolution = config.submap_resolution;
        let points = |k: usize| {
            search.key_poses.get(k)
                .body_cloud
                .as_ref()
                .map_or(0, |body| self.keyframe_clouds.get(k, body).voxelized(resolution).len())
        };
        let (source_idx, target_idx) = (search.diag.source_id, search.target_idx);
        let half_range = config.target_half_range(1);
        let source_points = points(source_idx);
        let target_points = (target_idx.saturating_sub(half_range)
            ..=(target_idx + half_range).min(search.key_poses.len() - 1))
            .map(points)
            .sum::<usize>();
        let (source, target) = (source_points.max(1) as f64, target_points.max(2) as f64);
        // the scan-context lookup grows with the map (none for a retry, which reuses its keyframe's)
        let lookup = if search.diag.cost_lookup_s > 0.0 {
            config.deterministic_config.fast_match_scans_per_keyframe * search.diag.map_keyframes as f64
        } else {
            0.0
        };
        let scans = config.deterministic_config.fast_match_scans_fixed_overhead
            + lookup
            + config.deterministic_config.fast_match_scans_per_point * (source + target)
            + config.deterministic_config.fast_match_scans_per_source_point * source;
        (scans, source_points, target_points)
    }
    /// The newest advancing scan's recorded time (`advancing_scans`; -inf before any).
    pub(crate) fn newest_scan_time(&self) -> f64 {
        self.newest_advancing_time.unwrap_or(f64::NEG_INFINITY)
    }
    /// Note a landed job's measured scans (its seconds x the scan rate so far, by recorded times) over its estimate
    /// (scans, before rounding up) (`runtime_profile`); never read by a decision.
    pub(crate) fn record_job_ratio(&mut self, kind: &'static str, seconds: f64, estimate_raw_scans: f64) {
        let rate = match self.first_advancing_time {
            Some(first) if self.newest_scan_time() > first => {
                (self.advancing_scans - 1) as f64 / (self.newest_scan_time() - first)
            }
            _ => f64::NAN,
        };
        if estimate_raw_scans > 0.0 && (seconds * rate).is_finite() {
            self.job_ratios.push((kind, seconds * rate / estimate_raw_scans));
        }
    }
    /// The coefficients this run measured, as a config block: each LET's coefficients scaled by the 99th percentile of
    /// its jobs' measured scans over their estimates (an upper bound on what this machine needed; nothing reads it), with
    /// that ratio and the jobs counted per LET. A whole search (one job per search) measures the fast-match and
    /// verification coefficients together.
    /// The `deterministic_config` this run suggests: each LET's coefficients scaled by its 99th-percentile measured /
    /// estimated scans (a LET with no jobs keeps its configured coefficients; the solve's exponents are kept), ready to
    /// paste; diagnostics only.
    pub fn suggested_deterministic_config(&self) -> crate::gsc_pgo::DeterministicConfig {
        let mut suggested = self.config.deterministic_config.clone();
        for (_, ratio, _, coefficients) in self.runtime_profile() {
            if !ratio.is_finite() {
                continue;
            }
            for (field, value) in coefficients {
                let slot = match field {
                    "fast_match_scans_fixed_overhead" => &mut suggested.fast_match_scans_fixed_overhead,
                    "fast_match_scans_per_keyframe" => &mut suggested.fast_match_scans_per_keyframe,
                    "fast_match_scans_per_point" => &mut suggested.fast_match_scans_per_point,
                    "fast_match_scans_per_source_point" => &mut suggested.fast_match_scans_per_source_point,
                    "verification_scans_fixed_overhead" => &mut suggested.verification_scans_fixed_overhead,
                    "verification_scans_per_round" => &mut suggested.verification_scans_per_round,
                    "verification_scans_per_round_point" => &mut suggested.verification_scans_per_round_point,
                    "solve_scans_scale" => &mut suggested.solve_scans_scale,
                    "debounce_scans_per_keyframe" => &mut suggested.debounce_scans_per_keyframe,
                    "merge_scans_per_keyframe" => &mut suggested.merge_scans_per_keyframe,
                    "pressure_scans_fixed_overhead" => &mut suggested.pressure_scans_fixed_overhead,
                    "pressure_scans_per_keyframe" => &mut suggested.pressure_scans_per_keyframe,
                    "pressure_scans_per_closure" => &mut suggested.pressure_scans_per_closure,
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
        let groups: [ProfileGroup; 6] = [
            ("fast_match", &["search", "fast_match"], vec![
                ("fast_match_scans_fixed_overhead", c.deterministic_config.fast_match_scans_fixed_overhead),
                ("fast_match_scans_per_keyframe", c.deterministic_config.fast_match_scans_per_keyframe),
                ("fast_match_scans_per_point", c.deterministic_config.fast_match_scans_per_point),
                ("fast_match_scans_per_source_point", c.deterministic_config.fast_match_scans_per_source_point),
            ]),
            ("verification", &["search", "verification"], vec![
                ("verification_scans_fixed_overhead", c.deterministic_config.verification_scans_fixed_overhead),
                ("verification_scans_per_round", c.deterministic_config.verification_scans_per_round),
                ("verification_scans_per_round_point", c.deterministic_config.verification_scans_per_round_point),
            ]),
            ("solve", &["solve"], vec![("solve_scans_scale", c.deterministic_config.solve_scans_scale)]),
            ("debounce", &["debounce"], vec![("debounce_scans_per_keyframe", c.deterministic_config.debounce_scans_per_keyframe)]),
            ("merge", &["merge"], vec![("merge_scans_per_keyframe", c.deterministic_config.merge_scans_per_keyframe)]),
            ("pressure", &["pressure"], vec![
                ("pressure_scans_fixed_overhead", c.deterministic_config.pressure_scans_fixed_overhead),
                ("pressure_scans_per_keyframe", c.deterministic_config.pressure_scans_per_keyframe),
                ("pressure_scans_per_closure", c.deterministic_config.pressure_scans_per_closure),
            ]),
        ];
        groups
            .into_iter()
            .map(|(name, kinds, coefficients)| {
                // two-stage jobs are timed by kind; with one job per search, the whole search times both groups
                let two_stage = self.job_ratios.iter().any(|(k, _)| *k == "fast_match" || *k == "verification");
                let kinds: Vec<&str> = match name {
                    "fast_match" | "verification" if !two_stage => vec!["search"],
                    "fast_match" | "verification" => vec![name],
                    _ => kinds.to_vec(),
                };
                let ratios: Vec<f64> = self
                    .job_ratios
                    .iter()
                    .filter(|(k, _)| kinds.contains(k))
                    .map(|&(_, ratio)| ratio)
                    .collect();
                let count = ratios.len();
                let ratio = percentile(ratios);
                let suggested = coefficients.into_iter().map(|(field, value)| (field, value * ratio)).collect();
                (name, ratio, count, suggested)
            })
            .collect()
    }
}

/// The advancing scans a logical-execution-time job holds: its estimate rounded up, at least one.
pub(crate) fn hold_scans(estimate_raw_scans: f64, _config: &crate::gsc_pgo::Config) -> usize {
    if estimate_raw_scans.is_finite() {
        (estimate_raw_scans.ceil() as usize).max(1)
    } else {
        1
    }
}

/// One `runtime_profile` row: a job kind, its 99th-percentile seconds, its count and its estimate's terms.
pub type RuntimeProfileRow = (&'static str, f64, usize, Vec<(&'static str, f64)>);
/// A `runtime_profile` job kind, the timing keys it sums and its estimate's terms.
type ProfileGroup = (&'static str, &'static [&'static str], Vec<(&'static str, f64)>);
