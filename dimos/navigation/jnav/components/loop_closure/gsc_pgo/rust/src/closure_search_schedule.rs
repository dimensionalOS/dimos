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

//! The deterministic closure-search schedule (`Config::closure_search_hold_enabled`): a search's result lands only once
//! its estimated cost has passed since its dispatch (in advancing scans and in data time), the estimate of each piece of
//! work fixed before that work starts (at dispatch, and each extension round's as it starts), so which searches run and
//! when their closures land does not depend on how fast the machine is.
//!
//! Logical execution time (`closure_search_let_enabled`): searches are jobs on worker pools, each landing at its own
//! dispatch + estimate; the due ones at a keyframe land in due-scan order, ties by dispatch order; a search reads only
//! its snapshot (taken at its dispatch), so what an in-flight search finds never reaches another until it lands.
//!
//! - Two-stage (`closure_search_two_stage`, the default): every scan-context candidate and retry alternative waits in a
//!   priority buffer (`sc_buffer_size`, the most similar first, ties by arrival; the least similar dropped when it
//!   overflows); each fast-match worker (`fast_match_workers`, one job at a time logically) takes the best as it frees,
//!   at its job's landing. A fast match decides a clear pass or fail; a match inside the uncertainty range enters the
//!   verification buffer at its landing (`verification_buffer_slots`, the highest single-scan confidence ratio first,
//!   ties by dispatch; the lowest dropped when it overflows), for a verification worker (`verification_workers`, one job
//!   at a time logically) to take as it frees: its extension rounds and their yank verdict, one job that counts each
//!   round's bound as the round starts, so it lands where per-round jobs would (each round's continue-or-stop depends
//!   only on the job's own inputs).
//! - Else every keyframe's search is its own job, dispatched at its keyframe, on `closure_search_workers` threads.
//! - Off: one search at a time on its own thread, the next keyframes' proposals going stale while it is held.

use crate::gsc_pgo::{ClosureSearch, ClosureSearchResult, GscPgo, SearchStage};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;

/// What a search has reported: the extension rounds it has started, and its outcome once it has finished.
#[derive(Default)]
struct ProgressState {
    rounds: usize,
    stage: Option<SearchStage>,
}

/// A search's progress, shared with the thread running it.
#[derive(Default)]
pub struct SearchProgress {
    state: Mutex<ProgressState>,
    changed: Condvar,
}

impl SearchProgress {
    pub(crate) fn round_started(&self) {
        self.state.lock().expect("search progress").rounds += 1;
        self.changed.notify_all();
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
    _workers: Vec<JoinHandle<()>>,
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

/// What a job runs: a whole search, a fast match, or a verification (`closure_search_two_stage`).
#[derive(Clone, Copy, PartialEq, Eq)]
enum JobKind {
    Search,
    FastMatch,
    Verification,
}

/// A dispatched job and its schedule: its dispatch number, the advancing-scan count (`GscPgo::advancing_scans`) and
/// recorded time at its dispatch, its estimate at dispatch (seconds), the seconds each extension round adds as it starts
/// (a whole search's: `closure_search_estimate_extension_*`), the scan period its seconds are counted in, and the seconds
/// of the search before this job (a verification's fast match), left out of its own.
pub struct SearchJob {
    kind: JobKind,
    progress: Arc<SearchProgress>,
    /// The thread of a one-at-a-time search (`closure_search_let_enabled` off).
    thread: Option<JoinHandle<()>>,
    order: u64,
    dispatch_scan: usize,
    /// `scans_seen` at dispatch, for the diagnostics (whose scan indices count every scan).
    dispatch_scans_seen: usize,
    dispatch_time: f64,
    estimate_s: f64,
    scan_period_s: f64,
    earlier_s: f64,
    /// Its hold passed while it was still running, in a run that does not wait (live: it lands late).
    late: std::cell::Cell<bool>,
}

impl SearchJob {
    /// The estimate (seconds) with `rounds` extension rounds started.
    fn estimate_s(&self, rounds: usize, config: &crate::gsc_pgo::Config) -> f64 {
        match self.kind {
            JobKind::FastMatch => self.estimate_s,
            JobKind::Verification => self.estimate_s + rounds as f64 * config.verification_estimate_round_s,
            JobKind::Search if rounds == 0 => self.estimate_s,
            JobKind::Search => {
                self.estimate_s
                    + config.closure_search_estimate_extension_overhead_s
                    + rounds as f64 * config.closure_search_estimate_extension_cloud_s
            }
        }
    }
    /// The scans after dispatch the estimate spans, at least one (a scan period unknown yet counts as one scan).
    fn estimate_scans(&self, rounds: usize, config: &crate::gsc_pgo::Config) -> usize {
        let scans = self.estimate_s(rounds, config) / self.scan_period_s;
        if scans.is_finite() {
            (scans.ceil() as usize).max(1)
        } else {
            1
        }
    }
}

impl GscPgo {
    /// One keyframe's closure searches under the deterministic schedule: land those whose estimate has passed (waiting
    /// for one then when `wait`, as a lockstep replay does; else only once it has finished), then propose this keyframe's
    /// match and dispatch what may start. Logical execution time (`closure_search_let_enabled`): every due job lands, in
    /// due order, and `may_begin` is ignored (under a compute shortage jobs overshoot rather than go unsearched); the
    /// two-stage search then fills its free workers from its buffers, else this keyframe's search is dispatched (and a
    /// rejected one's next alternative as it lands). Else one search at a time: a rejected search's next alternative
    /// goes before this keyframe's own match, and a match starts only when no search is held and `may_begin`. With
    /// `closure_search_hold_enabled` off, a search lands once it finishes.
    pub fn step_closure_search(&mut self, wait: bool, may_begin: bool) {
        if self.two_stage() {
            self.land_due_closure_searches(wait, false);
            self.propose_closure();
            self.buffer_waiting_proposal();
            self.fill_workers();
            return;
        }
        if self.let_searches() {
            self.land_due_closure_searches(wait, false);
            self.propose_closure();
            self.dispatch_closure_search(JobKind::Search);
            return;
        }
        let landed = self.land_due_closure_searches(wait, false) > 0;
        // a rejected search's next alternative goes before this keyframe's own match, as the inline search tries it
        if landed && may_begin && self.config.closure_search_hold_enabled {
            if self
                .waiting_proposal
                .as_ref()
                .is_some_and(|waiting| waiting.tries > 0)
            {
                self.dispatch_closure_search(JobKind::Search);
            }
        }
        self.propose_closure();
        if may_begin {
            self.dispatch_closure_search(JobKind::Search);
        }
    }
    /// Land every job still out and run every buffered match whatever its estimate, waiting for each (the end of the
    /// data).
    pub fn land_remaining_closure_search(&mut self) {
        loop {
            if !self.closure_searches.is_empty() {
                self.land_due_closure_searches(true, true);
            }
            if self.two_stage() {
                self.fill_workers();
            }
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
    fn let_searches(&self) -> bool {
        self.config.closure_search_hold_enabled && self.config.closure_search_let_enabled
    }
    fn two_stage(&self) -> bool {
        self.let_searches() && self.config.closure_search_two_stage
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
    /// Dispatch the waiting proposal's search, as a whole search or a fast match: onto its worker pool
    /// (`closure_search_let_enabled`), else on its own thread if no search is out.
    fn dispatch_closure_search(&mut self, kind: JobKind) {
        let pooled = self.let_searches();
        if !pooled && !self.closure_searches.is_empty() {
            return;
        }
        let Some(mut search) = self.begin_closure_search() else {
            return;
        };
        let progress = Arc::new(SearchProgress::default());
        search.progress = Some(progress.clone());
        let (estimate_s, source_points, target_points) = self.closure_search_estimate_s(&search);
        search.diag.dispatch_source_points = source_points;
        search.diag.dispatch_target_points = target_points;
        let finish = progress.clone();
        let thread = if pooled {
            let (workers, name) = if kind == JobKind::FastMatch {
                (self.config.fast_match_workers, "gsc-fast-match")
            } else {
                (self.config.closure_search_workers, "gsc-closure-search")
            };
            let work: Work = if kind == JobKind::FastMatch {
                Box::new(move || finish.finish(search.run_stage()))
            } else {
                Box::new(move || finish.finish(SearchStage::Done(search.run())))
            };
            self.search_pool
                .get_or_insert_with(|| SearchPool::new(workers, name))
                .send(work);
            None
        } else {
            Some(
                std::thread::Builder::new()
                    .name("gsc-closure-search".into())
                    .spawn(move || finish.finish(SearchStage::Done(search.run())))
                    .expect("spawn closure search thread"),
            )
        };
        self.search_order += 1;
        let order = self.search_order;
        self.push_job(kind, progress, thread, order, estimate_s, 0.0);
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
        let estimate_s = self.config.verification_estimate_overhead_s;
        self.push_job(JobKind::Verification, progress, None, order, estimate_s, earlier_s);
    }
    fn push_job(
        &mut self,
        kind: JobKind,
        progress: Arc<SearchProgress>,
        thread: Option<JoinHandle<()>>,
        order: u64,
        estimate_s: f64,
        earlier_s: f64,
    ) {
        self.closure_searches.push_back(SearchJob {
            kind,
            progress,
            thread,
            order,
            dispatch_scan: self.advancing_scans,
            dispatch_scans_seen: self.scans_seen,
            dispatch_time: self.newest_scan_time(),
            estimate_s,
            scan_period_s: self.scan_period_s(),
            earlier_s,
            late: std::cell::Cell::new(false),
        });
    }
    /// Whether `job`'s hold has passed with `rounds` rounds started: its scans and the data time its seconds span (an
    /// early scan cannot end it).
    fn hold_passed(&self, job: &SearchJob, rounds: usize) -> bool {
        self.advancing_scans >= job.dispatch_scan + job.estimate_scans(rounds, &self.config)
            && self.newest_scan_time() - job.dispatch_time >= job.estimate_s(rounds, &self.config)
    }
    /// Land the jobs whose hold has passed (all, when `now`) and that have finished, waiting for one to finish or report
    /// another extension round when `wait`, in due order (ties by dispatch order, a fast match before its own
    /// verification never being out at once). Returns how many landed.
    fn land_due_closure_searches(&mut self, wait: bool, now: bool) -> usize {
        let hold = self.config.closure_search_hold_enabled && !now;
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
            // one at a time, a later search can only be out once this one landed
            if !self.let_searches() {
                break;
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
            if self.two_stage() {
                // a rejected search's next alternative joins the scan-context buffer as its result lands
                self.buffer_waiting_proposal();
            } else if self.let_searches()
                && self
                    .waiting_proposal
                    .as_ref()
                    .is_some_and(|waiting| waiting.tries > 0)
            {
                // a rejected search's next alternative is its own job, dispatched (its snapshot taken) as its result lands
                self.dispatch_closure_search(JobKind::Search);
            }
        }
        self.trim_search_scan_times();
        due.len()
    }
    /// Apply a finished job: its schedule into its diagnostics, an overshoot logged and counted, and its result (or, for
    /// a fast match inside the uncertainty range, the match into the verification buffer).
    fn land_closure_search(&mut self, job: SearchJob) {
        let (rounds, _) = job.progress.snapshot();
        let estimate_scans = job.estimate_scans(rounds, &self.config);
        let estimate_s = job.estimate_s(rounds, &self.config);
        let stage = job.progress.take_stage().expect("a landing job has finished");
        if let Some(thread) = job.thread {
            thread.join().expect("closure search thread panicked");
        }
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
        // the data time from dispatch to the first scan its hold could end at, its scans and seconds both passed (none
        // when the data ended first)
        let first = (job.dispatch_scan + estimate_scans).saturating_sub(self.search_scan_base);
        let spanned = self
            .search_scan_times
            .iter()
            .skip(first)
            .map(|time| time - job.dispatch_time)
            .find(|spanned| *spanned >= estimate_s);
        // its own work ran past the data time its hold spanned, or (live) it was still running, queued or not, when due
        let overshoot = self.config.closure_search_hold_enabled
            && (spanned.is_some_and(|spanned| seconds > spanned) || job.late.get());
        if job.kind == JobKind::Verification {
            diag.verification_estimate_s = estimate_s;
            diag.verification_estimate_scans = estimate_scans;
            diag.verification_overshoot = overshoot;
        } else {
            diag.scan_index_dispatched = job.dispatch_scans_seen;
            diag.estimate_s = estimate_s;
            diag.estimate_scans = estimate_scans;
            diag.search_overshoot = overshoot;
        }
        if overshoot {
            let count = if job.kind == JobKind::Verification {
                self.verification_overshoots += 1;
                self.verification_overshoots
            } else {
                self.closure_search_overshoots += 1;
                self.closure_search_overshoots
            };
            eprintln!(
                "gsc_pgo: CLOSURE {} OVERSHOOT #{count}: keyframe {} onto {} took {seconds:.3}s, past the {:.3}s of data its estimate of {estimate_s:.3}s ({estimate_scans} scans) spanned",
                match job.kind {
                    JobKind::Verification => "VERIFICATION",
                    JobKind::FastMatch => "FAST MATCH",
                    JobKind::Search => "SEARCH",
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
    fn closure_search_estimate_s(&self, search: &ClosureSearch) -> (f64, usize, usize) {
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
        let lookup = if search.diag.cost_lookup_s > 0.0 {
            config.closure_search_estimate_lookup_s
                + config.closure_search_estimate_lookup_per_keyframe_s * search.diag.map_keyframes as f64
        } else {
            0.0
        };
        let seconds = lookup
            + config.closure_search_estimate_build_per_point_s * (source + target)
            + config.closure_search_estimate_degeneracy_per_point_s * source
            + config.closure_search_estimate_icp_s
                * source
                * target.log2()
                * config.closure_icp_max_iterations as f64
            + config.closure_search_estimate_gates_s;
        (seconds, source_points, target_points)
    }
    /// The newest advancing scan's recorded time (`advancing_scans`; -inf before any).
    pub(crate) fn newest_scan_time(&self) -> f64 {
        self.recent_scan_times
            .back()
            .copied()
            .unwrap_or(f64::NEG_INFINITY)
    }
    /// The median of the newest `scan_period_window` scan periods by recorded time (NaN before two scans).
    pub(crate) fn scan_period_s(&self) -> f64 {
        let mut periods: Vec<f64> = self
            .recent_scan_times
            .iter()
            .zip(self.recent_scan_times.iter().skip(1))
            .map(|(a, b)| b - a)
            .filter(|period| *period > 0.0)
            .collect();
        if periods.is_empty() {
            return f64::NAN;
        }
        periods.sort_by(f64::total_cmp);
        periods[periods.len() / 2]
    }
}
