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
//! work fixed before that work starts (the search's at dispatch, each extension round's as it starts), so which searches
//! run and when their closures land does not depend on how fast the machine is.
//!
//! Logical execution time (`closure_search_let_enabled`, the default): every keyframe's search is its own job, dispatched
//! (its graph snapshot taken) at the keyframe it is proposed, whatever other searches are doing, and run on a pool of
//! `closure_search_workers` threads in dispatch order. Results land in due order (ties by dispatch order); a search reads
//! only its snapshot, so what an in-flight search finds never reaches another until it lands. A rejected search's next
//! alternative is dispatched at the keyframe its result lands at, as its own job. Off: one search at a time on its own
//! thread, the next keyframes' proposals going stale while it is held.

use crate::gsc_pgo::{ClosureSearch, ClosureSearchResult, GscPgo};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;

/// What a search has reported: the extension rounds it has started, and its result once it has finished.
#[derive(Default)]
struct ProgressState {
    rounds: usize,
    result: Option<ClosureSearchResult>,
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
    fn finish(&self, result: ClosureSearchResult) {
        self.state.lock().expect("search progress").result = Some(result);
        self.changed.notify_all();
    }
    /// The rounds started and whether it has finished.
    fn snapshot(&self) -> (usize, bool) {
        let state = self.state.lock().expect("search progress");
        (state.rounds, state.result.is_some())
    }
    fn take_result(&self) -> Option<ClosureSearchResult> {
        self.state.lock().expect("search progress").result.take()
    }
    /// Block until the progress differs from `seen`.
    fn wait_past(&self, seen: (usize, bool)) {
        let state = self.state.lock().expect("search progress");
        let _unchanged = self
            .changed
            .wait_while(state, |now| (now.rounds, now.result.is_some()) == seen)
            .expect("search progress");
    }
}

/// The threads closure searches run on (`closure_search_workers`), taking them in dispatch order.
pub struct SearchPool {
    jobs: Sender<(ClosureSearch, Arc<SearchProgress>)>,
    _workers: Vec<JoinHandle<()>>,
}

impl SearchPool {
    fn new(workers: usize) -> SearchPool {
        let (jobs, queue) = channel::<(ClosureSearch, Arc<SearchProgress>)>();
        let queue: Arc<Mutex<Receiver<_>>> = Arc::new(Mutex::new(queue));
        let workers = (0..workers.max(1))
            .map(|i| {
                let queue = queue.clone();
                std::thread::Builder::new()
                    .name(format!("gsc-closure-search-{i}"))
                    .spawn(move || loop {
                        let next = queue.lock().expect("search queue").recv();
                        let Ok((search, progress)) = next else {
                            return;
                        };
                        progress.finish(search.run());
                    })
                    .expect("spawn closure search worker")
            })
            .collect();
        SearchPool {
            jobs,
            _workers: workers,
        }
    }
}

/// A dispatched closure search and its schedule: its dispatch number, the advancing-scan count (`GscPgo::advancing_scans`)
/// and recorded time at its dispatch, its estimate at dispatch (seconds) and the scan period its seconds are counted in.
pub struct SearchJob {
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
    /// Its hold passed while it was still running, in a run that does not wait (live: it lands late).
    late: std::cell::Cell<bool>,
}

impl SearchJob {
    /// The estimate (seconds) with `rounds` extension rounds started.
    fn estimate_s(&self, rounds: usize, config: &crate::gsc_pgo::Config) -> f64 {
        if rounds == 0 {
            return self.estimate_s;
        }
        self.estimate_s
            + config.closure_search_estimate_extension_overhead_s
            + rounds as f64 * config.closure_search_estimate_extension_cloud_s
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
    /// match and dispatch it. Logical execution time (`closure_search_let_enabled`): every due result lands, in due
    /// order, a rejected one's next alternative dispatched as it lands, and every keyframe's match is dispatched
    /// (`may_begin` is ignored: under a compute shortage searches overshoot rather than go unsearched). Else one search
    /// at a time: a rejected search's next alternative goes before this keyframe's own match, and a match starts only
    /// when no search is held and `may_begin`. With `closure_search_hold_enabled` off, a search lands once it finishes.
    pub fn step_closure_search(&mut self, wait: bool, may_begin: bool) {
        if self.let_searches() {
            self.land_due_closure_searches(wait, false);
            self.propose_closure();
            self.dispatch_closure_search();
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
                self.dispatch_closure_search();
            }
        }
        self.propose_closure();
        if may_begin {
            self.dispatch_closure_search();
        }
    }
    /// Land every search still out whatever its estimate, waiting for each (the end of the data).
    pub fn land_remaining_closure_search(&mut self) {
        while !self.closure_searches.is_empty() {
            self.land_due_closure_searches(true, true);
        }
    }
    /// Searches that ran past the data time their estimate spanned, and the seconds a lockstep replay waited on searches
    /// whose estimate had passed.
    pub fn closure_search_overshoots(&self) -> usize {
        self.closure_search_overshoots
    }
    pub fn closure_search_wait_s(&self) -> f64 {
        self.closure_search_wait_s
    }
    /// Whether a closure search is running or held for its estimate.
    pub fn closure_search_running(&self) -> bool {
        !self.closure_searches.is_empty()
    }
    fn let_searches(&self) -> bool {
        self.config.closure_search_hold_enabled && self.config.closure_search_let_enabled
    }
    /// Dispatch the waiting proposal's search: onto the worker pool (`closure_search_let_enabled`), else on its own
    /// thread if no search is out.
    fn dispatch_closure_search(&mut self) {
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
        let thread = if pooled {
            let workers = self.config.closure_search_workers;
            self.search_pool
                .get_or_insert_with(|| SearchPool::new(workers))
                .jobs
                .send((search, progress.clone()))
                .expect("closure search workers stopped");
            None
        } else {
            let progress = progress.clone();
            Some(
                std::thread::Builder::new()
                    .name("gsc-closure-search".into())
                    .spawn(move || progress.finish(search.run()))
                    .expect("spawn closure search thread"),
            )
        };
        self.search_order += 1;
        self.closure_searches.push_back(SearchJob {
            progress,
            thread,
            order: self.search_order,
            dispatch_scan: self.advancing_scans,
            dispatch_scans_seen: self.scans_seen,
            dispatch_time: self.newest_scan_time(),
            estimate_s,
            scan_period_s: self.scan_period_s(),
            late: std::cell::Cell::new(false),
        });
    }
    /// Whether `job`'s hold has passed with `rounds` rounds started: its scans and the data time its seconds span (an
    /// early scan cannot end it).
    fn hold_passed(&self, job: &SearchJob, rounds: usize) -> bool {
        self.advancing_scans >= job.dispatch_scan + job.estimate_scans(rounds, &self.config)
            && self.newest_scan_time() - job.dispatch_time >= job.estimate_s(rounds, &self.config)
    }
    /// Land the searches whose hold has passed (all, when `now`) and that have finished, waiting for one to finish or
    /// report another extension round when `wait`, in due order (ties by dispatch order). Returns how many landed.
    fn land_due_closure_searches(&mut self, wait: bool, now: bool) -> usize {
        let hold = self.config.closure_search_hold_enabled && !now;
        // each finished search whose hold has passed, by its due scan and dispatch number
        let mut due: Vec<(usize, u64)> = Vec::new();
        let mut waited_s = 0.0;
        for job in &self.closure_searches {
            loop {
                let seen = job.progress.snapshot();
                let (rounds, done) = seen;
                if hold && !self.hold_passed(job, rounds) {
                    break;
                }
                if done {
                    due.push((job.dispatch_scan + job.estimate_scans(rounds, &self.config), job.order));
                    break;
                }
                if !wait {
                    job.late.set(hold);
                    break;
                }
                // the replay ran ahead of the search: wait for it to finish or start another round (which moves its hold)
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
        for &(_, order) in &due {
            let index = self
                .closure_searches
                .iter()
                .position(|job| job.order == order)
                .expect("a due search is out");
            let job = self.closure_searches.remove(index).expect("index found above");
            self.land_closure_search(job);
            // a rejected search's next alternative is its own job, dispatched (its snapshot taken) as its result lands
            if self.let_searches()
                && self
                    .waiting_proposal
                    .as_ref()
                    .is_some_and(|waiting| waiting.tries > 0)
            {
                self.dispatch_closure_search();
            }
        }
        self.trim_search_scan_times();
        due.len()
    }
    /// Apply a finished search's result: its schedule into its diagnostics, an overshoot logged and counted.
    fn land_closure_search(&mut self, job: SearchJob) {
        let (rounds, _) = job.progress.snapshot();
        let estimate_scans = job.estimate_scans(rounds, &self.config);
        let estimate_s = job.estimate_s(rounds, &self.config);
        let mut result = job.progress.take_result().expect("a landing search has finished");
        if let Some(thread) = job.thread {
            thread.join().expect("closure search thread panicked");
        }
        let diag = &mut result.diag;
        let seconds = if diag.cost_lookup_s.is_finite() {
            diag.cost_lookup_s
        } else {
            0.0
        } + diag.cost_search_s;
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
        diag.scan_index_dispatched = job.dispatch_scans_seen;
        diag.estimate_s = estimate_s;
        diag.estimate_scans = estimate_scans;
        diag.search_overshoot = overshoot;
        if overshoot {
            self.closure_search_overshoots += 1;
            eprintln!(
                "gsc_pgo: CLOSURE SEARCH OVERSHOOT #{}: the search of keyframe {} onto {} took {seconds:.3}s, past the {:.3}s of data its estimate of {estimate_s:.3}s ({estimate_scans} scans) spanned",
                self.closure_search_overshoots,
                diag.source_id,
                diag.target_id,
                spanned.unwrap_or(f64::NAN),
            );
        }
        self.finish_closure_search(result);
    }
    /// Drop the scan times older than every out search's dispatch.
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
            search.key_poses[k]
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
    fn newest_scan_time(&self) -> f64 {
        self.recent_scan_times
            .back()
            .copied()
            .unwrap_or(f64::NEG_INFINITY)
    }
    /// The median of the newest 50 scan periods by recorded time (NaN before two scans).
    fn scan_period_s(&self) -> f64 {
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
