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

//! The deterministic closure-search schedule (`Config::closure_search_hold_enabled`): one search at a time runs on its
//! own thread, and its result lands only once its estimated cost in scans has passed since its dispatch, the estimate
//! of each piece of work fixed before that work starts (the search's at dispatch, each extension round's as it starts),
//! so which searches run and when their closures land does not depend on how fast the machine is.

use crate::gsc_pgo::{ClosureSearch, ClosureSearchResult, GscPgo};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;

/// What a running search has reported: the extension rounds it has started and whether it has finished.
#[derive(Default)]
pub struct SearchProgress {
    state: Mutex<(usize, bool)>,
    changed: Condvar,
}

impl SearchProgress {
    pub(crate) fn round_started(&self) {
        self.state.lock().expect("search progress").0 += 1;
        self.changed.notify_all();
    }
    pub(crate) fn done(&self) {
        self.state.lock().expect("search progress").1 = true;
        self.changed.notify_all();
    }
    fn snapshot(&self) -> (usize, bool) {
        *self.state.lock().expect("search progress")
    }
    /// Block until the progress differs from `seen`.
    fn wait_past(&self, seen: (usize, bool)) {
        let state = self.state.lock().expect("search progress");
        let _unchanged = self
            .changed
            .wait_while(state, |now| *now == seen)
            .expect("search progress");
    }
}

/// A closure search on its thread, with its schedule: the advancing-scan count (`GscPgo::advancing_scans`) and recorded
/// time at its dispatch, its estimate at
/// dispatch (seconds) and the scan period its seconds are counted in.
pub struct RunningSearch {
    handle: JoinHandle<ClosureSearchResult>,
    progress: Arc<SearchProgress>,
    dispatch_scan: usize,
    /// `scans_seen` at dispatch, for the diagnostics (whose scan indices count every scan).
    dispatch_scans_seen: usize,
    dispatch_time: f64,
    estimate_s: f64,
    scan_period_s: f64,
}

impl RunningSearch {
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
    /// One keyframe's closure search under the deterministic schedule: land the running search once its estimate has
    /// passed (waiting for it then when `wait`, as a lockstep replay does; else only once it has finished), start a
    /// rejected search's next alternative first, then propose this keyframe's match and start it when the search thread
    /// is free and `may_begin`. With `closure_search_hold_enabled` off, a search lands as soon as it has finished.
    pub fn step_closure_search(&mut self, wait: bool, may_begin: bool) {
        let landed = self.land_closure_search(wait, false);
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
    /// Land the running search whatever its estimate, waiting for it (the end of the data).
    pub fn land_remaining_closure_search(&mut self) {
        self.land_closure_search(true, true);
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
        self.closure_search.is_some()
    }
    /// Start the waiting proposal's search on its own thread if none is running.
    fn dispatch_closure_search(&mut self) {
        if self.closure_search.is_some() {
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
        let scan_period_s = self.scan_period_s();
        self.search_scan_times.clear();
        let handle = std::thread::Builder::new()
            .name("gsc-closure-search".into())
            .spawn(move || search.run())
            .expect("spawn closure search thread");
        self.closure_search = Some(RunningSearch {
            handle,
            progress,
            dispatch_scan: self.advancing_scans,
            dispatch_scans_seen: self.scans_seen,
            dispatch_time: self.newest_scan_time(),
            estimate_s,
            scan_period_s,
        });
    }
    /// Land the running search if its estimate has passed (or `now`) and it has finished, waiting for it to finish or
    /// report another extension round when `wait`. Returns whether one landed.
    fn land_closure_search(&mut self, wait: bool, now: bool) -> bool {
        let hold = self.config.closure_search_hold_enabled && !now;
        let Some(running) = &self.closure_search else {
            return false;
        };
        loop {
            let seen = running.progress.snapshot();
            let (rounds, done) = seen;
            // held until both its scans and the data time its seconds span have passed (an early scan cannot end it)
            if hold
                && (self.advancing_scans < running.dispatch_scan + running.estimate_scans(rounds, &self.config)
                    || self.newest_scan_time() - running.dispatch_time
                        < running.estimate_s(rounds, &self.config))
            {
                return false;
            }
            if done {
                break;
            }
            if !wait {
                return false;
            }
            // the replay ran ahead of the search: wait for it to finish or start another round (which moves its estimate)
            let started = std::time::Instant::now();
            running.progress.wait_past(seen);
            self.closure_search_wait_s += started.elapsed().as_secs_f64();
        }
        let running = self.closure_search.take().expect("checked above");
        // finished, so its rounds are final
        let (rounds, _) = running.progress.snapshot();
        let estimate_scans = running.estimate_scans(rounds, &self.config);
        let estimate_s = running.estimate_s(rounds, &self.config);
        let (dispatch_scan, dispatch_time) = (running.dispatch_scans_seen, running.dispatch_time);
        let mut result = running.handle.join().expect("closure search thread panicked");
        let diag = &mut result.diag;
        let seconds = if diag.cost_lookup_s.is_finite() {
            diag.cost_lookup_s
        } else {
            0.0
        } + diag.cost_search_s;
        // the data time from dispatch to the first scan its hold could end at, its scans and seconds both passed (none
        // when the data ended first)
        let spanned = self
            .search_scan_times
            .iter()
            .skip(estimate_scans - 1)
            .map(|time| time - dispatch_time)
            .find(|spanned| *spanned >= estimate_s);
        let overshoot = self.config.closure_search_hold_enabled
            && spanned.is_some_and(|spanned| seconds > spanned);
        diag.scan_index_dispatched = dispatch_scan;
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
        self.search_scan_times.clear();
        self.finish_closure_search(result);
        true
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
