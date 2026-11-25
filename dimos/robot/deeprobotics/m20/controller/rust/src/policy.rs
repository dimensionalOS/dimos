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

//! Policy inference off the 1 kHz loop. The loop offers a request; a busy worker refuses it (busy-drop, counted)
//! and the loop offers again next tick. Results are picked up whenever they land; nothing waits on a timer.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::Instant;

use anyhow::Result;

use crate::config::Config;
use crate::runner::{Runner, Targets};
use crate::types::RobotState;

#[derive(Debug, Clone)]
pub struct PolicyRequest {
    pub runner: String,
    /// clear history / last action / phase first (a freshly constructed runner: tests and sims)
    pub reset: bool,
    /// the first step since rl_control's entry: rl_deploy's RLControlState::OnEnter runs OnEnter(false) on the runner
    /// its OnExit left current (the fallback runner) before anything else
    pub entry: bool,
    /// a gait switch to this runner before this step (update_policy_ptr): OnExit of the current runner, OnEnter of this
    pub switch: bool,
    /// the runner just became the active one (a gait switch): restart its start-up gains
    pub activate: bool,
    pub state: RobotState,
    pub axes: [f64; 3],
    pub params: BTreeMap<String, f64>,
    pub seq: u64,
}

#[derive(Debug, Clone)]
pub struct PolicyResult {
    pub runner: String,
    pub seq: u64,
    pub targets: std::result::Result<Targets, String>,
    /// kp / kd multiplier the runner asks for now (start gains)
    pub gain_scale: f64,
    pub inference_s: f64,
}

pub trait PolicyService: Send {
    /// false when busy: the request is dropped
    fn offer(&mut self, request: PolicyRequest) -> bool;
    fn take_result(&mut self) -> Option<PolicyResult>;
    fn dropped(&self) -> u64;
}

/// Every configured runner, loaded once at startup so a gait switch never loads a model on the hot path.
pub struct RunnerSet {
    runners: HashMap<String, Runner>,
    /// rl_deploy's runner globals a gait switch hands over through
    pub handover: crate::runner::Handover,
    /// the runner rl_deploy's policy pointer is on (None before the first entry)
    pub current: Option<String>,
    /// the runner RLControlState::OnExit leaves the pointer on, whose OnEnter the next entry runs
    entry_runner: String,
}

impl RunnerSet {
    pub fn load(cfg: &Config, policy_dir: &Path) -> Result<RunnerSet> {
        let dt = cfg.timing.policy_decimation as f64 / cfg.timing.control_hz;
        let ort = crate::runner::onnxruntime_library(&cfg.inference.onnxruntime);
        let which = match &ort {
            Some(lib) => format!("policy inference: ONNX Runtime ({lib})"),
            None => "policy inference: tract (no ONNX Runtime library)".to_string(),
        };
        #[cfg(feature = "module")]
        tracing::info!("{which}");
        #[cfg(not(feature = "module"))]
        eprintln!("{which}");
        let mut runners = HashMap::new();
        for (name, spec) in &cfg.runners {
            runners.insert(
                name.clone(),
                Runner::with_inference(name, spec, &cfg.robot, policy_dir, dt, ort.as_deref())?,
            );
        }
        Ok(RunnerSet {
            runners,
            handover: Default::default(),
            current: None,
            entry_runner: cfg.states.rl_control.fallback_runner.clone(),
        })
    }

    /// Keep every runner's last network inputs (offline comparisons).
    pub fn keep_inputs(&mut self) {
        for r in self.runners.values_mut() {
            r.keep_inputs = true;
        }
    }

    /// One runner by name (replays set its state).
    pub fn runner_mut(&mut self, name: &str) -> Option<&mut Runner> {
        self.runners.get_mut(name)
    }

    /// No runners (state-only use of the controller).
    pub fn empty() -> RunnerSet {
        RunnerSet {
            runners: HashMap::new(),
            handover: Default::default(),
            current: None,
            entry_runner: String::new(),
        }
    }

    /// One request; a panic inside inference (tract) comes back as an error result, a policy fault.
    pub fn serve(&mut self, req: &PolicyRequest) -> PolicyResult {
        let started = Instant::now();
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| self.serve_inner(req))) {
            Ok(result) => result,
            Err(_) => PolicyResult {
                runner: req.runner.clone(),
                seq: req.seq,
                targets: Err("inference panicked".into()),
                gain_scale: 1.0,
                inference_s: started.elapsed().as_secs_f64(),
            },
        }
    }

    fn serve_inner(&mut self, req: &PolicyRequest) -> PolicyResult {
        let started = Instant::now();
        let mut gain_scale = 1.0;
        if req.entry {
            if let Some(r) = self.runners.get_mut(&self.entry_runner) {
                r.pipeline.on_enter(&mut self.handover, false);
                self.current = Some(self.entry_runner.clone());
            }
        }
        if req.switch && self.runners.contains_key(&req.runner) {
            if let Some(r) = self.current.as_ref().and_then(|c| self.runners.get(c)) {
                r.pipeline.on_exit(&mut self.handover);
            }
            if let Some(r) = self.runners.get_mut(&req.runner) {
                let per_leg = r.pipeline.spec.action.layout == crate::config::ActionLayout::PerLeg;
                r.pipeline.on_enter(&mut self.handover, per_leg);
                self.current = Some(req.runner.clone());
            }
        }
        let targets = match self.runners.get_mut(&req.runner) {
            None => Err(format!("no runner {}", req.runner)),
            Some(r) => {
                if req.reset {
                    r.reset();
                }
                if req.activate {
                    r.pipeline.activate();
                }
                gain_scale = r.pipeline.gain_scale();
                r.step(&req.state, &req.axes, &req.params)
                    .map_err(|e| format!("{e:#}"))
            }
        };
        let inference_s = started.elapsed().as_secs_f64();
        PolicyResult {
            runner: req.runner.clone(),
            seq: req.seq,
            targets,
            gain_scale,
            inference_s,
        }
    }
}

/// Runs inference in the caller's thread (deterministic; for tests and offline replays).
pub struct InlinePolicy {
    runners: RunnerSet,
    result: Option<PolicyResult>,
}

impl InlinePolicy {
    pub fn new(runners: RunnerSet) -> InlinePolicy {
        InlinePolicy {
            runners,
            result: None,
        }
    }
}

impl PolicyService for InlinePolicy {
    fn offer(&mut self, request: PolicyRequest) -> bool {
        self.result = Some(self.runners.serve(&request));
        true
    }
    fn take_result(&mut self) -> Option<PolicyResult> {
        self.result.take()
    }
    fn dropped(&self) -> u64 {
        0
    }
}

struct Slot {
    request: Option<PolicyRequest>,
    busy: bool,
    result: Option<PolicyResult>,
    stop: bool,
}

/// The slot even if a thread panicked holding it (the slot's fields stay consistent: each write is one assignment).
fn lock_slot(lock: &Mutex<Slot>) -> MutexGuard<'_, Slot> {
    lock.lock().unwrap_or_else(PoisonError::into_inner)
}

/// A worker thread with a one-request mailbox.
pub struct ThreadedPolicy {
    shared: Arc<(Mutex<Slot>, Condvar)>,
    dropped: Arc<AtomicU64>,
    worker: Option<JoinHandle<()>>,
}

impl ThreadedPolicy {
    pub fn spawn(
        mut runners: RunnerSet,
        setup: impl FnOnce() + Send + 'static,
    ) -> Result<ThreadedPolicy> {
        let shared = Arc::new((
            Mutex::new(Slot {
                request: None,
                busy: false,
                result: None,
                stop: false,
            }),
            Condvar::new(),
        ));
        let worker_shared = shared.clone();
        let worker = std::thread::Builder::new()
            .name("m20-policy".into())
            .spawn(move || {
                setup();
                let (lock, cv) = &*worker_shared;
                loop {
                    let req = {
                        let mut slot = lock_slot(lock);
                        let req = loop {
                            if slot.stop {
                                return;
                            }
                            if let Some(req) = slot.request.take() {
                                break req;
                            }
                            slot = cv.wait(slot).unwrap_or_else(PoisonError::into_inner);
                        };
                        slot.busy = true;
                        req
                    };
                    let result = runners.serve(&req);
                    let mut slot = lock_slot(lock);
                    slot.result = Some(result);
                    slot.busy = false;
                }
            })?;
        Ok(ThreadedPolicy {
            shared,
            dropped: Arc::new(AtomicU64::new(0)),
            worker: Some(worker),
        })
    }
}

impl PolicyService for ThreadedPolicy {
    fn offer(&mut self, request: PolicyRequest) -> bool {
        let (lock, cv) = &*self.shared;
        // never block the 1 kHz loop on the worker's lock (a poisoned lock still holds a consistent slot)
        let Some(mut slot) = try_lock_slot(lock) else {
            self.dropped.fetch_add(1, Ordering::Relaxed);
            return false;
        };
        if slot.busy || slot.request.is_some() {
            self.dropped.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        slot.request = Some(request);
        cv.notify_one();
        true
    }

    fn take_result(&mut self) -> Option<PolicyResult> {
        try_lock_slot(&self.shared.0).and_then(|mut s| s.result.take())
    }

    fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

fn try_lock_slot(lock: &Mutex<Slot>) -> Option<MutexGuard<'_, Slot>> {
    match lock.try_lock() {
        Ok(slot) => Some(slot),
        Err(std::sync::TryLockError::Poisoned(p)) => Some(p.into_inner()),
        Err(std::sync::TryLockError::WouldBlock) => None,
    }
}

impl Drop for ThreadedPolicy {
    fn drop(&mut self) {
        lock_slot(&self.shared.0).stop = true;
        self.shared.1.notify_one();
        if let Some(w) = self.worker.take() {
            let _ = w.join();
        }
    }
}
