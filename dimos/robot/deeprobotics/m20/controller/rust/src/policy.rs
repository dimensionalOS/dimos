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
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::Instant;

use anyhow::Result;

use crate::config::Config;
use crate::runner::{Runner, Targets};
use crate::types::RobotState;

#[derive(Debug, Clone)]
pub struct PolicyRequest {
    pub runner: String,
    /// clear history / last action / phase first (fresh runner state)
    pub reset: bool,
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
}

impl RunnerSet {
    pub fn load(cfg: &Config, policy_dir: &Path) -> Result<RunnerSet> {
        let dt = cfg.timing.policy_decimation as f64 / cfg.timing.control_hz;
        let mut runners = HashMap::new();
        for (name, spec) in &cfg.runners {
            runners.insert(
                name.clone(),
                Runner::new(name, spec, &cfg.robot, policy_dir, dt)?,
            );
        }
        Ok(RunnerSet { runners })
    }

    /// One runner by name (replays set its state).
    pub fn runner_mut(&mut self, name: &str) -> Option<&mut Runner> {
        self.runners.get_mut(name)
    }

    /// No runners (state-only use of the controller).
    pub fn empty() -> RunnerSet {
        RunnerSet {
            runners: HashMap::new(),
        }
    }

    pub fn serve(&mut self, req: &PolicyRequest) -> PolicyResult {
        let started = Instant::now();
        let mut gain_scale = 1.0;
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

/// A worker thread with a one-request mailbox.
pub struct ThreadedPolicy {
    shared: Arc<(Mutex<Slot>, Condvar)>,
    dropped: Arc<AtomicU64>,
    worker: Option<JoinHandle<()>>,
}

impl ThreadedPolicy {
    pub fn spawn(mut runners: RunnerSet, setup: impl FnOnce() + Send + 'static) -> ThreadedPolicy {
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
                        let mut slot = lock.lock().unwrap();
                        while slot.request.is_none() && !slot.stop {
                            slot = cv.wait(slot).unwrap();
                        }
                        if slot.stop {
                            return;
                        }
                        slot.busy = true;
                        slot.request.take().unwrap()
                    };
                    let result = runners.serve(&req);
                    let mut slot = lock.lock().unwrap();
                    slot.result = Some(result);
                    slot.busy = false;
                }
            })
            .expect("spawn policy thread");
        ThreadedPolicy {
            shared,
            dropped: Arc::new(AtomicU64::new(0)),
            worker: Some(worker),
        }
    }
}

impl PolicyService for ThreadedPolicy {
    fn offer(&mut self, request: PolicyRequest) -> bool {
        let (lock, cv) = &*self.shared;
        // never block the 1 kHz loop on the worker's lock
        let Ok(mut slot) = lock.try_lock() else {
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
        self.shared
            .0
            .try_lock()
            .ok()
            .and_then(|mut s| s.result.take())
    }

    fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

impl Drop for ThreadedPolicy {
    fn drop(&mut self) {
        if let Ok(mut s) = self.shared.0.lock() {
            s.stop = true;
        }
        self.shared.1.notify_one();
        if let Some(w) = self.worker.take() {
            let _ = w.join();
        }
    }
}
