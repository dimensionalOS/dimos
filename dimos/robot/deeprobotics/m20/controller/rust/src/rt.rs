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

//! The 1 kHz loop: command in, state machine tick, one hardware exchange, pace to the next millisecond.
//! Sleeps are absolute-deadline clock_nanosleep retried on EINTR (a signal never ends the loop or the process);
//! an overrun skips ahead instead of bursting to catch up. Policy inference lives on its own thread
//! (crate::policy) and is never waited on.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::Result;

use crate::commands::CommandSource;
use crate::config::{Config, Realtime};
use crate::hardware::Hardware;
use crate::policy::PolicyService;
use crate::state_machine::Controller;
use crate::types::{JointCommand, RobotState, UserCommand};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pacing {
    /// one tick per control period of wall time (the robot, and real-time sim tests)
    Realtime,
    /// as fast as the hardware answers (offline sim replays)
    Free,
}

#[derive(Debug, Default, Clone)]
pub struct LoopStats {
    pub ticks: u64,
    pub overruns: u64,
    pub hardware_errors: u64,
    pub max_tick_s: f64,
}

/// What each tick saw and sent, for observers (tests, logging off the hot path).
pub struct Tick<'a> {
    pub state: &'a RobotState,
    pub command: &'a JointCommand,
    pub user: &'a UserCommand,
    pub controller: &'a Controller,
    pub stats: &'a LoopStats,
}

/// Apply SCHED_FIFO, CPU pinning and mlockall where configured (Linux; a no-op elsewhere).
pub fn apply_realtime(
    cpu: Option<usize>,
    fifo_priority: Option<i32>,
    lock_memory: bool,
) -> Result<()> {
    #[cfg(target_os = "linux")]
    unsafe {
        if lock_memory && libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) != 0 {
            anyhow::bail!("mlockall: {}", std::io::Error::last_os_error());
        }
        if let Some(cpu) = cpu {
            let mut set: libc::cpu_set_t = std::mem::zeroed();
            libc::CPU_SET(cpu, &mut set);
            if libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set) != 0 {
                anyhow::bail!(
                    "sched_setaffinity {cpu}: {}",
                    std::io::Error::last_os_error()
                );
            }
        }
        if let Some(prio) = fifo_priority {
            let param = libc::sched_param {
                sched_priority: prio,
            };
            if libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) != 0 {
                anyhow::bail!("SCHED_FIFO {prio}: {}", std::io::Error::last_os_error());
            }
        }
        confirm_realtime(cpu, fifo_priority)?;
    }
    #[cfg(not(target_os = "linux"))]
    let _ = (cpu, fifo_priority, lock_memory);
    Ok(())
}

/// Read back this thread's placement: exactly `cpu` and SCHED_FIFO at `fifo_priority`, or an error saying what it is.
#[cfg(target_os = "linux")]
unsafe fn confirm_realtime(cpu: Option<usize>, fifo_priority: Option<i32>) -> Result<()> {
    let (Some(cpu), Some(prio)) = (cpu, fifo_priority) else {
        anyhow::bail!("not configured real-time: realtime cpu and fifo priority must both be set");
    };
    let mut set: libc::cpu_set_t = std::mem::zeroed();
    if libc::sched_getaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &mut set) != 0 {
        anyhow::bail!("sched_getaffinity: {}", std::io::Error::last_os_error());
    }
    let cpus: Vec<usize> = (0..libc::CPU_SETSIZE as usize)
        .filter(|&c| libc::CPU_ISSET(c, &set))
        .collect();
    if cpus != [cpu] {
        anyhow::bail!("pinned to cpus {cpus:?}, not only {cpu}");
    }
    let policy = libc::sched_getscheduler(0);
    let mut param = libc::sched_param { sched_priority: 0 };
    if policy != libc::SCHED_FIFO
        || libc::sched_getparam(0, &mut param) != 0
        || param.sched_priority != prio
    {
        anyhow::bail!(
            "scheduler {policy} priority {}, not SCHED_FIFO {prio}",
            param.sched_priority
        );
    }
    Ok(())
}

pub fn control_realtime(rt: &Realtime) -> Result<()> {
    apply_realtime(rt.control_cpu, rt.control_fifo_priority, rt.lock_memory)
}

/// Sleep until `deadline` (on the Instant clock); EINTR resumes the sleep. `slack` is used off Linux only.
fn sleep_until(deadline: Instant, slack: Duration) {
    #[cfg(target_os = "linux")]
    {
        let now = Instant::now();
        if deadline <= now {
            return;
        }
        let mut ts = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        let _ = slack;
        unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
        // the wait is under a period, so these conversions cannot truncate
        let wait = deadline - now;
        let total_ns = ts.tv_nsec as u128 + wait.as_nanos();
        let target = libc::timespec {
            tv_sec: ts.tv_sec + (total_ns / 1_000_000_000) as libc::time_t,
            tv_nsec: (total_ns % 1_000_000_000) as libc::c_long,
        };
        loop {
            let r = unsafe {
                libc::clock_nanosleep(
                    libc::CLOCK_MONOTONIC,
                    libc::TIMER_ABSTIME,
                    &target,
                    std::ptr::null_mut(),
                )
            };
            if r != libc::EINTR {
                return;
            }
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        // macOS oversleeps sub-ms requests by up to ~1 ms: sleep short of the deadline, then yield until it
        let now = Instant::now();
        if deadline > now + slack {
            std::thread::sleep(deadline - now - slack);
        }
        while Instant::now() < deadline {
            std::thread::yield_now();
        }
    }
}

/// Run until `stop` is set. On stop the loop sends `shutdown_ticks` joint-damping commands, so the motors are never
/// left holding the last position target.
#[allow(clippy::too_many_arguments)]
pub fn run<H: Hardware + ?Sized>(
    cfg: &Config,
    hardware: &mut H,
    controller: &mut Controller,
    policy: &mut dyn PolicyService,
    commands: &mut dyn CommandSource,
    pacing: Pacing,
    stop: &AtomicBool,
    mut observe: impl FnMut(&mut H, &Tick),
) -> Result<LoopStats> {
    // validated finite and positive at load; try_ keeps a hand-built config from panicking
    let period = Duration::try_from_secs_f64(cfg.control_dt())?;
    let slack = Duration::try_from_secs_f64(cfg.realtime.sleep_slack_s)?;
    let mut stats = LoopStats::default();
    let mut retry_at = Instant::now();
    let mut state = loop {
        match hardware.read_state() {
            Ok(s) => break s,
            Err(_) if !stop.load(Ordering::Relaxed) => {
                stats.hardware_errors += 1;
                retry_at = (retry_at + period).max(Instant::now());
                sleep_until(retry_at, slack);
            }
            Err(e) => return Err(e),
        }
    };
    let mut user = UserCommand::default();
    let mut last_command_at = 0.0f64;
    let mut deadline = Instant::now() + period;
    while !stop.load(Ordering::Relaxed) {
        let tick_start = Instant::now();
        // command time is controller time (ticks), which is wall time whenever the loop keeps up
        let now = controller.time();
        if let Some(c) = commands.poll(now) {
            user = c;
            last_command_at = now;
        }
        let mut effective = user.clone();
        if now - last_command_at > cfg.safety.command_timeout_s && commands.expires() {
            effective.axes = [0.0; 3];
            // a metric velocity (cmd_vel) expires the same way
            effective.velocity = effective.velocity.map(|_| [0.0; 3]);
        }
        let cmd = controller.tick(&state, &effective, policy);
        match hardware.exchange(&cmd) {
            Ok(s) => {
                state = s;
                controller.note_fresh();
            }
            Err(_) => {
                stats.hardware_errors += 1;
                controller.note_stale();
            }
        }
        stats.ticks += 1;
        stats.max_tick_s = stats.max_tick_s.max(tick_start.elapsed().as_secs_f64());
        observe(
            hardware,
            &Tick {
                state: &state,
                command: &cmd,
                user: &effective,
                controller,
                stats: &stats,
            },
        );
        if pacing == Pacing::Realtime {
            let now = Instant::now();
            if now > deadline + period {
                stats.overruns += 1;
                deadline = now + period;
            } else {
                sleep_until(deadline, slack);
                deadline += period;
            }
        }
    }
    let damping = UserCommand {
        target: crate::types::Target::JointDamping,
        ..Default::default()
    };
    // the shutdown ticks keep the loop's absolute deadlines (no drift)
    deadline = deadline.max(Instant::now());
    for _ in 0..cfg.safety.shutdown_damping_ticks {
        let cmd = controller.tick(&state, &damping, policy);
        if let Ok(s) = hardware.exchange(&cmd) {
            state = s;
        }
        if pacing == Pacing::Realtime {
            sleep_until(deadline, slack);
            deadline += period;
        }
    }
    Ok(stats)
}
