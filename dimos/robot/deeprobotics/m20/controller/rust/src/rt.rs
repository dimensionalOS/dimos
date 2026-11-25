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

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Mutex, TryLockError};
use std::time::{Duration, Instant};

use anyhow::Result;

use crate::commands::CommandSource;
use crate::config::{tile, Config, Realtime};
use crate::hardware::Hardware;
use crate::policy::PolicyService;
use crate::state_machine::Controller;
use crate::types::{JointCommand, RobotState, UserCommand};

/// The 1 kHz loop's reads that found a receiver thread holding the lock and went on without it (controller_report).
pub static LOCK_SKIPS: AtomicU64 = AtomicU64::new(0);

/// The 1 kHz loop's read of a lock a receiver thread writes: never waits; None (counted in `skips`) while it is held.
pub fn try_read<T, R>(
    lock: &Mutex<T>,
    skips: &AtomicU64,
    read: impl FnOnce(&mut T) -> R,
) -> Option<R> {
    match lock.try_lock() {
        Ok(mut value) => Some(read(&mut value)),
        Err(TryLockError::Poisoned(p)) => Some(read(&mut p.into_inner())),
        Err(TryLockError::WouldBlock) => {
            skips.fetch_add(1, Ordering::Relaxed);
            None
        }
    }
}

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
    let policy = libc::sched_getscheduler(0);
    let mut param = libc::sched_param { sched_priority: 0 };
    let read = libc::sched_getparam(0, &mut param) == 0;
    check_placement(
        &cpus,
        policy == libc::SCHED_FIFO && read,
        param.sched_priority,
        cpu,
        prio,
    )
}

/// The placement read back is exactly `cpu`, SCHED_FIFO (`fifo`) at `prio`; otherwise an error saying what it is.
pub fn check_placement(
    cpus: &[usize],
    fifo: bool,
    priority: i32,
    cpu: usize,
    prio: i32,
) -> Result<()> {
    if cpus != [cpu] {
        anyhow::bail!("pinned to cpus {cpus:?}, not only {cpu}");
    }
    if !fifo || priority != prio {
        anyhow::bail!("scheduler fifo={fifo} priority {priority}, not SCHED_FIFO {prio}");
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
    let mut panicked = false;
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
        // ours: a panic in the tick (a bug) damps the joints instead of leaving the motors on their last command
        let tick = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            controller.tick(&state, &effective, policy)
        }));
        let Ok(cmd) = tick else {
            panicked = true;
            break;
        };
        commands.observe(controller);
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
    // nothing holds the robot once this process exits (rl_deploy stays up and holds it): lie a standing robot down
    let lie_down = UserCommand {
        target: crate::types::Target::LieDown,
        gait: user.gait,
        ..Default::default()
    };
    if panicked {
        // the controller cannot be trusted to tick again: joint_damping's gains straight to the motors
        let jd = &cfg.states.joint_damping;
        let damp = JointCommand {
            kp: tile(&jd.kp, crate::config::JOINTS)?
                .try_into()
                .unwrap_or([0.0; crate::config::JOINTS]),
            kd: tile(&jd.kd, crate::config::JOINTS)?
                .try_into()
                .unwrap_or([0.0; crate::config::JOINTS]),
            ..Default::default()
        };
        for _ in 0..cfg.safety.shutdown_damping_ticks {
            let _ = hardware.exchange(&damp);
            if pacing == Pacing::Realtime {
                sleep_until(deadline, slack);
                deadline += period;
            }
        }
        anyhow::bail!("the controller panicked; the joints were damped");
    }
    let lie_ticks = (cfg.safety.shutdown_lie_down_max_s / cfg.control_dt()) as u64;
    for _ in 0..lie_ticks {
        if controller.at_rest() {
            break;
        }
        let cmd = controller.tick(&state, &lie_down, policy);
        if let Ok(s) = hardware.exchange(&cmd) {
            state = s;
        }
        if pacing == Pacing::Realtime {
            sleep_until(deadline, slack);
            deadline += period;
        }
    }
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

#[cfg(test)]
mod try_read_tests {
    use super::*;
    use std::sync::Arc;

    #[test]
    fn a_held_lock_is_skipped_at_once_and_counted() {
        let (lock, skips) = (Arc::new(Mutex::new(7)), AtomicU64::new(0));
        assert_eq!(try_read(&lock, &skips, |v| *v), Some(7));
        let (held, release) = (std::sync::Barrier::new(2), std::sync::Barrier::new(2));
        std::thread::scope(|s| {
            s.spawn(|| {
                let _guard = lock.lock().unwrap();
                held.wait();
                release.wait();
            });
            held.wait();
            // the receiver holds it: no value (WouldBlock, never a wait), one skip
            assert_eq!(try_read(&lock, &skips, |v| *v), None);
            assert_eq!(skips.load(Ordering::Relaxed), 1);
            release.wait();
        });
        assert_eq!(try_read(&lock, &skips, |v| *v), Some(7));
        assert_eq!(skips.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn a_poisoned_lock_is_still_read() {
        let (lock, skips) = (Arc::new(Mutex::new(3)), AtomicU64::new(0));
        let poisoner = lock.clone();
        let _ = std::thread::spawn(move || {
            let _guard = poisoner.lock().unwrap();
            panic!("a receiver panics holding the lock");
        })
        .join();
        assert!(lock.is_poisoned());
        assert_eq!(try_read(&lock, &skips, |v| *v), Some(3));
        assert_eq!(skips.load(Ordering::Relaxed), 0);
    }
}

#[cfg(test)]
mod placement_tests {
    use super::check_placement;

    #[test]
    fn only_the_configured_core_at_the_configured_fifo_priority_passes() {
        assert!(check_placement(&[4], true, 80, 4, 80).is_ok());
        assert!(check_placement(&[4, 5], true, 80, 4, 80).is_err());
        assert!(check_placement(&[0, 1, 2, 3], true, 80, 4, 80).is_err());
        assert!(check_placement(&[4], false, 0, 4, 80).is_err());
        assert!(check_placement(&[4], true, 70, 4, 80).is_err());
    }
}

#[cfg(test)]
mod shutdown_tests {
    use super::*;
    use crate::commands::CommandSource;
    use crate::policy::{PolicyRequest, PolicyResult};
    use crate::state_machine::State;
    use crate::types::{JointCommand, RobotState, Target};

    /// Joints that follow the command exactly (and a moving stamp and torque, so no frozen-motor fault).
    struct Follow(RobotState);

    impl Hardware for Follow {
        fn read_state(&mut self) -> Result<RobotState> {
            Ok(self.0.clone())
        }
        fn exchange(&mut self, cmd: &JointCommand) -> Result<RobotState> {
            self.0.stamp += 0.001;
            self.0.tau = [self.0.stamp; crate::config::JOINTS];
            if cmd.kp.iter().any(|k| *k > 0.0) {
                self.0.q = cmd.q;
            }
            Ok(self.0.clone())
        }
    }

    /// Holds the pose it is given for every runner.
    struct Hold(Option<PolicyRequest>);

    impl crate::policy::PolicyService for Hold {
        fn offer(&mut self, request: PolicyRequest) -> bool {
            self.0 = Some(request);
            true
        }
        fn take_result(&mut self) -> Option<PolicyResult> {
            self.0.take().map(|r| PolicyResult {
                runner: r.runner,
                seq: r.seq,
                targets: Ok(crate::runner::Targets {
                    q: r.state.q,
                    wheel_velocity: [0.0; 4],
                    action: [0.0; crate::config::JOINTS],
                    velocity_estimate: None,
                    command_zero: true,
                }),
                gain_scale: 1.0,
                inference_s: 0.0,
            })
        }
        fn dropped(&self) -> u64 {
            0
        }
    }

    /// Stands (rl_deploy's stand, then RL control as basic_server sends them) for `stand_s`, then asks the loop to stop.
    struct StandThenStop<'a> {
        sent: u64,
        stand_s: f64,
        stop: &'a AtomicBool,
    }

    impl CommandSource for StandThenStop<'_> {
        fn poll(&mut self, now: f64) -> Option<UserCommand> {
            if now > self.stand_s {
                self.stop.store(true, Ordering::Relaxed);
            }
            // the stand now, RL control right after (the stand-up holds it until both phases are over)
            let target = match self.sent {
                0 => Target::Stand,
                1 => Target::RlControl,
                _ => return None,
            };
            self.sent += 1;
            Some(UserCommand {
                target,
                gait: 0x1001,
                motion_seq: self.sent,
                ..Default::default()
            })
        }
    }

    fn shutdown_after(stand_s: f64) -> (State, f64) {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        let cfg = Config::load(&path).unwrap();
        let mut s = RobotState {
            quat: [1.0, 0.0, 0.0, 0.0],
            ..Default::default()
        };
        s.q = crate::state_machine::leg_pose_for_height(&cfg.robot, 0.03, 0.0);
        let mut hw = Follow(s);
        let mut c = Controller::new(cfg.clone());
        c.set_time(10.0);
        let stop = AtomicBool::new(false);
        let mut commands = StandThenStop {
            sent: 0,
            stand_s: 10.0 + stand_s,
            stop: &stop,
        };
        let mut policy = Hold(None);
        let mut lay = false;
        let _ = run(
            &cfg,
            &mut hw,
            &mut c,
            &mut policy,
            &mut commands,
            Pacing::Free,
            &stop,
            |_, _| {},
        );
        // the transitions after the stop request: a lie-down, then the damping
        for (_, _, to) in &c.transitions {
            lay |= *to == State::LieDown;
        }
        assert!(lay, "a standing robot lay down: {:?}", c.transitions);
        let height = crate::dock::obstacle::body_height(&cfg.robot, &hw.0.q, 0.0);
        (c.state, height)
    }

    #[test]
    fn a_stop_while_walking_lies_the_robot_down_before_the_damping() {
        let (state, height) = shutdown_after(12.0);
        assert_eq!(state, State::JointDamping);
        // the lie-down pose (IK 0.03, the knee clamped) is 0.086 hip-to-axle; standing is 0.38
        assert!(height < 0.1, "lying: hip-to-axle {height}");
    }

    #[test]
    fn a_stop_during_the_stand_up_and_stand_in_waits_them_out_then_lies_down() {
        for stand_s in [0.5, 4.0] {
            let (state, height) = shutdown_after(stand_s);
            assert_eq!(state, State::JointDamping, "stop after {stand_s} s");
            assert!(height < 0.1, "stop after {stand_s} s: hip-to-axle {height}");
        }
    }

    /// Follow with replies dropped where `drop(tick)` says (the exchange errs; the motors still get the command).
    struct Dropping {
        inner: Follow,
        tick: u64,
        drop: fn(u64) -> bool,
    }

    impl Hardware for Dropping {
        fn read_state(&mut self) -> Result<RobotState> {
            self.inner.read_state()
        }
        fn exchange(&mut self, cmd: &JointCommand) -> Result<RobotState> {
            self.tick += 1;
            let s = self.inner.exchange(cmd)?;
            if (self.drop)(self.tick) {
                anyhow::bail!("no reply")
            }
            Ok(s)
        }
    }

    /// Stands into rl_control under the drop pattern for 14 s; the transitions and the faults seen.
    fn stand_with_drops(drop: fn(u64) -> bool) -> (Vec<State>, Vec<String>) {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        let cfg = Config::load(&path).unwrap();
        let mut s = RobotState {
            quat: [1.0, 0.0, 0.0, 0.0],
            ..Default::default()
        };
        s.q = crate::state_machine::leg_pose_for_height(&cfg.robot, 0.03, 0.0);
        let mut hw = Dropping {
            inner: Follow(s),
            tick: 0,
            drop,
        };
        let mut c = Controller::new(cfg.clone());
        c.set_time(10.0);
        let stop = AtomicBool::new(false);
        let never = AtomicBool::new(false);
        let mut commands = StandThenStop {
            sent: 0,
            stand_s: f64::INFINITY,
            stop: &never,
        };
        let mut faults = Vec::new();
        let _ = run(
            &cfg,
            &mut hw,
            &mut c,
            &mut Hold(None),
            &mut commands,
            Pacing::Free,
            &stop,
            |_, tick| {
                if let Some(f) = &tick.controller.fault {
                    if faults.last() != Some(f) {
                        faults.push(f.clone());
                    }
                }
                if tick.controller.time() > 24.0 {
                    stop.store(true, Ordering::Relaxed);
                }
            },
        );
        (c.transitions.iter().map(|t| t.2).collect(), faults)
    }

    /// safety.missed_replies: scattered misses pass, a run of them lies the robot down.
    #[test]
    fn a_run_of_missed_replies_faults_and_lies_down() {
        let cfg = Config::load(
            &std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml"),
        )
        .unwrap();
        let n = cfg.safety.missed_replies.expect("missed_replies on");
        // every other reply missing for the whole run: never n in a row
        let (states, faults) = stand_with_drops(|t| t % 2 == 0);
        assert!(faults.is_empty(), "{faults:?}");
        assert_eq!(states[..2], [State::StandUp, State::RlControl]);
        // a 60 ms gap 10 s in, in rl_control
        let (states, faults) = stand_with_drops(|t| (10_000..10_060).contains(&t));
        assert!(
            faults.iter().any(|f| f.contains("replies missed")),
            "{faults:?} (limit {n})"
        );
        assert!(states.contains(&State::LieDown), "{states:?}");
    }

    /// Follow, keeping every command sent.
    struct Recording {
        inner: Follow,
        sent: Vec<JointCommand>,
    }

    impl Hardware for Recording {
        fn read_state(&mut self) -> Result<RobotState> {
            self.inner.read_state()
        }
        fn exchange(&mut self, cmd: &JointCommand) -> Result<RobotState> {
            self.sent.push(*cmd);
            self.inner.exchange(cmd)
        }
    }

    /// Hold, but its `panic_at`th answer panics (a bug in the tick).
    struct PanicAt(Hold, u64, u64);

    impl crate::policy::PolicyService for PanicAt {
        fn offer(&mut self, request: PolicyRequest) -> bool {
            self.0.offer(request)
        }
        fn take_result(&mut self) -> Option<PolicyResult> {
            let r = self.0.take_result();
            if r.is_some() {
                self.1 += 1;
                assert!(self.1 < self.2, "injected panic");
            }
            r
        }
        fn dropped(&self) -> u64 {
            0
        }
    }

    /// A panic in the tick: the loop ends, the joints get joint_damping's gains to the end, run reports it.
    #[test]
    fn a_panic_in_the_tick_damps_the_joints() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        let cfg = Config::load(&path).unwrap();
        let mut s = RobotState {
            quat: [1.0, 0.0, 0.0, 0.0],
            ..Default::default()
        };
        s.q = crate::state_machine::leg_pose_for_height(&cfg.robot, 0.03, 0.0);
        let mut hw = Recording {
            inner: Follow(s),
            sent: vec![],
        };
        let mut c = Controller::new(cfg.clone());
        c.set_time(10.0);
        let stop = AtomicBool::new(false);
        let never = AtomicBool::new(false);
        let mut commands = StandThenStop {
            sent: 0,
            stand_s: f64::INFINITY,
            stop: &never,
        };
        let mut policy = PanicAt(Hold(None), 0, 100);
        let result = run(
            &cfg,
            &mut hw,
            &mut c,
            &mut policy,
            &mut commands,
            Pacing::Free,
            &stop,
            |_, _| {},
        );
        assert!(result.is_err());
        assert_eq!(c.state, State::RlControl, "it panicked standing");
        let tail = &hw.sent[hw.sent.len() - cfg.safety.shutdown_damping_ticks as usize..];
        let kd = tile(&cfg.states.joint_damping.kd, crate::config::JOINTS).unwrap();
        assert!(tail
            .iter()
            .all(|c| c.kp == [0.0; crate::config::JOINTS] && c.kd.to_vec() == kd));
    }

    /// A start with the legs up (a restart while the motors hold a stand): the first state is a lie-down.
    #[test]
    fn a_start_standing_lies_down_first() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        for (cfg, lies) in [
            (Config::load(&path).unwrap(), true),
            (Config::load(&path).unwrap().rl_deploy(), false),
        ] {
            let mut s = RobotState {
                quat: [1.0, 0.0, 0.0, 0.0],
                ..Default::default()
            };
            s.q = crate::state_machine::leg_pose_for_height(
                &cfg.robot,
                cfg.states.stand_up.stand_height,
                0.0,
            );
            let mut c = Controller::new(cfg.clone());
            let mut policy = Hold(None);
            for _ in 0..10 {
                s.stamp += 0.001;
                s.tau = [s.stamp; crate::config::JOINTS];
                c.tick(&s, &UserCommand::default(), &mut policy);
            }
            assert_eq!(c.state == State::LieDown, lies);
        }
    }
}
