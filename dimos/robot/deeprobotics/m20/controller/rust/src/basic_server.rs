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

//! basic_server's VelControl (AOS basic_server, md5 538e34f2) between the operator and the controller: usage mode, its gating of the velocity, the stand sequence and the gait refusals.

use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex};

use tracing::{info, warn};

use crate::commands::CommandSource;
use crate::dock::docking::{DockingCommands, Phase, Status as DockStatus};
use crate::state_machine::{Controller, State};
use crate::types::{Target, UserCommand};

/// The app's usage modes (dimos basic_server.py's 1 is navigation); the sensors each switches are not ours to switch.
pub const MODE_REGULAR: i32 = 0;
pub const MODE_NAVIGATION: i32 = 1;
pub const MODE_ASSIST: i32 = 2;
pub const GAIT_BASIC: u32 = 0x1001;

/// The gait codes SwitchGait (0x261640) sends; it logs and drops any other.
pub fn known_gait(gait: u32) -> bool {
    matches!(
        gait,
        0x1001..=0x1005 | 0x2001..=0x2004 | 0x3001..=0x3005 | 0x5001..=0x5003 | 0xf001..=0xf003
    )
}

/// The Extre gaits (0x2001-0x2004: Move, Jump, Bipedal, Inv_Bipedal).
pub fn extre_gait(gait: u32) -> bool {
    (0x2001..=0x2004).contains(&gait)
}

/// charge_manager's /CHARGE_STATUS state (0 idle, 1 going to the pile, 2 charging, 3 leaving it), as basic_server reads it.
pub trait ChargerState {
    fn charger_state(&self) -> i32;
}

/// charge_manager's state for our docking's status: 5 (on the dock, no charge) is basic_server.py's CHARGE_ON_DOCK.
pub fn charger_state_of(status: &DockStatus) -> i32 {
    match status.phase {
        Phase::Charging if status.charging => 2,
        Phase::Charging => 5,
        _ if status.active && status.undocking => 3,
        // charge_manager's error state: an undock (command 0) leaves it, where idle ignores one
        Phase::Idle if status.error_code.is_some() => 4,
        Phase::Idle => 0,
        _ if status.active => 1,
        _ => 0,
    }
}

impl<C: CommandSource> ChargerState for DockingCommands<C> {
    fn charger_state(&self) -> i32 {
        charger_state_of(self.docking.status())
    }
}

/// Where the module's handlers put a usage-mode request (CmdHandle::cmdHandle_ControlUsage) and read the mode.
#[derive(Default)]
pub struct UsageMode {
    pub request: Mutex<Option<i32>>,
    pub current: AtomicI32,
    /// CmdHandle::cmdHandle_AligningBodyPosture: get up after a fall
    pub align: std::sync::atomic::AtomicBool,
}

/// aligningBodyPosture (0x262550): RL control on 0xf003 from Idle until up ([basic_server.align]), else soft e-stop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Align {
    pub next_check: f64,
    pub checks_left: u32,
}

pub struct BasicServer<C> {
    inner: C,
    usage: Arc<UsageMode>,
    mode: i32,
    charger: i32,
    /// the operator's (or docking's) latest command
    last: UserCommand,
    /// SendGait from basic_server itself (a stand, a usage mode change), until the operator's gait changes
    gait_override: Option<(u32, (u32, u64))>,
    /// SetMotionState(1): waiting for the stand, until this controller time
    stand_until: Option<f64>,
    /// a change to send out on the next poll
    dirty: bool,
    /// a fall recovery under way
    align: Option<Align>,
    /// basic_server's MatchAuthorizationCode (its authority_code.toml matched)
    pub extre_authorized: bool,
    /// MotionInfo's height from the joints (base_link above the wheels' contact)
    cfg: crate::config::BasicServerConfig,
    real_steer: crate::config::RealSteer,
}

impl<C: CommandSource + ChargerState> BasicServer<C> {
    pub fn new(
        inner: C,
        usage: Arc<UsageMode>,
        cfg: crate::config::BasicServerConfig,
        real_steer: crate::config::RealSteer,
    ) -> Self {
        BasicServer {
            inner,
            usage,
            cfg,
            real_steer,
            mode: MODE_REGULAR,
            charger: 0,
            last: UserCommand::default(),
            gait_override: None,
            stand_until: None,
            dirty: false,
            align: None,
            extre_authorized: false,
        }
    }

    pub fn inner(&self) -> &C {
        &self.inner
    }

    pub fn mode(&self) -> i32 {
        self.mode
    }

    fn switch_gait(&mut self, gait: u32) {
        self.gait_override = Some((gait, (self.last.gait, self.last.gait_seq)));
        self.dirty = true;
    }

    /// setControlUsageMode (0x261ba8): on a change only; modes 0 and 2 also SwitchGait(0x1001).
    fn set_mode(&mut self, mode: i32) {
        if mode == self.mode {
            return;
        }
        info!(from = self.mode, to = mode, "usage mode");
        if matches!(mode, MODE_REGULAR | MODE_ASSIST) {
            self.switch_gait(GAIT_BASIC);
        }
        self.mode = mode;
        self.usage.current.store(mode, Ordering::Relaxed);
        self.dirty = true;
    }

    fn out(&self) -> UserCommand {
        let mut c = self.last.clone();
        if let Some((gait, _)) = self.gait_override {
            c.gait = gait;
        }
        // NavCmd / RealSteer and charge_manager's steer reach rl_deploy only in modes 1 and 2
        if !matches!(self.mode, MODE_NAVIGATION | MODE_ASSIST) {
            c.axes = [0.0; 3];
        }
        c
    }
}

impl<C: CommandSource + ChargerState> CommandSource for BasicServer<C> {
    fn poll(&mut self, now: f64) -> Option<UserCommand> {
        if let Some(mut c) = self.inner.poll(now) {
            let motion = c.motion_seq != self.last.motion_seq;
            if c.gait != self.last.gait && !known_gait(c.gait) {
                warn!(
                    gait = format!("{:#x}", c.gait),
                    "unknown gait; basic_server drops it"
                );
                c.gait = self.last.gait;
            }
            // cmdHandle_Gait (0x18b430): Extre needs the authorization code (the M20's authority_code.toml is empty)
            if c.gait != self.last.gait && extre_gait(c.gait) && !self.extre_authorized {
                warn!(
                    gait = format!("{:#x}", c.gait),
                    "Extre gait without authorization (0xe007); dropped"
                );
                c.gait = self.last.gait;
            }
            if motion && c.target == Target::Stand {
                self.stand_until = Some(now + self.cfg.stand_wait_s);
            }
            if self
                .gait_override
                .is_some_and(|(_, at)| at != (c.gait, c.gait_seq))
            {
                self.gait_override = None;
            }
            // HandlerVel normalizes a twist once, on receipt, by the gait then commanded; a later gait change keeps it
            if let Some(v) = c.velocity.take() {
                let gait = self.gait_override.map_or(c.gait, |(g, _)| g);
                c.axes = self.real_steer.axes(gait, v);
            }
            self.last = c;
            self.dirty = true;
        }
        if let Some(mode) =
            crate::rt::try_read(&self.usage.request, &crate::rt::LOCK_SKIPS, |r| r.take()).flatten()
        {
            self.set_mode(mode);
        }
        // DDSWorker::ChannelChargerStatusCallback (0x220c60), on a change of charge_manager's state
        let charger = self.inner.charger_state();
        if charger != self.charger {
            self.charger = charger;
            // charge_manager writes the gaits while it docks or undocks
            if charger != 0 && self.gait_override.take().is_some() {
                self.dirty = true;
            }
            match charger {
                0 => self.set_mode(MODE_REGULAR),
                1 | 3 => self.set_mode(MODE_NAVIGATION),
                _ => {}
            }
        }
        if self.dirty {
            self.dirty = false;
            Some(self.out())
        } else {
            None
        }
    }

    fn expires(&self) -> bool {
        self.inner.expires()
    }

    fn observe(&mut self, controller: &Controller) {
        self.inner.observe(controller);
        let now = controller.time();
        if self.usage.align.swap(false, Ordering::Relaxed) {
            if controller.state == State::Passive {
                info!("aligning body posture");
                self.last.target = Target::RlControl;
                self.switch_gait(crate::state_machine::ALIGN_GAIT);
                self.align = Some(Align {
                    next_check: now + self.cfg.align.check_period_s,
                    checks_left: self.cfg.align.checks,
                });
            } else {
                warn!(state = ?controller.state, "aligning body posture needs Idle");
            }
        }
        if let Some(mut a) = self.align {
            if now >= a.next_check {
                let up = controller.last_accel[2] > self.cfg.align.accel_z
                    && controller.motion_info_height() as f64 > self.cfg.align.height_m;
                a.checks_left -= 1;
                a.next_check = now + self.cfg.align.check_period_s;
                if up {
                    self.align = None;
                    self.switch_gait(GAIT_BASIC);
                } else if a.checks_left == 0 {
                    warn!("fall recovery failed: soft e-stop");
                    self.align = None;
                    self.last.target = Target::JointDamping;
                    self.dirty = true;
                } else {
                    self.align = Some(a);
                }
            }
        }
        let Some(until) = self.stand_until else {
            return;
        };
        if controller.state == State::StandUp {
            // the stand is reported: RLControl, SendGait(0x1001), setControlUsageMode(0)
            self.stand_until = None;
            self.last.target = Target::RlControl;
            self.switch_gait(GAIT_BASIC);
            self.set_mode(MODE_REGULAR);
        } else if controller.time() > until {
            warn!("StandingUp timeout");
            self.stand_until = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A scripted operator with a settable charger state.
    struct Script {
        next: Option<UserCommand>,
        charger: i32,
    }

    impl CommandSource for Script {
        fn poll(&mut self, _now: f64) -> Option<UserCommand> {
            self.next.take()
        }
    }

    impl ChargerState for Script {
        fn charger_state(&self) -> i32 {
            self.charger
        }
    }

    fn basic_server_config() -> crate::config::BasicServerConfig {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        crate::config::Config::load(&path).unwrap().basic_server
    }

    fn real_steer_config() -> crate::config::RealSteer {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        crate::config::Config::load(&path).unwrap().real_steer
    }

    fn server() -> (BasicServer<Script>, Arc<UsageMode>) {
        let usage = Arc::new(UsageMode::default());
        let s = BasicServer::new(
            Script {
                next: None,
                charger: 0,
            },
            usage.clone(),
            basic_server_config(),
            real_steer_config(),
        );
        (s, usage)
    }

    fn send(s: &mut BasicServer<Script>, c: UserCommand) -> UserCommand {
        s.inner.next = Some(c);
        s.poll(0.0).unwrap()
    }

    #[test]
    fn velocity_reaches_the_controller_only_in_modes_one_and_two() {
        let (mut s, usage) = server();
        let walk = UserCommand {
            target: Target::RlControl,
            gait: 0x3003,
            velocity: Some([0.5, 0.0, 0.2]),
            ..Default::default()
        };
        assert_eq!(send(&mut s, walk.clone()).axes, [0.0; 3]);
        *usage.request.lock().unwrap() = Some(MODE_ASSIST);
        let out = s.poll(0.0).unwrap();
        // mode 2 also switched to the basic gait
        assert_eq!(out.gait, GAIT_BASIC);
        // HandlerVel on 0x3003: (x / 2, y, yaw / 2), in f32
        assert_eq!(out.axes, [0.25, 0.0, 0.1f32 as f64]);
    }

    #[test]
    fn usage_mode_two_switches_to_basic_and_passes_velocity() {
        let (mut s, usage) = server();
        *usage.request.lock().unwrap() = Some(MODE_ASSIST);
        let _ = s.poll(0.0);
        let out = send(
            &mut s,
            UserCommand {
                target: Target::RlControl,
                gait: 0x3003,
                velocity: Some([0.5, 0.0, 0.2]),
                motion_seq: 1,
                ..Default::default()
            },
        );
        assert_eq!(
            out.gait, 0x3003,
            "the operator's gait after the mode change wins"
        );
        // HandlerVel on 0x3003: (x / 2, y, yaw / 2), in f32
        assert_eq!(out.axes, [0.25, 0.0, 0.1f32 as f64]);
        assert_eq!(usage.current.load(Ordering::Relaxed), MODE_ASSIST);
        // the same mode again does nothing (no gait reset)
        *usage.request.lock().unwrap() = Some(MODE_ASSIST);
        assert!(s.poll(0.0).is_none());
        // navigation mode (1): no gait reset
        *usage.request.lock().unwrap() = Some(MODE_NAVIGATION);
        assert_eq!(s.poll(0.0).unwrap().gait, 0x3003);
    }

    #[test]
    fn charger_states_drive_the_mode_and_the_undock_ends_on_basic() {
        let (mut s, usage) = server();
        *usage.request.lock().unwrap() = Some(MODE_ASSIST);
        let _ = send(
            &mut s,
            UserCommand {
                target: Target::RlControl,
                gait: 0x3003,
                ..Default::default()
            },
        );
        s.inner.charger = 1;
        let _ = s.poll(0.0);
        assert_eq!(s.mode(), MODE_NAVIGATION);
        s.inner.charger = 2;
        assert!(s.poll(0.0).is_none(), "charging changes no mode");
        s.inner.charger = 3;
        let _ = s.poll(0.0);
        assert_eq!(s.mode(), MODE_NAVIGATION);
        // docking restored the gait it started from; the end of the undock (state 0) resets to regular and 0x1001
        let restored = send(
            &mut s,
            UserCommand {
                target: Target::RlControl,
                gait: 0x3003,
                ..Default::default()
            },
        );
        assert_eq!(restored.gait, 0x3003);
        s.inner.charger = 0;
        let out = s.poll(0.0).unwrap();
        assert_eq!(s.mode(), MODE_REGULAR);
        assert_eq!(out.gait, GAIT_BASIC);
        // a velocity-only update keeps the basic gait
        let out = send(
            &mut s,
            UserCommand {
                target: Target::RlControl,
                gait: 0x3003,
                velocity: Some([0.1, 0.0, 0.0]),
                ..Default::default()
            },
        );
        assert_eq!(out.gait, GAIT_BASIC);
        assert_eq!(out.axes, [0.0; 3]);
    }

    #[test]
    fn a_stand_then_enters_rl_control_on_basic_in_regular_mode() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        let cfg = crate::config::Config::load(&path).unwrap();
        let mut c = Controller::new(cfg);
        c.set_time(10.0);
        let (mut s, usage) = server();
        *usage.request.lock().unwrap() = Some(MODE_ASSIST);
        let mut cmd = send(
            &mut s,
            UserCommand {
                target: Target::Stand,
                gait: 0x3003,
                motion_seq: 1,
                ..Default::default()
            },
        );
        // lying (a start with the legs up lies down first)
        let lying = c.config().states.lie_down.lie_height;
        let state = crate::types::RobotState {
            quat: [1.0, 0.0, 0.0, 0.0],
            q: crate::state_machine::leg_pose_for_height(&c.config().robot, lying, 0.0),
            ..Default::default()
        };
        let mut policy = crate::policy::InlinePolicy::new(crate::policy::RunnerSet::empty());
        for _ in 0..3 {
            c.tick(&state, &cmd, &mut policy);
            s.observe(&c);
            if let Some(next) = s.poll(c.time()) {
                cmd = next;
            }
        }
        assert_eq!(c.state, State::StandUp);
        assert_eq!(cmd.target, Target::RlControl);
        assert_eq!(cmd.gait, GAIT_BASIC);
        assert_eq!(s.mode(), MODE_REGULAR);
    }

    #[test]
    fn a_stand_that_never_comes_times_out() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        let cfg = crate::config::Config::load(&path).unwrap();
        let mut c = Controller::new(cfg);
        let (mut s, _) = server();
        let cmd = send(
            &mut s,
            UserCommand {
                target: Target::Stand,
                motion_seq: 1,
                ..Default::default()
            },
        );
        let state = crate::types::RobotState {
            quat: [1.0, 0.0, 0.0, 0.0],
            ..Default::default()
        };
        let mut policy = crate::policy::InlinePolicy::new(crate::policy::RunnerSet::empty());
        // the unchanging joints are a frozen-motor fault by the time the 2 s start-up gate opens: it never stands
        for _ in 0..5200 {
            c.tick(&state, &cmd, &mut policy);
            s.observe(&c);
        }
        assert!(s.stand_until.is_none());
        assert_eq!(s.poll(0.0).map(|c| c.target), None);
    }

    /// One fall recovery: ticks the controller, feeding the accel / height the robot reaches, until `seconds`.
    fn align_run(
        up_at: Option<f64>,
        seconds: f64,
    ) -> (BasicServer<Script>, Controller, UserCommand) {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml");
        let cfg = crate::config::Config::load(&path).unwrap();
        let robot = cfg.robot.clone();
        let mut c = Controller::new(cfg);
        c.set_time(10.0);
        let (mut s, usage) = server();
        let mut cmd = UserCommand::default();
        let mut policy = crate::policy::InlinePolicy::new(crate::policy::RunnerSet::empty());
        usage.align.store(true, Ordering::Relaxed);
        let start = c.time();
        while c.time() - start < seconds {
            let t = c.time() - start;
            let up = up_at.is_some_and(|u| t >= u);
            let mut state = crate::types::RobotState {
                quat: [1.0, 0.0, 0.0, 0.0],
                accel: [0.0, 0.0, if up { 9.8 } else { 3.0 }],
                ..Default::default()
            };
            // standing (hip 0.38 m above the axle) or lying
            let h = if up { 0.38 } else { 0.03 };
            state.q = crate::state_machine::leg_pose_for_height(&robot, h, 0.0);
            state.stamp = t;
            state.tau = [t; crate::config::JOINTS];
            c.tick(&state, &cmd, &mut policy);
            s.observe(&c);
            if let Some(next) = s.poll(c.time()) {
                cmd = next;
            }
        }
        (s, c, cmd)
    }

    #[test]
    fn a_fall_recovery_enters_rl_control_on_0xf003_and_returns_to_basic_once_up() {
        let (s, c, cmd) = align_run(Some(2.5), 4.0);
        assert!(c
            .transitions
            .iter()
            .any(|(_, from, to)| *from == State::Passive && *to == State::RlControl));
        // up at 2.5 s: the 3 s check switches to 0x1001
        assert_eq!(cmd.gait, GAIT_BASIC);
        assert!(s.align.is_none());
    }

    #[test]
    fn a_fall_recovery_that_never_gets_up_soft_e_stops_after_7_checks() {
        let (_, _, cmd) = align_run(None, 6.5);
        assert_eq!(cmd.target, Target::RlControl);
        assert_eq!(cmd.gait, crate::state_machine::ALIGN_GAIT);
        let (s, _, cmd) = align_run(None, 7.5);
        assert_eq!(cmd.target, Target::JointDamping);
        assert!(s.align.is_none());
    }

    #[test]
    fn extre_gaits_need_the_authorization_code() {
        let (mut s, _) = server();
        let _ = send(
            &mut s,
            UserCommand {
                gait: 0x3003,
                ..Default::default()
            },
        );
        let jump = UserCommand {
            gait: 0x2002,
            ..Default::default()
        };
        assert_eq!(send(&mut s, jump.clone()).gait, 0x3003);
        s.extre_authorized = true;
        assert_eq!(send(&mut s, jump).gait, 0x2002);
    }

    /// DdsInterface::HandlerVel's table and its slow-speed boost on the stair gaits.
    #[test]
    fn a_twist_is_normalized_by_the_commanded_gait_on_receipt() {
        let r = real_steer_config();
        let close = |a: [f64; 3], b: [f64; 3]| a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-6);
        assert!(close(r.axes(0x1001, [1.0, 0.4, 0.75]), [0.5, 0.5, 0.5]));
        assert!(close(r.axes(0x3001, [0.25, 0.15, 0.25]), [0.5, 0.5, 0.5]));
        assert!(close(r.axes(0x3002, [1.0, 0.5, 0.75]), [0.5, 0.5, 0.5]));
        assert!(
            close(r.axes(0xf001, [1.0, 0.2, 0.3]), [0.5, 0.5, 0.5]),
            "default"
        );
        // stairs: 0.1 m/s forward is boosted to 0.2, below 0.1 not, 0.2 and up not
        assert!(close(r.axes(0x3003, [0.1, 0.05, 0.0]), [0.1, 0.1, 0.0]));
        assert!(close(r.axes(0x3003, [0.09, 0.0, 0.0]), [0.045, 0.0, 0.0]));
        assert!(close(r.axes(0x3003, [0.3, 0.0, 0.0]), [0.15, 0.0, 0.0]));
        assert!(
            close(r.axes(0x1002, [0.15, 0.0, 0.0]), [0.1, 0.0, 0.0]),
            "no boost off the stair gaits"
        );
        // the gait at receipt counts: a twist sent on 0x1001 keeps 0x1001's scaling after a switch to 0x3003
        let (mut s, usage) = server();
        *usage.request.lock().unwrap() = Some(MODE_NAVIGATION);
        let out = send(
            &mut s,
            UserCommand {
                gait: 0x1001,
                velocity: Some([1.0, 0.0, 0.0]),
                ..Default::default()
            },
        );
        assert!(close(out.axes, [0.5, 0.0, 0.0]));
        let out = send(
            &mut s,
            UserCommand {
                gait: 0x3003,
                gait_seq: 1,
                axes: out.axes,
                ..Default::default()
            },
        );
        assert!(close(out.axes, [0.5, 0.0, 0.0]));
    }

    #[test]
    fn unknown_gaits_are_dropped() {
        let (mut s, _) = server();
        let _ = send(
            &mut s,
            UserCommand {
                gait: 0x3002,
                ..Default::default()
            },
        );
        let out = send(
            &mut s,
            UserCommand {
                gait: 0x1234,
                ..Default::default()
            },
        );
        assert_eq!(out.gait, 0x3002);
    }
}
