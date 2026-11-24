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

/// SetMotionState(1)'s wait for rl_deploy to report the stand (waitForMotionStateChange(1, 5000)).
const STAND_WAIT_S: f64 = 5.0;

/// The gait codes SwitchGait (0x261640) sends; it logs and drops any other.
pub fn known_gait(gait: u32) -> bool {
    matches!(
        gait,
        0x1001..=0x1005 | 0x2001..=0x2004 | 0x3001..=0x3005 | 0x5001..=0x5003 | 0xf001..=0xf003
    )
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
        Phase::StandUp | Phase::BackOff | Phase::Restore => 3,
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
}

impl<C: CommandSource + ChargerState> BasicServer<C> {
    pub fn new(inner: C, usage: Arc<UsageMode>) -> Self {
        BasicServer {
            inner,
            usage,
            mode: MODE_REGULAR,
            charger: 0,
            last: UserCommand::default(),
            gait_override: None,
            stand_until: None,
            dirty: false,
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
            c.velocity = c.velocity.map(|_| [0.0; 3]);
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
            if motion && c.target == Target::Stand {
                self.stand_until = Some(now + STAND_WAIT_S);
            }
            if self
                .gait_override
                .is_some_and(|(_, at)| at != (c.gait, c.gait_seq))
            {
                self.gait_override = None;
            }
            self.last = c;
            self.dirty = true;
        }
        if let Some(mode) = self.usage.request.lock().ok().and_then(|mut r| r.take()) {
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

    fn server() -> (BasicServer<Script>, Arc<UsageMode>) {
        let usage = Arc::new(UsageMode::default());
        let s = BasicServer::new(
            Script {
                next: None,
                charger: 0,
            },
            usage.clone(),
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
        assert_eq!(send(&mut s, walk.clone()).velocity, Some([0.0; 3]));
        *usage.request.lock().unwrap() = Some(MODE_ASSIST);
        let out = s.poll(0.0).unwrap();
        // mode 2 also switched to the basic gait
        assert_eq!(out.gait, GAIT_BASIC);
        assert_eq!(out.velocity, Some([0.5, 0.0, 0.2]));
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
        assert_eq!(out.velocity, Some([0.5, 0.0, 0.2]));
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
        assert_eq!(out.velocity, Some([0.0; 3]));
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
        let state = crate::types::RobotState {
            quat: [1.0, 0.0, 0.0, 0.0],
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
