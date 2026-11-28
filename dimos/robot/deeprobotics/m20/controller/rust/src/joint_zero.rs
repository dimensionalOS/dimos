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

//! The app's joint zeroing (ResetJointsZero 1101/8) as AOS AutoSetHome -1 does it (main 0x1058f0); see [joint_zero] in the toml.

use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU64, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Mutex;

use serde::Serialize;

use crate::app_commands::error;
use crate::state_machine::State;
use crate::types::Target;

/// Between whoever asks (the app server, the module's motion_cmd) and the 1 kHz loop that runs it.
#[derive(Default)]
pub struct Requests {
    /// a request the loop has not taken yet, and where its result code goes
    slot: Mutex<Option<(Sender<i32>, Vec<usize>)>>,
    running: AtomicBool,
    abort: AtomicBool,
    finished: AtomicU64,
    last: AtomicI32,
}

/// The zeroing in controller_report.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
pub struct Report {
    pub running: bool,
    /// zeroings finished (or refused) so far
    pub finished: u64,
    /// the last one's code: 0 done, joint + 1 (AutoSetHome's exit code), or an app error code
    pub last_code: i32,
}

impl Requests {
    /// Ask for a zeroing; its code arrives on the receiver (0xe008 while one is asked or running).
    pub fn request(&self, joints: Vec<usize>) -> Result<Receiver<i32>, i32> {
        let mut slot = self.slot.lock().unwrap_or_else(|p| p.into_inner());
        if slot.is_some() || self.running() {
            return Err(error::NOT_ALLOWED);
        }
        let (tx, rx) = channel();
        *slot = Some((tx, joints));
        Ok(rx)
    }

    /// The loop's take: never waits (a request being written is taken next tick).
    pub fn take(&self) -> Option<(Sender<i32>, Vec<usize>)> {
        crate::rt::try_read(&self.slot, &crate::rt::LOCK_SKIPS, |s| s.take()).flatten()
    }

    pub fn running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }

    pub fn set_running(&self, on: bool) {
        self.running.store(on, Ordering::Relaxed);
        if !on {
            self.abort.store(false, Ordering::Relaxed);
        }
    }

    /// A soft e-stop during the zeroing: it ends at once with the last Disable frame.
    pub fn abort(&self) {
        if self.running() {
            self.abort.store(true, Ordering::Relaxed);
        }
    }

    pub fn take_abort(&self) -> bool {
        self.abort.swap(false, Ordering::Relaxed)
    }

    /// Answer a request with `code` and count it.
    pub fn finish(&self, to: &Sender<i32>, code: i32) {
        self.last.store(code, Ordering::Relaxed);
        self.finished.fetch_add(1, Ordering::Relaxed);
        let _ = to.send(code);
    }

    pub fn report(&self) -> Report {
        Report {
            running: self.running(),
            finished: self.finished.load(Ordering::Relaxed),
            last_code: self.last.load(Ordering::Relaxed),
        }
    }
}

/// main 0x1058f0's arguments: "-1" (ours: "all") is all 16, else indices 0-15 in order; ours refuses any other or none.
pub fn parse_joints<S: AsRef<str>>(args: &[S]) -> Result<Vec<usize>, String> {
    let mut joints = Vec::new();
    for a in args {
        let a = a.as_ref().trim();
        if a == "-1" || a == "all" {
            return Ok((0..crate::config::JOINTS).collect());
        }
        match a.parse::<usize>() {
            Ok(j) if j < crate::config::JOINTS => joints.push(j),
            _ => return Err(format!("invalid joint {a:?}: -1 / all, or indices 0-15 (robot order fl fr hl hr x hipx hipy knee wheel)")),
        }
    }
    if joints.is_empty() {
        return Err(NO_JOINTS.to_string());
    }
    Ok(joints)
}

/// The answer to a call without joints: nothing is sent.
pub const NO_JOINTS: &str = "reset_joint_zero sends nothing without joints: it presses each given joint into its mechanical \
stop at 5-10 N m for 6 s and rewrites its driver's stored zero (the robot lying or passive, off the dock, allow_gains on); \
call reset_joint_zero(-1) (or \"all\") for all 16, or joint indices 0-15 (robot order fl fr hl hr x hipx hipy knee wheel)";

/// What the gate reads on the tick a request is taken.
#[derive(Debug, Clone, Copy)]
pub struct Conditions {
    pub state: State,
    /// the controller's lie-down is over (it commands nothing)
    pub at_rest: bool,
    /// the operator's target
    pub target: Target,
    /// basic_server's charger state (0 idle: not docking, on the dock or charging)
    pub charger_state: i32,
    /// a charger is on the packs (the battery frames)
    pub charger_connected: bool,
    /// the ECAN board's hard-stop byte is set
    pub hard_stop: bool,
    /// ecan.allow_gains: the owner lets the press torque out
    pub allow_gains: bool,
}

/// Why not now, as the app's ErrorCode; None: go.
pub fn refusal(c: &Conditions) -> Option<i32> {
    // cmdHandle_ResetJointsZero: MotionState 0 (idle) or 4 (lie down), else 0xe008
    let vendor = matches!(c.state, State::Passive | State::LieDown);
    // ours: the lie-down finished, no stand asked, not on or near the dock, the motors powered and allowed to act
    let ours = c.at_rest
        && matches!(c.target, Target::Passive | Target::LieDown)
        && c.charger_state == 0
        && !c.charger_connected
        && !c.hard_stop
        && c.allow_gains;
    (!(vendor && ours)).then_some(error::NOT_ALLOWED)
}

#[cfg(feature = "ecan")]
pub use sequence::*;

#[cfg(feature = "ecan")]
mod sequence {
    use m20_ecan::joint::CommandIndex;
    use m20_ecan::{JointCommand, JointId, JointState};

    use crate::app_commands::error;
    use crate::config::{JointZero, JOINTS};

    /// One tick of the zeroing.
    #[derive(Debug, Clone, PartialEq)]
    pub enum Step {
        /// send this frame's commands
        Send(Vec<JointCommand>),
        /// send nothing (AutoSetHome sleeps)
        Quiet,
        /// over: 0, the first failing joint + 1, or 0xe009 when aborted
        Done(i32),
    }

    /// AutoSetHome's frames and check, one tick at a time.
    pub struct Sequence {
        cfg: JointZero,
        selected: [bool; JOINTS],
        /// the joints in the order given (the check reports the first failing one in this order, as 0x108c70)
        order: Vec<usize>,
        /// AutoSetHome's unit array: each frame changes some units and sends all 16
        units: Vec<JointCommand>,
        tick: u64,
        /// the last Disable frame went out: replies are checked
        checking: bool,
        /// the newest position each joint replied (motor side)
        positions: [Option<f32>; JOINTS],
        failed: Option<i32>,
        aborted: bool,
    }

    impl Sequence {
        /// `-1`: every joint.
        pub fn all(cfg: &JointZero) -> Sequence {
            Sequence::new(cfg, &(0..JOINTS).collect::<Vec<_>>())
        }

        /// `joints`: indices under 16 (others are dropped), as parse_joints gives them.
        pub fn new(cfg: &JointZero, joints: &[usize]) -> Sequence {
            let order: Vec<usize> = joints.iter().copied().filter(|&j| j < JOINTS).collect();
            let mut selected = [false; JOINTS];
            for &j in &order {
                selected[j] = true;
            }
            // main 0x105bd0: board 0x79, the joint's CAN channel and id (FUN_0010b318's CA9B table), the rest 0, word 4
            let units = JointId::all()
                .map(|joint| JointCommand {
                    board_id: m20_ecan::joint::BOARD_ID,
                    joint,
                    position: 0.0,
                    velocity: 0.0,
                    torque: 0.0,
                    kp: 0.0,
                    kd: 0.0,
                    control_word: CommandIndex::MotorControl as u16,
                    data_id: 0,
                })
                .collect();
            Sequence {
                cfg: cfg.clone(),
                selected,
                order,
                units,
                tick: 0,
                checking: false,
                positions: [None; JOINTS],
                failed: None,
                aborted: false,
            }
        }

        /// The tick the last Disable frame goes out on.
        fn last_frame_tick(&self) -> u64 {
            let c = &self.cfg;
            let s = c.frame_spacing_ticks;
            c.quiet_ticks + (3 + c.press_frames) * s + c.set_home_wait_ticks + c.hold_frames * s
        }

        /// End now: the last Disable frame if anything was sent, then Done(0xe009).
        pub fn abort(&mut self) {
            self.aborted = true;
        }

        fn set(&mut self, all: bool, word: CommandIndex, torque: bool) {
            for (i, u) in self.units.iter_mut().enumerate() {
                if all || self.selected[i] {
                    u.control_word = word as u16;
                    if torque {
                        u.torque = self.cfg.press_torque[i];
                    }
                }
            }
        }

        /// This tick's step.
        pub fn step(&mut self) -> Step {
            let t = self.tick;
            self.tick += 1;
            let c = self.cfg.clone();
            let s = c.frame_spacing_ticks;
            let last = self.last_frame_tick();
            if self.aborted {
                if t > c.quiet_ticks && !self.checking {
                    self.checking = true;
                    self.set(true, CommandIndex::Disable, false);
                    return Step::Send(self.units.clone());
                }
                return Step::Done(error::FAILED);
            }
            let press = c.quiet_ticks + 3 * s;
            let set_home = press + c.press_frames * s;
            let hold = set_home + c.set_home_wait_ticks;
            let on = |start: u64, frames: u64| {
                t >= start && t < start + frames * s && (t - start).is_multiple_of(s)
            };
            if t == c.quiet_ticks {
                self.set(true, CommandIndex::Disable, false);
            } else if t == c.quiet_ticks + s {
                self.set(false, CommandIndex::ErrorReset, false);
            } else if t == c.quiet_ticks + 2 * s {
                self.set(false, CommandIndex::Enable, false);
            } else if on(press, c.press_frames) {
                self.set(false, CommandIndex::MotorControl, true);
            } else if t == set_home {
                self.set(false, CommandIndex::SetHome, false);
            } else if on(hold, c.hold_frames) {
                self.set(false, CommandIndex::MotorControl, false);
            } else if t == last {
                // the torque stays in the units (AutoSetHome never clears it)
                self.set(true, CommandIndex::Disable, false);
                self.checking = true;
            } else if t >= last + c.check_ticks {
                return Step::Done(self.failed.unwrap_or(0));
            } else {
                return Step::Quiet;
            }
            Step::Send(self.units.clone())
        }

        /// A joint-state reply: after the last frame, the first selected joint off its new zero fails the zeroing.
        pub fn reply(&mut self, joints: &[JointState]) {
            for j in joints {
                if let Some(p) = self.positions.get_mut(j.joint.index()) {
                    *p = Some(j.position);
                }
            }
            if !self.checking || self.failed.is_some() || self.aborted {
                return;
            }
            let tol = self.cfg.home_tolerance_rad;
            // 0x108c70: (double)position >= 0.001 or <= -0.001 fails; ours: a non-finite one fails too
            self.failed = self
                .order
                .iter()
                .copied()
                .find(|&i| {
                    self.positions[i].is_some_and(|p| {
                        let p = p as f64;
                        !p.is_finite() || p >= tol || p <= -tol
                    })
                })
                .map(|i| i as i32 + 1);
        }
    }
}

#[cfg(all(test, feature = "ecan"))]
mod tests {
    use super::*;
    use crate::config::{JointZero, JOINTS};
    use m20_ecan::owner::{Owner, OwnerConfig};
    use m20_ecan::{Direction, Frame, JointState, Message};

    fn cfg() -> JointZero {
        crate::config::Config::load(std::path::Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/m20_controller.toml"
        )))
        .unwrap()
        .joint_zero
        .unwrap()
    }

    /// Every tick of a run: (tick, frame commands) for the sends, and the Done tick and code.
    fn run(seq: &mut Sequence) -> (Vec<(u64, Vec<m20_ecan::JointCommand>)>, u64, i32) {
        let mut sent = Vec::new();
        for t in 0.. {
            match seq.step() {
                Step::Send(c) => sent.push((t, c)),
                Step::Quiet => {}
                Step::Done(code) => return (sent, t, code),
            }
        }
        unreachable!()
    }

    /// AutoSetHome's frame body as main builds it at 0x1764d8 (header, then per joint 0x79, k / 4, 2 + k % 4, the floats).
    fn vendor_body(seq: u8, words: [u16; JOINTS], torque: [f32; JOINTS]) -> Vec<u8> {
        let mut payload = Vec::new();
        for k in 0..JOINTS {
            payload.extend_from_slice(&[0x79, (k / 4) as u8]);
            payload.extend_from_slice(&(2 + (k % 4) as u16).to_le_bytes());
            payload.extend_from_slice(&0f32.to_le_bytes());
            payload.extend_from_slice(&0f32.to_le_bytes());
            payload.extend_from_slice(&torque[k].to_le_bytes());
            payload.extend_from_slice(&[0, 0, 0, 0]);
            payload.extend_from_slice(&words[k].to_le_bytes());
            payload.extend_from_slice(&[0, 0]);
        }
        let sum = payload.iter().fold(0u16, |s, &b| s.wrapping_add(b as u16));
        let mut body = vec![0x44, 0x52, seq, 1, 0x12, 0x89];
        body.extend_from_slice(&(payload.len() as u16).to_le_bytes());
        body.extend_from_slice(&sum.to_le_bytes());
        body.extend_from_slice(&payload);
        body
    }

    #[test]
    fn all_joints_get_autosethome_s_frames_on_its_schedule() {
        let c = cfg();
        let (sent, done, code) = run(&mut Sequence::all(&c));
        let torque = c.press_torque;
        let zero = [0.0; JOINTS];
        let (q, s) = (c.quiet_ticks, c.frame_spacing_ticks);
        let mut expected: Vec<(u64, [u16; JOINTS], [f32; JOINTS])> = vec![
            (q, [1; JOINTS], zero),
            (q + s, [17; JOINTS], zero),
            (q + 2 * s, [2; JOINTS], zero),
        ];
        let press = q + 3 * s;
        for k in 0..c.press_frames {
            expected.push((press + k * s, [4; JOINTS], torque));
        }
        let set_home = press + c.press_frames * s;
        expected.push((set_home, [6; JOINTS], torque));
        let hold = set_home + c.set_home_wait_ticks;
        for k in 0..c.hold_frames {
            expected.push((hold + k * s, [4; JOINTS], torque));
        }
        let last = hold + c.hold_frames * s;
        expected.push((last, [1; JOINTS], torque));
        assert_eq!(sent.len(), expected.len());
        for ((t, cmds), (et, words, tq)) in sent.iter().zip(&expected) {
            assert_eq!(t, et);
            // byte for byte the frame AutoSetHome sends (sequence 1 on all of them)
            assert_eq!(
                Message::encode_joint_commands(1, cmds),
                vendor_body(1, *words, *tq),
                "tick {t}"
            );
        }
        // 5010 quiet, 3 + 1000 frames 5 ms apart, SetHome, 100 ms, 200 frames, Disable, 1 s
        assert_eq!(
            (sent.len(), last, done, code),
            (1205, 5010 + 5 * 1003 + 100 + 5 * 200, last + 1000, 0)
        );
    }

    #[test]
    fn some_joints_keep_disable_and_the_check_fails_the_first_joint_off_zero() {
        let c = cfg();
        let selected = [9, 2];
        let mut seq = Sequence::new(&c, &selected);
        let state = |positions: [f32; JOINTS]| -> Vec<JointState> {
            m20_ecan::JointId::all()
                .map(|joint| JointState {
                    board_id: 0x79,
                    joint,
                    position: positions[joint.index()],
                    velocity: 0.0,
                    torque: 0.0,
                    motor_temperature_c: 0.0,
                    driver_temperature_c: 0.0,
                    status_word: 1,
                    data_id: 0,
                })
                .collect()
        };
        let mut off = [0.0; JOINTS];
        // before the last frame nothing is checked; an unselected joint never is
        off[9] = 0.5;
        off[0] = 3.0;
        let mut frames = 0;
        let code = loop {
            match seq.step() {
                Step::Send(cmds) => {
                    frames += 1;
                    let words: Vec<u16> = cmds.iter().map(|c| c.control_word).collect();
                    // ErrorReset, Enable, press, SetHome, hold reach only the selected joints
                    if frames == 2 {
                        let mut w = [1; JOINTS];
                        w[2] = 17;
                        w[9] = 17;
                        assert_eq!(words, w);
                    }
                    if words.contains(&4) {
                        assert_eq!(cmds[0].torque, 0.0);
                        assert_eq!(cmds[9].torque, c.press_torque[9]);
                    }
                    seq.reply(&state(off));
                    if frames == 1204 {
                        off[9] = 0.0;
                    }
                }
                Step::Quiet => {}
                Step::Done(code) => break code,
            }
        };
        assert_eq!(code, 0, "joint 9 read zero after the last frame");
        let mut seq = Sequence::new(&c, &selected);
        let mut code = None;
        while code.is_none() {
            match seq.step() {
                Step::Send(_) => {
                    let mut p = [0.0; JOINTS];
                    p[2] = -0.001;
                    p[9] = f32::NAN;
                    seq.reply(&state(p));
                }
                Step::Quiet => {}
                Step::Done(c) => code = Some(c),
            }
        }
        // 0x108c70: <= -0.001 and NaN fail; the first failing joint in the order given wins (exit code joint + 1)
        assert_eq!(code, Some(10));
    }

    #[test]
    fn an_abort_sends_the_last_disable_frame_once_then_fails() {
        let c = cfg();
        let mut seq = Sequence::all(&c);
        seq.abort();
        assert_eq!(
            seq.step(),
            Step::Done(error::FAILED),
            "nothing sent yet: nothing to disable"
        );
        let mut seq = Sequence::all(&c);
        while !matches!(seq.step(), Step::Send(_)) {}
        for _ in 0..(3 * c.frame_spacing_ticks + 1) {
            seq.step();
        }
        seq.abort();
        let Step::Send(cmds) = seq.step() else {
            panic!("the Disable frame")
        };
        assert!(cmds.iter().all(|c| c.control_word == 1));
        assert_eq!(seq.step(), Step::Done(error::FAILED));
    }

    #[test]
    fn the_owner_with_gains_allowed_sends_the_frames_as_they_are_and_refuses_them_without() {
        let c = cfg();
        let frames: Vec<_> = run(&mut Sequence::all(&c))
            .0
            .into_iter()
            .map(|(_, f)| f)
            .collect();
        for allow_gains in [true, false] {
            let mut owner = Owner::new(OwnerConfig { allow_gains });
            while owner.in_startup() {
                owner.next(None);
            }
            for f in &frames {
                let body = owner.next(Some(f)).body.unwrap();
                let frame = Frame::parse(&body).unwrap();
                let Ok(Message::JointCommands(out)) = Message::decode(&frame, Direction::ToBoard)
                else {
                    panic!("a joint command")
                };
                let pressing = f.iter().any(|c| c.torque != 0.0);
                if allow_gains || !pressing {
                    assert_eq!(&out, f);
                } else {
                    assert_eq!(out, m20_ecan::startup::sitting_commands());
                }
            }
        }
    }

    fn conditions() -> Conditions {
        Conditions {
            state: State::LieDown,
            at_rest: true,
            target: Target::LieDown,
            charger_state: 0,
            charger_connected: false,
            hard_stop: false,
            allow_gains: true,
        }
    }

    #[test]
    fn only_a_resting_robot_off_the_dock_with_its_motors_allowed_is_zeroed() {
        assert_eq!(refusal(&conditions()), None);
        assert_eq!(
            refusal(&Conditions {
                state: State::Passive,
                target: Target::Passive,
                ..conditions()
            }),
            None
        );
        let refused = [
            Conditions {
                state: State::RlControl,
                ..conditions()
            },
            Conditions {
                state: State::StandUp,
                ..conditions()
            },
            Conditions {
                state: State::JointDamping,
                ..conditions()
            },
            // MotionState 3: basic_server refuses it too
            Conditions {
                state: State::StartDamping,
                ..conditions()
            },
            Conditions {
                at_rest: false,
                ..conditions()
            },
            Conditions {
                target: Target::Stand,
                ..conditions()
            },
            Conditions {
                charger_state: 2,
                ..conditions()
            },
            Conditions {
                charger_state: 1,
                ..conditions()
            },
            Conditions {
                charger_connected: true,
                ..conditions()
            },
            Conditions {
                hard_stop: true,
                ..conditions()
            },
            Conditions {
                allow_gains: false,
                ..conditions()
            },
        ];
        for c in refused {
            assert_eq!(refusal(&c), Some(error::NOT_ALLOWED), "{c:?}");
        }
    }

    #[test]
    fn joints_are_autosethome_s_arguments_and_none_or_a_bad_one_sends_nothing() {
        let all: Vec<usize> = (0..JOINTS).collect();
        assert_eq!(parse_joints(&["-1"]), Ok(all.clone()));
        assert_eq!(parse_joints(&["all"]), Ok(all.clone()));
        assert_eq!(parse_joints(&["5", "-1"]), Ok(all));
        assert_eq!(parse_joints(&["7"]), Ok(vec![7]));
        assert_eq!(parse_joints(&["9", "2", "9"]), Ok(vec![9, 2, 9]));
        assert_eq!(parse_joints::<&str>(&[]), Err(NO_JOINTS.to_string()));
        for bad in [["16"], ["-2"], ["knee"], [""]] {
            assert!(
                parse_joints(&bad).unwrap_err().starts_with("invalid joint"),
                "{bad:?}"
            );
        }
        assert!(parse_joints(&["3", "x"]).is_err());
        // a single joint: only it gets ErrorReset, Enable, the press and SetHome; Disable reaches all 16
        let c = cfg();
        let (sent, _, code) = run(&mut Sequence::new(&c, &[7]));
        assert_eq!(code, 0);
        for (_, cmds) in &sent {
            let others: Vec<_> = cmds.iter().filter(|u| u.joint.index() != 7).collect();
            assert!(others
                .iter()
                .all(|u| u.control_word == 1 && u.torque == 0.0));
        }
        assert!(sent
            .iter()
            .any(|(_, f)| f[7].control_word == 6 && f[7].torque == c.press_torque[7]));
    }

    #[test]
    fn one_request_at_a_time_and_its_code_reaches_the_asker() {
        let r = Requests::default();
        let rx = r.request(vec![3]).unwrap();
        assert_eq!(r.request(vec![3]).err(), Some(error::NOT_ALLOWED));
        let (tx, joints) = r.take().unwrap();
        assert_eq!(joints, [3]);
        r.set_running(true);
        assert_eq!(r.request(vec![3]).err(), Some(error::NOT_ALLOWED));
        r.abort();
        assert!(r.take_abort());
        assert!(!r.take_abort());
        r.finish(&tx, 3);
        r.set_running(false);
        assert_eq!(rx.recv(), Ok(3));
        assert_eq!(
            r.report(),
            Report {
                running: false,
                finished: 1,
                last_code: 3
            }
        );
        // a loop gone: the asker's receive ends
        let rx = r.request(vec![3]).unwrap();
        drop(r.take());
        assert!(rx.recv().is_err());
        // an abort outside a zeroing is nothing
        r.abort();
        assert!(!r.take_abort());
    }
}
