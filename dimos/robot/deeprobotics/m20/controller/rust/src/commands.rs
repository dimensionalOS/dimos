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

//! Operator commands: JSON [UserCommand](crate::types::UserCommand) datagrams over UDP, or a timed script.

use std::net::UdpSocket;

use anyhow::{Context, Result};
use serde::Deserialize;

use crate::types::UserCommand;

pub trait CommandSource: Send {
    /// A new command, if one arrived; `now` is seconds since the loop started.
    fn poll(&mut self, now: f64) -> Option<UserCommand>;
    /// Whether silence means the operator is gone (velocity axes are then zeroed after command_timeout_s).
    fn expires(&self) -> bool {
        true
    }
    /// The controller after each tick (a source that sequences the controller, like docking, watches it here).
    fn observe(&mut self, _controller: &crate::state_machine::Controller) {}
}

/// Non-blocking UDP: each datagram is one JSON UserCommand; the newest one per tick wins.
pub struct UdpCommands {
    socket: UdpSocket,
    buf: Vec<u8>,
    pub rejected: u64,
}

impl UdpCommands {
    pub fn bind(addr: &str) -> Result<UdpCommands> {
        let socket = UdpSocket::bind(addr).with_context(|| format!("binding {addr}"))?;
        socket.set_nonblocking(true)?;
        Ok(UdpCommands {
            socket,
            buf: vec![0u8; 65536],
            rejected: 0,
        })
    }
}

impl CommandSource for UdpCommands {
    fn poll(&mut self, _now: f64) -> Option<UserCommand> {
        let mut newest = None;
        // EINTR and WouldBlock both just end this tick's read
        while let Ok(n) = self.socket.recv(&mut self.buf) {
            match serde_json::from_slice::<UserCommand>(&self.buf[..n]) {
                Ok(c) => newest = Some(c),
                Err(_) => self.rejected += 1,
            }
        }
        newest
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct ScriptStep {
    /// seconds since the loop started
    pub at: f64,
    pub command: UserCommand,
}

/// Commands at fixed times (tests and bench runs); never expires.
pub struct ScriptedCommands {
    steps: Vec<ScriptStep>,
    next: usize,
}

impl ScriptedCommands {
    pub fn new(mut steps: Vec<ScriptStep>) -> ScriptedCommands {
        steps.sort_by(|a, b| a.at.total_cmp(&b.at));
        ScriptedCommands { steps, next: 0 }
    }

    pub fn load(path: &std::path::Path) -> Result<ScriptedCommands> {
        let text =
            std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        Ok(ScriptedCommands::new(serde_json::from_str(&text)?))
    }

    pub fn end(&self) -> f64 {
        self.steps.last().map_or(0.0, |s| s.at)
    }
}

impl CommandSource for ScriptedCommands {
    fn poll(&mut self, now: f64) -> Option<UserCommand> {
        let mut out = None;
        while self.next < self.steps.len() && self.steps[self.next].at <= now {
            out = Some(self.steps[self.next].command.clone());
            self.next += 1;
        }
        out
    }
    fn expires(&self) -> bool {
        false
    }
}

/// The vendor drdds-zenoh bridge (main.cpp publish_cycle): the latest twist resent each cycle, zero past the timeout.
pub struct NavBridge<C> {
    inner: C,
    period_s: f64,
    timeout_s: f64,
    command: Option<UserCommand>,
    latest: [f64; 3],
    received_at: f64,
    active: bool,
    next_cycle: f64,
}

impl<C: CommandSource> NavBridge<C> {
    pub fn new(inner: C, rate_hz: f64, timeout_s: f64) -> Self {
        NavBridge {
            inner,
            period_s: 1.0 / rate_hz,
            timeout_s,
            command: None,
            latest: [0.0; 3],
            received_at: 0.0,
            active: false,
            next_cycle: 0.0,
        }
    }
}

impl<C: CommandSource> CommandSource for NavBridge<C> {
    fn poll(&mut self, now: f64) -> Option<UserCommand> {
        let mut out = None;
        if let Some(c) = self.inner.poll(now) {
            let previous = self.command.as_ref();
            let control = previous.is_none_or(|p| {
                (p.motion_seq, p.gait_seq, p.target, p.gait)
                    != (c.motion_seq, c.gait_seq, c.target, c.gait)
            });
            if let Some(v) = c.velocity {
                if !control {
                    self.latest = v;
                    self.received_at = now;
                    self.active = true;
                }
            } else {
                self.active = false;
            }
            let held = previous.and_then(|p| p.velocity);
            let mut c = c;
            // the twist goes out on the bridge's cycle; until then the last one sent holds
            if !control {
                c.velocity = held;
            }
            self.command = Some(c.clone());
            if control {
                out = Some(c);
            }
        }
        if now >= self.next_cycle {
            self.next_cycle = (self.next_cycle + self.period_s).max(now);
            if let (true, Some(c)) = (self.active, self.command.as_mut()) {
                c.velocity = Some(if now - self.received_at > self.timeout_s {
                    self.active = false;
                    [0.0; 3]
                } else {
                    self.latest
                });
                out = Some(c.clone());
            }
        }
        out
    }

    fn expires(&self) -> bool {
        false
    }

    fn observe(&mut self, controller: &crate::state_machine::Controller) {
        self.inner.observe(controller);
    }
}

#[cfg(test)]
mod nav_bridge_tests {
    use super::*;

    /// The cmd_vel stamps of a live drive as a source: each twist at its arrival time.
    struct Replay {
        twists: Vec<(f64, [f64; 3])>,
        at: usize,
    }

    impl CommandSource for Replay {
        fn poll(&mut self, now: f64) -> Option<UserCommand> {
            let mut out = None;
            while self.at < self.twists.len() && self.twists[self.at].0 <= now {
                out = Some(UserCommand {
                    velocity: Some(self.twists[self.at].1),
                    ..Default::default()
                });
                self.at += 1;
            }
            out
        }
    }

    fn capture() -> Vec<(f64, [f64; 3])> {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/cmd_vel_20261124_060657.txt");
        std::fs::read_to_string(path)
            .unwrap()
            .lines()
            .filter(|l| !l.starts_with('#'))
            .map(|l| {
                let v: Vec<f64> = l.split_whitespace().map(|x| x.parse().unwrap()).collect();
                (v[0], [v[1], v[2], v[3]])
            })
            .collect()
    }

    /// The bridge's output each 1 ms over a twist sequence: (time, velocity sent, the latest twist received).
    fn run(twists: &[(f64, [f64; 3])], end: f64) -> Vec<(f64, [f64; 3], [f64; 3])> {
        let (dt, mut out, mut sent, mut latest) = (0.001, Vec::new(), [0.0; 3], [0.0; 3]);
        let mut bridge = NavBridge::new(
            Replay {
                twists: twists.to_vec(),
                at: 0,
            },
            10.0,
            0.4,
        );
        let mut k = 0;
        while (k as f64) * dt < end {
            let now = k as f64 * dt;
            if let Some(t) = twists.iter().take_while(|t| t.0 <= now).last() {
                latest = t.1;
            }
            if let Some(v) = bridge.poll(now).and_then(|c| c.velocity) {
                sent = v;
            }
            out.push((now, sent, latest));
            k += 1;
        }
        out
    }

    #[test]
    fn the_live_drive_never_zeroes_a_moving_command() {
        // its gaps while moving reach 0.26 s (rl_deploy's own 0.155 s would zero there); the bridge's 0.4 s does not
        let twists = capture();
        let trace = run(&twists, twists.last().unwrap().0 + 1.0);
        let end = twists.last().unwrap().0;
        // each cycle sends the latest twist: a zero while the latest twist moves is the timeout's
        let timeout_zeros: Vec<f64> = trace
            .windows(2)
            .filter(|w| {
                w[1].0 < end && w[1].1 != w[0].1 && w[1].1 == [0.0; 3] && w[1].2 != [0.0; 3]
            })
            .map(|w| w[1].0)
            .collect();
        assert!(timeout_zeros.is_empty(), "zeroed at {timeout_zeros:?}");
    }

    #[test]
    fn a_silence_over_the_timeout_zeroes_on_the_next_cycle() {
        let twists = vec![(0.0, [0.5, 0.0, 0.3]), (0.35, [0.5, 0.0, 0.3])];
        let trace = run(&twists, 1.5);
        let zero_at = trace
            .iter()
            .find(|(_, sent, _)| *sent == [0.0; 3])
            .map(|(t, ..)| *t)
            .unwrap();
        // last twist 0.35 s, timeout 0.4 s: the cycle after 0.75 s, 0.8 s
        assert!((zero_at - 0.8).abs() < 0.002, "zeroed at {zero_at}");
        // until then the twist was resent
        assert!(trace
            .iter()
            .filter(|(t, ..)| *t < 0.8)
            .all(|(_, sent, _)| *sent == [0.5, 0.0, 0.3]));
    }

    /// One scripted command per poll.
    struct Script(Vec<UserCommand>);

    impl CommandSource for Script {
        fn poll(&mut self, _now: f64) -> Option<UserCommand> {
            (!self.0.is_empty()).then(|| self.0.remove(0))
        }
    }

    #[test]
    fn motion_commands_pass_at_once_and_twists_wait_for_the_cycle() {
        let walk = UserCommand {
            velocity: Some([0.5, 0.0, 0.0]),
            ..Default::default()
        };
        let lie = UserCommand {
            target: crate::types::Target::LieDown,
            motion_seq: 1,
            ..Default::default()
        };
        let mut bridge = NavBridge::new(Script(vec![walk.clone(), walk, lie]), 10.0, 0.4);
        // the first command (and cycle) at 0
        assert!(bridge.poll(0.0).is_some());
        // a twist between cycles waits
        assert!(bridge.poll(0.05).is_none());
        let lie = bridge.poll(0.06).expect("a motion command at once");
        assert_eq!(lie.target, crate::types::Target::LieDown);
        assert!(!bridge.expires());
    }
}
