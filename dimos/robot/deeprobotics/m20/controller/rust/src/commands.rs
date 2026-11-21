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
