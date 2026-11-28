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

//! Shadow (dry-run) mode: run the controller next to rl_deploy on the robot from passive inputs only and log what
//! it would have sent beside what rl_deploy sent, every cycle. Built only with the "shadow" feature and without
//! "ecan", so the motor-bus transport is not in the binary at all; on Linux it also installs a seccomp filter that
//! makes every send / write on its sockets fail.
//!
//! Inputs: the ecan_tap.so shim's live stream (rl_deploy's received joint states and sent commands, unix datagram),
//! the body IMU and the velocity commands sniffed off DDS (receive-only packet socket), and rl_deploy's state / gait
//! log (journalctl). Mirrors dimos.simulation.m20_policy_sim.replay, which settled every rule here against a
//! recording.

pub mod config;
pub mod engine;
pub mod journal;
pub mod log;
pub mod seccomp;
pub mod session;
pub mod sniff;
pub mod tap;
