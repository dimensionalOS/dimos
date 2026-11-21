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

//! The M20 controller: the dimos module that owns the motor bus (joint state, batteries, charge enable; feature
//! "module"), with a replacement for the vendor rl_deploy built in: the rl_deploy state machine, its ONNX policy
//! runners (observation and action pipelines per m20_controller.toml), a 1 kHz real-time loop with the policy on its
//! own thread, and the hardware backends (MuJoCo over a socket for tests, the eth1 ECAN motor bus).

#![cfg_attr(
    not(test),
    deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)
)]

pub mod commands;
pub mod config;
pub mod dock;
pub mod frames;
pub mod hardware;
#[cfg(feature = "module")]
pub mod module;
pub mod policy;
#[cfg(feature = "module")]
pub mod publish;
pub mod rt;
pub mod runner;
#[cfg(feature = "shadow")]
pub mod shadow;
pub mod state_machine;
pub mod types;
