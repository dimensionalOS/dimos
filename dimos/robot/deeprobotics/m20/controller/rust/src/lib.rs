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

//! M20 motor controller, a replacement for the vendor rl_deploy: the rl_deploy state machine, its ONNX policy
//! runners (observation and action pipelines per m20_controller.toml), a 1 kHz real-time loop with the policy on its
//! own thread, and two hardware backends (MuJoCo over a socket for tests, the eth1 ECAN motor bus).

pub mod commands;
pub mod config;
pub mod frames;
pub mod hardware;
pub mod policy;
pub mod rt;
pub mod runner;
#[cfg(feature = "shadow")]
pub mod shadow;
pub mod state_machine;
pub mod types;
