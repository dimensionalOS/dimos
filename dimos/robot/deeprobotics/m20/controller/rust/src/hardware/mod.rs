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

//! The motor bus as the controller sees it: send one 1 ms joint command, get the joint and IMU state back.

#[cfg(feature = "ecan")]
pub mod bus;
#[cfg(feature = "ecan")]
pub mod ecan;
pub mod mujoco;

use anyhow::Result;

use crate::types::{JointCommand, RobotState};

pub trait Hardware: Send {
    /// The current state without commanding the motors (startup).
    fn read_state(&mut self) -> Result<RobotState>;
    /// Send one control tick's command and return the state the bus replies with.
    fn exchange(&mut self, cmd: &JointCommand) -> Result<RobotState>;
}
