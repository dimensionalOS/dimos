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

//! The robot-side transport shell for the motion stack: `motion_planner` and
//! `trajectory_follower` as native dimos modules.
//!
//! WHAT LIVES HERE AND WHAT DOES NOT. Nothing in this crate is an algorithm.
//! The planner is `dimos_motion2_target::planner::plan`, the laws are
//! `dimos_motion2_tc::laws`, the room hint is
//! `dimos_motion2_tc::clearance::path_clearance` and the stamp dialect is
//! `dimos_motion2_tc::stamps` -- all four are parity-locked to their python
//! twins, and a behaviour change has to land python-first. This crate owns
//! subscriptions, the fixed-rate loops, staleness, goal arrival and the
//! message marshalling, which is exactly what `adapter/planner.py` and
//! `adapter/follower.py` own on the python side.
//!
//! The python modules stay the reference implementation. These two exist so
//! the last stage of the loop can run on the Go2 without python in the tick.

pub mod emb;
pub mod follower;
pub mod msg;
pub mod obstacles;
pub mod planner;
pub mod tf_pose;
