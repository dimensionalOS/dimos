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

//! dimos-gsc-pgo: loop closure for a lidar keyframe pose graph, in the order a scan meets the stages:
//!
//! - stage 0 (`gsc_pgo`): keyframes and the threads that keep compute bounded.
//! - stage 1 `uniqueness`: is this scan distinctive enough to place.
//! - stage 2 `similarity_discovery`: which earlier keyframes it might be (scan context, position).
//! - stage 3 `connection_fit`: how well the two places line up (ICP).
//! - stage 4 `proportional_confidence`: extended ICP until the evidence matches the yank.
//! - stage 5 `big_picture_fit`: the pose graph solve (GNC) deciding which loops agree.
//!
//! `mat3` and `pointcloud` are the small math underneath. The module executable (`src/main.rs`,
//! binary `gsc-pgo`) builds behind the default `native` feature.

pub mod big_picture_fit;
pub mod connection_fit;
pub mod gsc_pgo;
pub mod mat3;
pub mod pointcloud;
pub mod proportional_confidence;
pub mod similarity_discovery;
pub mod uniqueness;
