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
//! - stage 5 `big_picture_fit`: the pose graph solve (GNC) deciding which closures agree.
//!
//! `mat3` and `pointcloud` are the small math underneath. The module executable (`src/main.rs`,
//! binary `gsc-pgo`), the offline eval (`src/eval.rs`, binary `pgo-eval`) and the `records` they share build
//! behind the default `native` feature.

pub mod adoption_schedule;
pub mod big_picture_fit;
pub mod closure_search_schedule;
pub mod connection_fit;
pub mod gsc_pgo;
pub mod icp_cache;
pub mod landmarks;
pub mod loop_overlap;
pub mod mat3;
pub mod odometry_error_estimate;
#[cfg(test)]
mod odometry_error_estimate_overtrust;
pub mod pointcloud;
pub mod pose_graph_solving;
pub mod pressure_schedule;
pub mod store;
pub mod proportional_confidence;
#[cfg(feature = "native")]
pub mod records;
pub mod similarity_discovery;
pub mod uniqueness;
pub mod world_positions;
