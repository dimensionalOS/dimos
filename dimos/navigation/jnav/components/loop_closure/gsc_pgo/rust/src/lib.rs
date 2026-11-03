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

//! Lidar keyframe loop closure; stages: `gsc_pgo` (keyframes, threads), `uniqueness`, `similarity_discovery`,
//! `connection_fit` (ICP), `proportional_confidence` (extended ICP), `big_picture_fit` (GNC solve).

pub mod adoption_schedule;
pub mod big_picture_fit;
pub mod closure_search_schedule;
pub mod connection_fit;
pub mod fast_path;
pub mod graph_simplify;
pub mod gsc_pgo;
pub mod icp_cache;
pub mod landmarks;
pub mod lidar_range;
pub mod loop_overlap;
pub mod mat3;
pub mod odometry_error_estimate;
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
