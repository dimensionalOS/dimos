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

//! dimos-gsc-pgo — pose-graph-optimization core. Layout:
//!
//! - `gtsam`: safe wrappers over a hand-written C FFI shim
//!   (shim/gtsam_shim.{h,cpp}) exposing the gtsam surface the core uses —
//!   ISAM2 incremental update with factor removal, Pose3 prior/between
//!   factors, Values, diagonal/gaussian/robust-Huber noise.
//! - `scan_context`: the Scan Context descriptor (ring/sector max-z binning,
//!   ring key, shifted cosine distance, structure/occupancy degeneracy
//!   proxies).
//! - `mat3`: small 3x3/vector helpers + a Jacobi symmetric eigensolver.
//! - `pointcloud`: voxel-grid centroid downsampling, kd-tree NN,
//!   point-to-point ICP, and the normal-scatter degeneracy measure.
//! - `gsc_pgo`: the `GscPgo` core itself (keyframe gating, loop search +
//!   gates, location constraints with revision, iSAM2 smoothing).
//! - `gnc`: the batch GNC re-solve (`solve_gnc`) and its background worker,
//!   run alongside `GscPgo`'s iSAM2 smoother.
//! - `msgs`: the jnav custom LCM wire formats the module executable speaks
//!   (Graph3D / GraphDelta3D / DeformationNode encode, LocationConstraint
//!   decode).
//!
//! The module executable itself (`src/main.rs`, binary `gsc-pgo`) runs the PGO
//! on the `dimos_module` framework; it builds behind the default `native`
//! feature.
//!
//! Building needs gtsam (and its Eigen/boost headers); see build.rs for the
//! env-var contract and flake.nix for the pinned environment
//! (`nix develop path:. --command cargo test`).

pub mod gnc;
pub mod gsc_pgo;
pub mod gtsam;
pub mod mat3;
pub mod msgs;
pub mod pointcloud;
pub mod scan_context;
