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

//! dimos-gsc-pgo — Rust port of the gsc_pgo pose-graph-optimization core.
//!
//! Ground truth is the C++ module at jeff-hykin's gsc_pgo (simple_pgo.cpp,
//! scan_context.{h,cpp}, branch remove-tag-handling). Layout:
//!
//! - `gtsam`: safe wrappers over a hand-written C FFI shim
//!   (shim/gtsam_shim.{h,cpp}) exposing exactly the gtsam surface the C++
//!   core uses — ISAM2 incremental update with factor removal, Pose3
//!   prior/between factors, Values, diagonal/gaussian/robust-Huber noise.
//! - `scan_context`: a faithful port of the Scan Context descriptor
//!   (ring/sector max-z binning, ring key, shifted cosine distance,
//!   structure/occupancy degeneracy proxies).
//! - `mat3`: small 3x3/vector helpers + a Jacobi symmetric eigensolver.
//! - `pointcloud`: the PCL pieces the core leans on — voxel-grid centroid
//!   downsampling, kd-tree NN, point-to-point ICP with PCL convergence
//!   semantics, and the normal-scatter degeneracy measure.
//! - `simple_pgo`: the `SimplePgo` port itself (keyframe gating, loop
//!   search + gates, location constraints with revision, iSAM2 smoothing).
//! - `msgs`: the jnav custom LCM wire formats the module executable speaks
//!   (Graph3D / GraphDelta3D / DeformationNode encode, LocationConstraint
//!   decode) — byte-for-byte ports of the Python implementations.
//!
//! The module executable itself (`src/main.rs`, binary `gsc-pgo`) runs the PGO
//! on the `dimos_module` framework; it builds behind the default `native`
//! feature.
//!
//! Building needs gtsam (and its Eigen/boost headers); see build.rs for the
//! env-var contract and flake.nix for the pinned environment
//! (`nix develop path:. --command cargo test`).

pub mod gtsam;
pub mod mat3;
pub mod msgs;
pub mod pointcloud;
pub mod scan_context;
pub mod simple_pgo;
