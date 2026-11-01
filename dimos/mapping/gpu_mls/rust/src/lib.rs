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

//! GPU-resident voxel mapping and MLS surface extraction on wgpu compute: a
//! port of the ray tracer's update_map and the MLS planner's surface stages.
//!
//! # Building and running
//!
//! `nix build .#default` builds the tests and bench (`result/bin/gpu-mls-tests`, `gpu-mls-bench`).
//! On Linux they are wrapped to reach the host's NVIDIA Vulkan driver, which nix's glibc cannot
//! find on its own; `nix develop` in this directory sets up the same for `cargo test`. On a
//! Jetson the Vulkan ICD also needs glvnd's libEGL and libEGL_nvidia, or it silently fails.
//!
//! `nix build .#cross-aarch64-linux` (or `cross-x86_64-linux`) from any machine, this Mac
//! included, builds the same binaries linked against glibc 2.35 with zig. Copy `result/` to the
//! Jetson (`rsync -aL`); they use the host's Vulkan directly.
//!
//! Set `DIMOS_DATA_DIR` to the repo's `data` directory when running a binary built elsewhere,
//! or the real-map test and bench rows are skipped. Check the printed adapter names the GPU,
//! not llvmpipe. Brick pools are bounded by the adapter's storage binding limit (about 2 GB on
//! Orin); see `BrickMap::max_brick_capacity`.

pub mod brick_map;
pub mod gpu;
pub mod surfaces;
pub mod synthetic;

/// The repo's `data` directory: `DIMOS_DATA_DIR` when set, which a binary
/// built elsewhere (nix, cross) needs, else the checkout this was built from.
pub fn data_dir() -> std::path::PathBuf {
    std::env::var_os("DIMOS_DATA_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| {
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../data")
        })
}
