// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Repulsive-field (navigation-function) local planner core.
//!
//! Pure algorithm crate: `costmap` builds the level-aware occupancy + chamfer
//! clearance field from raw points; `solver` runs the wavefront plan. Bindings
//! are feature-gated so the same core serves the dimos native module
//! ("native"), offline Python tests ("python"), and the browser demo ("wasm").

pub mod costmap;
pub mod solver;

#[cfg(feature = "python")]
mod python;

#[cfg(feature = "wasm")]
mod wasm;
