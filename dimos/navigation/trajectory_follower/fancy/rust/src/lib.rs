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

//! Onboard trajectory controllers: the pursuit laws with no python in the tick. `laws/hinted` is what the
//! follower runs, `laws/seed` the frozen A/B baseline. Every law is a stateless port of its `control/laws/*.py`
//! twin, held to 1e-9 by `control/test_rust_parity.py` (python op order, first-min argmin, IEEE-remainder angles).

pub mod emb;
pub mod laws;

// The path dialect and its geometry are the planner's: it encodes what these laws decode.
pub use dimos_local_planner::{clearance, geom, stamps};

#[cfg(feature = "module")]
pub mod module;

#[cfg(feature = "python")]
mod python;
