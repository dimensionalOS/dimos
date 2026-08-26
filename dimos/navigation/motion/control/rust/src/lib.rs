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

//! Onboard trajectory controllers for the RK3588: the motion2 pursuit laws
//! with no python in the tick.
//!
//! LAYOUT. `geom` and `stamps` are shared facilities; `laws/` holds one
//! module per law: `hinted`, which the follower runs, and `seed`, the
//! permanent A/B baseline. A research generation lands by replacing
//! `laws/hinted.rs` -- never by editing `laws/seed.rs`, and never by changing
//! the meaning of an existing `geom` function, which would move the baseline
//! as a side effect.
//!
//! RULES. Every law is a PORT, not a redesign -- its `control/laws/*.py` twin
//! is the specification and `control/test_rust_parity.py` holds the two to
//! 1e-9 per component. Any change to an algorithm has to land on the python
//! side first. Single-threaded and deterministic; dependencies stay at
//! pyo3/numpy.
//!
//! STATE. Neither law keeps any. One that does would be a `#[pyclass]`
//! rather than a free function, `reset()` would have to make a used instance
//! indistinguishable from a fresh one, and its parity would be replayed as a
//! SEQUENCE (a single call would only ever prove tick one). Determinism holds
//! either way: no wall clock, no unseeded randomness, and the tick time
//! arrives as an argument.
//!
//! NUMERICS. Parity is per-operation, not per-formula: `geom.rs` keeps the
//! python's operation ORDER and its exact tie-breaks (`argmin` takes the
//! first minimum, `searchsorted` is side='left'), and angle wrapping is IEEE
//! remainder like `math.remainder`, never `%` or `rem_euclid`.

pub mod clearance;
pub mod emb;
pub mod geom;
pub mod laws;
pub mod stamps;

#[cfg(feature = "python")]
mod python;
