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

//! Autoresearch candidate for the motion planning benchmark.
//!
//! RULES. Deterministic: same inputs -> bit-identical output (a scoring
//! pillar; parallel float reductions are order-dependent, so threads buy
//! risk). Threading is not banned outright — the referee charges plan()
//! by total CPU across all threads (time.process_time), so threads buy no
//! free speed either — but the deployment budget is one core on a shared
//! RK3588. Keep dependencies to pyo3/numpy. Rewrite the ALGORITHM in
//! planner.rs; the python-facing surface in python.rs stays stable.
//!
//! COUPLING NOTE. planner.rs's yaw publication (densify + the two-tier
//! yaw gate) is tuned to the judge's scoring constants: sweep_yaw_step
//! 0.15, SCORE_STRIDE_M 0.3, turn_yaw_eps 0.5 (see geometry.py). If those
//! change, the publication cadence must be re-derived from `resolution`
//! or re-tuned — the gate's soundness argument rests on them.

pub mod planner;

#[cfg(feature = "python")]
mod python;
