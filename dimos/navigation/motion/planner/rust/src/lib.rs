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

//! The SE(2) local planner crate.
//!
//! RULES. Deterministic: same inputs -> bit-identical output (parallel float
//! reductions are order-dependent, so threads buy risk), and the deployment
//! budget is one core on a shared RK3588. Keep dependencies to pyo3/numpy.
//! Rewrite the ALGORITHM in planner.rs; the python-facing surface in
//! py/src/lib.rs stays stable.
//!
//! COUPLING NOTE. planner.rs's yaw publication (densify + the two-tier yaw
//! gate) is derived from the station constants it declares (`YAW_STEP`,
//! `MAX_STATION_YAW`, `SCORE_STRIDE_M`). If those change, the publication
//! cadence must be re-derived from `resolution` — the gate's soundness
//! argument rests on them.

pub mod planner;
