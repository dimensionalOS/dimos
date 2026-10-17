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

//! A law's parameters, read off the body it drives. The body itself arrives
//! as `embodiment/base.py`'s record (a module's config, or the JSON the
//! python wrappers pass); there is no table here to drift from it.

use dimos_local_planner::planner::Emb;

use crate::laws::hinted::HintedParams;

// The base tuning and the governor are read off the body by the planner's crate,
// which encodes the dialect these laws decode.
pub use dimos_local_planner::emb::{base_params, governor};

pub fn hinted_params(emb: &Emb) -> HintedParams {
    HintedParams {
        base: base_params(emb, [emb.min_speed, emb.max_speed]),
        walk_gain: emb.walk_gain,
        walk_slip: emb.walk_slip,
        slip_ramp: emb.walk_slip_ramp,
    }
}
