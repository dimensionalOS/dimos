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

//! A law's parameters, read off the body it drives -- `controller.law_params`
//! and friends. The body itself arrives in a module's config, deserialised
//! from `embodiment/base.py`'s record; there is no table here to drift from it.

use dimos_motion2_target::planner::Emb;

use crate::geom::Params;
use crate::laws::blind::BlindParams;
use crate::laws::hinted::HintedParams;

/// `controller.law_params`: the body's tuning plus its plant, driving inside
/// `band` -- the governor's for seed/blind, the gait's for hinted.
pub fn base_params(emb: &Emb, band: [f64; 2]) -> Params {
    let c = &emb.control;
    Params {
        lookahead: c.lookahead,
        max_speed: band[1],
        max_yaw_rate: emb.max_yaw_rate,
        k_pos: c.k_pos,
        k_yaw: c.k_yaw,
        fan_yaw_per_m: c.fan_yaw_per_m,
        fan_yaw_done: c.fan_yaw_done,
        min_speed: band[0],
        speed_clearance: emb.speed_clearance,
        speed_floor_clearance: emb.precision,
        speed_lookahead: c.speed_lookahead,
    }
}

pub fn hinted_params(emb: &Emb) -> HintedParams {
    let c = &emb.control;
    HintedParams {
        base: base_params(emb, emb.gait_band),
        slew: emb.command_slew,
        tangent_preview: c.tangent_preview,
        escape_clearance: c.escape_clearance,
        escape_preview: c.escape_preview,
        escape_speed: c.escape_speed,
        brake_accel: c.brake_accel,
        brake_margin: c.brake_margin,
    }
}

pub fn blind_params(emb: &Emb) -> BlindParams {
    BlindParams {
        base: base_params(emb, [emb.min_speed, emb.max_speed]),
        walk_gain: emb.walk_gain,
        walk_slip: emb.walk_slip,
        slip_ramp: emb.walk_slip_ramp,
    }
}
