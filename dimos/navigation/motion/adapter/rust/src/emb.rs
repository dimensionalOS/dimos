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

//! What the adapters do with the body they are configured with. The body
//! itself arrives in the config, deserialised from `embodiment/base.py`'s
//! record -- there is no table here to drift from it.

use dimos_motion2_target::planner::Emb;
use dimos_motion2_tc::geom::Params;
use dimos_motion2_tc::laws::blind::BlindParams;
use dimos_motion2_tc::laws::hinted::HintedParams;
use dimos_motion2_tc::stamps;

/// `Embodiment.dilated`, formula for formula: every box grown by `by` PER
/// SIDE, the measured rows with it.
pub fn dilated(emb: Emb, by: f64) -> Emb {
    if by == 0.0 {
        return emb;
    }
    let pad = 2.0 * by;
    Emb {
        length: emb.length + pad,
        width: emb.width + pad,
        envelope: emb
            .envelope
            .iter()
            .map(|r| [r[0], r[1] + pad, r[2] + pad, r[3], r[4]])
            .collect(),
        ..emb
    }
}

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

pub fn half_width(emb: &Emb) -> f64 {
    emb.width / 2.0
}

/// The stamp dialect's curve, read off the body the plan was made for.
pub fn governor(emb: &Emb) -> stamps::Governor {
    stamps::Governor {
        max_speed: emb.max_speed,
        min_speed: emb.min_speed,
        speed_clearance: emb.speed_clearance,
        floor: emb.precision,
        max_yaw_rate: emb.max_yaw_rate,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dilation_grows_every_box_per_side_and_nothing_else() {
        let e = Emb::go2();
        let d = dilated(e.clone(), 0.05);
        assert!((d.length - e.length - 0.10).abs() < 1e-12);
        assert!((d.width - e.width - 0.10).abs() < 1e-12);
        for (a, b) in d.envelope.iter().zip(&e.envelope) {
            assert_eq!(a[0], b[0]);
            assert!((a[1] - b[1] - 0.10).abs() < 1e-12 && (a[2] - b[2] - 0.10).abs() < 1e-12);
            assert_eq!((a[3], a[4]), (b[3], b[4]));
        }
        assert_eq!(d.precision, e.precision);
        assert_eq!(dilated(e.clone(), 0.0).length, e.length);
    }

    #[test]
    fn the_body_round_trips_through_its_config_json() {
        // the python module sends `Embodiment` as a dict; this is that dict
        let e = Emb::go2();
        let back: Emb = serde_json::from_str(&serde_json::to_string(&e).unwrap()).unwrap();
        assert_eq!(back.envelope, e.envelope);
        assert_eq!(back.tag, "go2");
        assert_eq!(governor(&back), governor(&e));
    }
}
