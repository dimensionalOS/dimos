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
pub use dimos_motion2_tc::emb::{base_params, blind_params, governor, hinted_params};

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

pub fn half_width(emb: &Emb) -> f64 {
    emb.width / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dilation_grows_every_box_per_side_and_nothing_else() {
        let e = Emb::fixture();
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
        // the python module sends `Embodiment` as a dict
        let e = Emb::fixture();
        let back: Emb = serde_json::from_str(&serde_json::to_string(&e).unwrap()).unwrap();
        assert_eq!(back.envelope, e.envelope);
        assert_eq!(governor(&back), governor(&e));
    }
}
