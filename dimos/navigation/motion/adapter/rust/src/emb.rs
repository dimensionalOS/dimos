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

//! Body dimensions by embodiment tag, keyed to `scenarios.py::EMBODIMENTS`.
//!
//! WHY THE TABLE IS HERE AND NOT IN THE PURE CRATE. `dimos_motion2_target`
//! has an `Emb::go2()`, but it is a TEST FIXTURE: the python never calls it
//! (`RustTargetEpisode.plan` marshals the eight numbers itself from
//! `scenarios.py`), and it has since gone stale -- it still carries the
//! 0.31 m trunk width from before the measured moving-body envelope landed
//! (`1e4750b03`), where `scenarios.py` now says 0.50. Deploying against it
//! would plan for a body 19 cm narrower than the one that walks, and would
//! hand the follower's governor a half-width the planner did not use.
//!
//! So the deployed table is written against `scenarios.py`, which is the
//! source of truth both python modules read, and the pure crate's fixture is
//! left alone. If `scenarios.py` moves, this moves with it.

use dimos_motion2_target::planner::Emb;

/// `scenarios.py::Embodiment` field defaults, i.e. `GO2`.
fn go2() -> Emb {
    Emb {
        length: 0.85,
        width: 0.50,
        center_off: -0.01,
        comfort: 0.4,
        precision: 0.05,
        strafe: 1.8,
        reverse: 1.5,
        yaw_w: 0.25,
    }
}

/// The body for an embodiment tag, or `None` when the tag is unknown.
///
/// The four tags are `scenarios.py::EMBODIMENTS`, each spelled as that entry's
/// overrides on top of the `GO2` defaults.
pub fn by_tag(tag: &str) -> Option<Emb> {
    let emb = match tag {
        "go2" => go2(),
        // payload adds 8 cm in front: longer body, centre 4 cm further forward
        "go2-payload" => Emb {
            length: 0.93,
            center_off: 0.03,
            comfort: 0.5,
            ..go2()
        },
        "slim" => Emb {
            length: 2.0,
            width: 0.24,
            comfort: 0.3,
            ..go2()
        },
        // cannot crab
        "diffdrive" => Emb {
            strafe: 50.0,
            reverse: 3.0,
            ..go2()
        },
        _ => return None,
    };
    Some(emb)
}

/// The tags a config may name, for the validation error message.
pub const TAGS: [&str; 4] = ["go2", "go2-payload", "slim", "diffdrive"];

/// Half the body width -- the offset both the planner's stamped profile and the
/// follower's recomputed hint subtract, so they must read it the same way
/// (`control/world.py` and `adapter/follower.py` both take `emb.width / 2`).
pub fn half_width(emb: &Emb) -> f64 {
    emb.width / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn go2_carries_the_measured_envelope_not_the_trunk() {
        let e = by_tag("go2").expect("go2 is a known tag");
        // scenarios.py GO2: the swinging legs set the width, not the 0.31 m
        // trunk the pure crate's test fixture still names
        assert_eq!(e.width, 0.50);
        assert_eq!(e.length, 0.85);
        assert_eq!(e.center_off, -0.01);
        assert_eq!(half_width(&e), 0.25);
    }

    #[test]
    fn overrides_sit_on_top_of_the_go2_defaults() {
        let p = by_tag("go2-payload").expect("known tag");
        assert_eq!(p.length, 0.93);
        assert_eq!(p.comfort, 0.5);
        assert_eq!(p.width, 0.50); // inherited
        let d = by_tag("diffdrive").expect("known tag");
        assert_eq!(d.strafe, 50.0);
        assert_eq!(d.length, 0.85); // inherited
    }

    #[test]
    fn an_unknown_tag_is_refused_rather_than_defaulted() {
        assert!(by_tag("go3").is_none());
        assert!(by_tag("").is_none());
        for tag in TAGS {
            assert!(by_tag(tag).is_some(), "{tag} is advertised but unknown");
        }
    }
}
