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
//! (`RustTargetEpisode.plan` marshals the numbers itself from
//! `embodiment.py`), and the crate has no business carrying the OTHER three
//! tags, whose overrides only exist for the benchmark. The
//! fixture has gone stale before -- it carried the 0.31 m trunk width for a
//! while after the measured moving-body envelope landed (`1e4750b03`) -- and
//! deploying against a stale one would plan for a body narrower than the one
//! that walks, and hand the follower's governor a half-width the planner did
//! not use.
//!
//! So the deployed table is written against `embodiment.py`, which is the
//! source of truth both python modules read. If `embodiment.py` moves, this
//! moves with it, and `go2_matches_the_pure_crate_fixture` below is the pin
//! that says so out loud.

use dimos_motion2_target::planner::{Emb, GO2_ENVELOPE};
use dimos_motion2_tc::stamps;

/// `embodiment.py::GO2` -- the all-gait union, plus the per-heading envelope
/// rows measured over the governed slow band (`planner/envelope_results.md`).
fn go2() -> Emb {
    Emb {
        length: 0.883,
        width: 0.593,
        center_off: 0.002,
        comfort: 0.4,
        precision: 0.05,
        max_speed: 0.5,
        min_speed: 0.2,
        speed_clearance: 0.35,
        max_yaw_rate: 1.4,
        command_slew: [2.5, 2.0, 5.0],
        gait_band: [0.45, 0.95],
        walk_gain: 0.964,
        walk_slip: 0.132,
        walk_slip_ramp: 0.08,
        strafe: 1.8,
        reverse: 1.5,
        yaw_w: 0.25,
        envelope: GO2_ENVELOPE.to_vec(),
        arc_inflate: 0.0334,
    }
}

/// The other three tags have no measured envelope of their own and fall back
/// to the union at every heading, exactly as `embodiment.py` leaves them.
fn unmeasured(emb: Emb) -> Emb {
    Emb {
        envelope: Vec::new(),
        arc_inflate: 0.0,
        ..emb
    }
}

/// Every box grown by `by` PER SIDE; negative shrinks it.
///
/// `embodiment.py::Embodiment.dilated`, formula for formula. The table's
/// numbers are measured -- the swinging legs, not the trunk, set the width --
/// so a negative value is a deployment planning tighter than the legs measured.
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

/// The body for an embodiment tag, or `None` when the tag is unknown.
///
/// The four tags are `scenarios.py::EMBODIMENTS`, each spelled as that entry's
/// overrides on top of the `GO2` defaults.
pub fn by_tag(tag: &str) -> Option<Emb> {
    let emb = match tag {
        "go2" => go2(),
        // payload adds 8 cm in front: longer body, centre 4 cm further forward
        "go2-payload" => unmeasured(Emb {
            length: 0.963,
            center_off: 0.042,
            comfort: 0.5,
            ..go2()
        }),
        "slim" => unmeasured(Emb {
            length: 2.0,
            width: 0.24,
            comfort: 0.3,
            ..go2()
        }),
        // cannot crab
        "diffdrive" => unmeasured(Emb {
            strafe: 50.0,
            reverse: 3.0,
            ..go2()
        }),
        _ => return None,
    };
    Some(emb)
}

/// The body's vertical geometry, all measured from the surface the feet stand
/// on -- `embodiment.py`'s `steppable` / `height` / `base_height`.
///
/// Not on `Emb`: that is the pure crate's type, and the search does not read
/// these. The obstacle models do (`obstacles.rs`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vert {
    /// Legs negotiate obstacles below this -- at a cost (later).
    pub steppable: f64,
    /// Above this the body passes underneath; not an obstacle.
    pub height: f64,
    /// Base origin above the support surface; frame plumbing, not semantics.
    pub base_height: f64,
}

/// The vertical geometry for an embodiment tag, or `None` when it is unknown.
pub fn vert_by_tag(tag: &str) -> Option<Vert> {
    let go2 = Vert {
        steppable: 0.20,
        height: 0.45,
        base_height: 0.29,
    };
    match tag {
        // only diffdrive differs: no legs to step over anything with
        "diffdrive" => Some(Vert {
            steppable: 0.0,
            ..go2
        }),
        t if by_tag(t).is_some() => Some(go2),
        _ => None,
    }
}

/// The tags a config may name, for the validation error message.
pub const TAGS: [&str; 4] = ["go2", "go2-payload", "slim", "diffdrive"];

/// Half the body width -- the offset both the planner's stamped profile and the
/// follower's recomputed hint subtract, so they must read it the same way
/// (`control/world.py` and `adapter/follower.py` both take `emb.width / 2`).
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
    fn go2_carries_the_measured_envelope_not_the_trunk() {
        let e = by_tag("go2").expect("go2 is a known tag");
        // embodiment.py GO2: the swinging legs set the union's width, not the
        // 0.31 m trunk, and the union is the honest-conservative shape
        assert_eq!(e.width, 0.593);
        assert_eq!(e.length, 0.883);
        assert_eq!(e.center_off, 0.002);
        assert_eq!(half_width(&e), 0.2965);
        // ...and the per-heading rows, without which a doorway the trunk walks
        // through reads as a wall
        assert_eq!(e.envelope.len(), 9);
        assert_eq!(e.envelope[0], [0.0, 0.819, 0.416, -0.023, 0.000]);
        assert_eq!(e.arc_inflate, 0.0334);
    }

    #[test]
    fn overrides_sit_on_top_of_the_go2_defaults() {
        let p = by_tag("go2-payload").expect("known tag");
        assert_eq!(p.length, 0.963);
        assert_eq!(p.comfort, 0.5);
        assert_eq!(p.width, 0.593); // inherited
        let d = by_tag("diffdrive").expect("known tag");
        assert_eq!(d.strafe, 50.0);
        assert_eq!(d.length, 0.883); // inherited
                                     // but never the go2's measured envelope: nobody measured theirs, and
                                     // the union is the only honest fallback
        for tag in ["go2-payload", "slim", "diffdrive"] {
            let e = by_tag(tag).expect("known tag");
            assert!(e.envelope.is_empty(), "{tag} inherited the go2's rows");
            assert_eq!(
                e.arc_inflate, 0.0,
                "{tag} inherited the go2's arc inflation"
            );
        }
    }

    /// The deployed table and the pure crate's fixture describe the same robot.
    /// They are written twice on purpose (see the module note); this is the pin
    /// that catches one of them moving without the other.
    #[test]
    fn go2_matches_the_pure_crate_fixture() {
        let (a, b) = (by_tag("go2").expect("known tag"), Emb::go2());
        assert_eq!(a.length, b.length);
        assert_eq!(a.width, b.width);
        assert_eq!(a.center_off, b.center_off);
        assert_eq!(a.comfort, b.comfort);
        assert_eq!(a.precision, b.precision);
        assert_eq!(a.envelope, b.envelope);
        assert_eq!(a.arc_inflate, b.arc_inflate);
    }

    #[test]
    fn an_unknown_tag_is_refused_rather_than_defaulted() {
        assert!(by_tag("go3").is_none());
        assert!(by_tag("").is_none());
        for tag in TAGS {
            assert!(by_tag(tag).is_some(), "{tag} is advertised but unknown");
        }
    }

    #[test]
    fn every_tag_carries_vertical_geometry_too() {
        for tag in TAGS {
            assert!(vert_by_tag(tag).is_some(), "{tag} has no vertical geometry");
        }
        assert!(vert_by_tag("go3").is_none());
    }

    #[test]
    fn the_go2_vertical_geometry_is_embodiment_py() {
        let v = vert_by_tag("go2").expect("go2");
        assert_eq!(v.steppable, 0.20);
        assert_eq!(v.height, 0.45);
        assert_eq!(v.base_height, 0.29);
        // a diffdrive has no legs to negotiate anything with
        assert_eq!(vert_by_tag("diffdrive").expect("known").steppable, 0.0);
    }
}
