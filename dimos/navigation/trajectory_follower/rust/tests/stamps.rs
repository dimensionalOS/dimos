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

//! The wire dialect from the producer's end; `hinted.rs` has `decode_ceilings` from
//! the consumer's. Together they pin that a stamp written here reads as the same speed there.

use std::f64::consts::PI;

use dimos_local_planner::planner::Emb;
use dimos_trajectory_follower::emb::hinted_params;
use dimos_trajectory_follower::stamps::{
    decode_ceilings, encode_precision, governor_speed, Governor,
};

fn straight(n: usize, step: f64) -> Vec<[f64; 3]> {
    (0..n).map(|k| [k as f64 * step, 0.0, 0.0]).collect()
}

/// `Emb::fixture()`'s governor, the curve every number below was written against.
const GOV: Governor = Governor {
    max_speed: 0.5,
    min_speed: 0.2,
    speed_clearance: 0.35,
    floor: 0.05,
    max_yaw_rate: 1.4,
};
const MAX_SPEED: f64 = GOV.max_speed;
const MIN_SPEED: f64 = GOV.min_speed;
const SPEED_CLEARANCE: f64 = GOV.speed_clearance;
const FLOOR_CLEARANCE: f64 = GOV.floor;
const MAX_YAW_RATE: f64 = GOV.max_yaw_rate;

#[test]
fn open_room_cruises_and_the_floor_creeps() {
    assert_eq!(governor_speed(f64::INFINITY, &GOV), MAX_SPEED);
    assert_eq!(governor_speed(SPEED_CLEARANCE, &GOV), MAX_SPEED);
    assert_eq!(governor_speed(FLOOR_CLEARANCE, &GOV), MIN_SPEED);
    // below the floor is still the floor
    assert_eq!(governor_speed(0.0, &GOV), MIN_SPEED);
    assert_eq!(governor_speed(-1.0, &GOV), MIN_SPEED);
    let mid = governor_speed((FLOOR_CLEARANCE + SPEED_CLEARANCE) / 2.0, &GOV);
    assert!(mid > MIN_SPEED && mid < MAX_SPEED);
}

#[test]
fn a_stamped_segment_reads_back_as_the_speed_it_was_priced_at() {
    let path = straight(5, 0.4);
    let clearance = vec![f64::INFINITY, 0.3, 0.1, FLOOR_CLEARANCE, 1.0];
    let ts = encode_precision(&path, &clearance, 0.0, &GOV);
    let got = decode_ceilings(&ts, &path, &hinted_params(&Emb::fixture()).base)
        .expect("stamped path decodes");

    // a ceiling belongs to the segment ending at its waypoint: min(clearance[k-1], clearance[k])
    for k in 1..path.len() {
        let want = governor_speed(clearance[k - 1], &GOV).min(governor_speed(clearance[k], &GOV));
        assert!(
            (got[k] - want).abs() < 1e-12,
            "waypoint {k}: decoded {}, priced at {want}",
            got[k]
        );
    }
}

#[test]
fn tighter_room_stamps_a_longer_dt() {
    let path = straight(2, 1.0);
    let roomy = encode_precision(&path, &[f64::INFINITY, f64::INFINITY], 0.0, &GOV);
    let tight = encode_precision(&path, &[FLOOR_CLEARANCE, FLOOR_CLEARANCE], 0.0, &GOV);
    assert!((roomy[1] - 1.0 / MAX_SPEED).abs() < 1e-12);
    assert!((tight[1] - 1.0 / MIN_SPEED).abs() < 1e-12);
    assert!(tight[1] > roomy[1]);
}

#[test]
fn t0_only_offsets_the_stamps() {
    let path = straight(4, 0.3);
    let clearance = vec![0.2; 4];
    let base = encode_precision(&path, &clearance, 0.0, &GOV);
    let shifted = encode_precision(&path, &clearance, 1234.5, &GOV);
    for k in 0..path.len() {
        assert!((shifted[k] - base[k] - 1234.5).abs() < 1e-9);
    }
}

#[test]
fn a_fan_is_priced_by_yaw_not_by_room() {
    // coincident waypoints: dt is the yaw span at MAX_YAW_RATE and clearance must not enter
    let path = vec![[0.0, 0.0, 0.0], [0.0, 0.0, PI / 2.0]];
    let roomy = encode_precision(&path, &[f64::INFINITY, f64::INFINITY], 0.0, &GOV);
    let tight = encode_precision(&path, &[FLOOR_CLEARANCE, FLOOR_CLEARANCE], 0.0, &GOV);
    assert!((roomy[1] - (PI / 2.0) / MAX_YAW_RATE).abs() < 1e-12);
    assert_eq!(roomy[1], tight[1]);
}

#[test]
fn a_fan_takes_the_short_way_round() {
    // -3pi/4 to +3pi/4 is a quarter turn through pi, not three quarters the other way
    let path = vec![[0.0, 0.0, -3.0 * PI / 4.0], [0.0, 0.0, 3.0 * PI / 4.0]];
    let ts = encode_precision(&path, &[], 0.0, &GOV);
    assert!((ts[1] - (PI / 2.0) / MAX_YAW_RATE).abs() < 1e-12);
}

#[test]
fn clearance_of_the_wrong_length_prices_everything_at_cruise() {
    let path = straight(4, 0.5);
    let ts = encode_precision(&path, &[0.05, 0.05], 0.0, &GOV);
    for k in 1..path.len() {
        assert!((ts[k] - ts[k - 1] - 0.5 / MAX_SPEED).abs() < 1e-12);
    }
    assert_eq!(ts, encode_precision(&path, &[], 0.0, &GOV));
}

#[test]
fn degenerate_paths_do_not_panic() {
    assert!(encode_precision(&[], &[], 0.0, &GOV).is_empty());
    // a single pose is the planner's refusal stub
    assert_eq!(
        encode_precision(&[[1.0, 2.0, 0.3]], &[0.1], 7.0, &GOV),
        vec![7.0]
    );
}

#[test]
fn a_nan_clearance_creeps_rather_than_panicking() {
    // a panic in the planner tick is worse than the creep this degrades to
    let path = straight(2, 1.0);
    let ts = encode_precision(&path, &[f64::NAN, f64::NAN], 0.0, &GOV);
    assert!(ts[1].is_finite());
    assert!((ts[1] - 1.0 / MIN_SPEED).abs() < 1e-12);
}
