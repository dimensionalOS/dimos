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

//! The SEED law's behavioural cases, mirroring `test_controller.py` so a
//! regression shows up in `cargo test` and not only under pytest. Exact
//! agreement with the python is a separate gate -- `test_rust_parity.py`.
//!
//! This law is the permanent baseline, so these cases are frozen: a research
//! generation that wants different behaviour lands in its own track's law and
//! brings its own test file (see `blind.rs`).
//!
//! Run with `cargo test --release --no-default-features` to skip the pyo3
//! link; the crate exposes the laws as an rlib.

use std::f64::consts::PI;

use dimos_motion2_target::planner::Emb;
use dimos_motion2_tc::emb::base_params;
use dimos_motion2_tc::geom::{ieee_remainder, Params, TAU};
use dimos_motion2_tc::laws::seed::update;

/// The fixture's tuning inside its governor band.
fn cfg() -> Params {
    let e = Emb::fixture();
    base_params(&e, [e.min_speed, e.max_speed])
}

/// 4 m of straight path along +x at yaw 0, the python `_straight_path()`.
fn straight() -> Vec<[f64; 3]> {
    (0..40).map(|k| [k as f64 * 0.1, 0.0, 0.0]).collect()
}

/// Rotate in place at the origin, then walk away: the python `_fan_path()`.
fn fan() -> Vec<[f64; 3]> {
    let mut p: Vec<[f64; 3]> = [0.0, 0.3, 0.6, 0.9, 1.2, 1.5]
        .iter()
        .map(|&y| [0.0, 0.0, y])
        .collect();
    p.push([0.1, 0.0, 1.5]);
    p.push([1.0, 0.1, 1.5]);
    p
}

#[test]
fn on_path_drives_forward() {
    let (vx, vy, wz) = update((0.0, 0.0, 0.0), &straight(), None, &cfg());
    assert!(vx > 0.3, "vx={vx}");
    assert!(vy.abs() < 1e-6 && wz.abs() < 1e-6, "vy={vy} wz={wz}");
}

#[test]
fn lateral_offset_commands_crab_back() {
    // path is at y=0, robot at +0.3: crab right while still advancing
    let (vx, vy, _) = update((1.0, 0.3, 0.0), &straight(), None, &cfg());
    assert!(vy < -0.1 && vx > 0.0, "vx={vx} vy={vy}");
}

#[test]
fn behind_the_path_still_drives_onto_it() {
    // the carrot is ahead of the closest waypoint, which is the path start
    let (vx, vy, _) = update((-1.0, 0.0, 0.0), &straight(), None, &cfg());
    assert!(vx > 0.0 && vy.abs() < 1e-12, "vx={vx} vy={vy}");
}

#[test]
fn speed_and_yaw_rate_clamped() {
    let cfg = cfg();
    let (vx, vy, _) = update((-2.0, -2.0, 0.0), &straight(), None, &cfg);
    assert!(
        vx.hypot(vy) <= cfg.max_speed + 1e-12,
        "speed={}",
        vx.hypot(vy)
    );
    let (_, _, wz) = update((0.0, 0.0, PI - 0.1), &straight(), None, &cfg);
    assert!(wz.abs() <= cfg.max_yaw_rate + 1e-12, "wz={wz}");
    // and the clamp is a saturation, not a sign flip: facing +y off a yaw-0
    // path turns clockwise
    let (_, _, wz) = update((0.0, 0.0, PI / 2.0), &straight(), None, &cfg);
    assert!(wz < -0.5, "wz={wz}");
}

#[test]
fn fan_rotates_in_place() {
    let (vx, vy, wz) = update((0.0, 0.0, 0.0), &fan(), None, &cfg());
    assert!(wz > 0.3, "wz={wz}");
    assert!(
        vx.hypot(vy) < 0.15,
        "holds position: speed={}",
        vx.hypot(vy)
    );
}

#[test]
fn fan_done_resumes_translation() {
    let (vx, vy, _) = update((0.0, 0.0, 1.45), &fan(), None, &cfg());
    assert!(vx > 0.1 || vy != 0.0, "vx={vx} vy={vy}");
}

/// Coincident fan waypoints carry no arc, so `argmin` alone would park on the
/// fan's first pose and re-command the whole rotation every tick. The yaw
/// advance walks forward while the next pose is a better yaw match, so a
/// robot already at 0.6 rad sees the REST of the fan.
#[test]
fn fan_advances_by_yaw_progress() {
    let cfg = cfg();
    let p = fan(); // yaws 0.0 .. 1.5 in 0.3 steps, all at the origin
                   // At yaw 0.6 the carrot is 0.9, one step ON: k_yaw * 0.3. Without the
                   // advance the index stays pinned at the fan's first pose and the command
                   // is k_yaw * (0.3 - 0.6) = -0.6 -- rotating back into the fan.
    let (_, _, wz_mid) = update((0.0, 0.0, 0.6), &p, None, &cfg);
    assert!(
        (wz_mid - cfg.k_yaw * 0.3).abs() < 1e-9,
        "mid-fan target is not the next pose: wz={wz_mid}"
    );
    // a robot past the whole fan is out of it entirely and translates again
    let (vx, vy, _) = update((0.0, 0.0, 1.5), &p, None, &cfg);
    assert!(
        vx.hypot(vy) > 0.05,
        "still stuck rotating: speed={}",
        vx.hypot(vy)
    );
}

#[test]
fn governor_creeps_in_tight_room() {
    let cfg = cfg();
    let p = straight();
    let tight = vec![0.06; p.len()]; // barely above the precision floor
    let (vx, vy, _) = update((0.0, 0.0, 0.0), &p, Some(&tight), &cfg);
    assert!(
        vx.hypot(vy) <= cfg.min_speed + 0.02,
        "speed={}",
        vx.hypot(vy)
    );
}

#[test]
fn governor_is_open_room_and_a_no_op_when_blind() {
    let cfg = cfg();
    let p = straight();
    let wide = vec![1.0; p.len()];
    let open = update((-1.0, 0.0, 0.0), &p, Some(&wide), &cfg);
    let blind = update((-1.0, 0.0, 0.0), &p, None, &cfg);
    assert_eq!(open, blind, "wide clearance changed the command");
    // a wrong-length annotation is ignored, exactly as in the python
    let stub = vec![0.06; p.len() - 1];
    assert_eq!(update((-1.0, 0.0, 0.0), &p, Some(&stub), &cfg), blind);
}

/// The window is [arc_i, arc_i + speed_lookahead]: room already behind the
/// robot does not govern it.
#[test]
fn governor_reads_room_ahead_not_behind() {
    let cfg = cfg();
    let p = straight();
    let mut clear = vec![1.0; p.len()];
    clear[..5].fill(0.06);
    let (vx, vy, _) = update((1.5, 0.0, 0.0), &p, Some(&clear), &cfg);
    assert!(vx.hypot(vy) > cfg.min_speed + 0.1, "speed={}", vx.hypot(vy));
}

#[test]
fn empty_and_single_pose_paths_hold_position() {
    let cfg = cfg();
    assert_eq!(update((1.0, 2.0, 0.3), &[], None, &cfg), (0.0, 0.0, 0.0));
    let stub = [[5.0, 5.0, 1.0]];
    assert_eq!(update((1.0, 2.0, 0.3), &stub, None, &cfg), (0.0, 0.0, 0.0));
}

/// `math.remainder` semantics: half-to-even quotient, so the wrap lands in
/// [-pi, pi] and `remainder(pi, tau)` is `+pi`, not `-pi`. `%` and
/// `rem_euclid` both get this wrong.
#[test]
fn ieee_remainder_matches_python() {
    for (x, want) in [
        (PI, PI),
        (-PI, -PI),
        // 3pi/tau is exactly 1.5, and half rounds to the EVEN 2 -- so the
        // wrap goes down to -pi. Truncation would leave +pi here.
        (3.0 * PI, -PI),
        (2.5 * PI, PI / 2.0),
        (TAU + 0.25, 0.25),
        (-TAU - 0.25, -0.25),
        (0.0, 0.0),
    ] {
        let got = ieee_remainder(x, TAU);
        assert!(
            (got - want).abs() < 1e-12,
            "remainder({x}, tau) = {got}, want {want}"
        );
    }
}

/// The law is stateless: the same tick twice is bit-identical, and no call
/// leaves a trace in the next one.
#[test]
fn stateless_and_deterministic() {
    let cfg = cfg();
    let p = straight();
    let a = update((0.3, -0.2, 0.4), &p, None, &cfg);
    update((9.0, 9.0, 3.0), &fan(), None, &cfg);
    let b = update((0.3, -0.2, 0.4), &p, None, &cfg);
    assert_eq!(a.0.to_bits(), b.0.to_bits());
    assert_eq!(a.1.to_bits(), b.1.to_bits());
    assert_eq!(a.2.to_bits(), b.2.to_bits());
}
