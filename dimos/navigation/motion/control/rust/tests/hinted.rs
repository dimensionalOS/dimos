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

use std::f64::consts::PI;

use dimos_motion2_tc::geom::{ieee_remainder, Params, TAU};
use dimos_motion2_tc::laws::hinted::{update, HintedParams, Law};

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
    let (vx, vy, wz) = update((0.0, 0.0, 0.0), &straight(), None, &HintedParams::default());
    assert!(vx > 0.3, "vx={vx}");
    assert!(vy.abs() < 1e-6 && wz.abs() < 1e-6, "vy={vy} wz={wz}");
}

#[test]
fn lateral_offset_commands_crab_back() {
    // path is at y=0, robot at +0.3: crab right while still advancing
    let (vx, vy, _) = update((1.0, 0.3, 0.0), &straight(), None, &HintedParams::default());
    assert!(vy < -0.1 && vx > 0.0, "vx={vx} vy={vy}");
}

#[test]
fn behind_the_path_still_drives_onto_it() {
    // the carrot is ahead of the closest waypoint, which is the path start
    let (vx, vy, _) = update(
        (-1.0, 0.0, 0.0),
        &straight(),
        None,
        &HintedParams::default(),
    );
    assert!(vx > 0.0 && vy.abs() < 1e-12, "vx={vx} vy={vy}");
}

#[test]
fn speed_and_yaw_rate_clamped() {
    let cfg = HintedParams::default();
    let (vx, vy, _) = update((-2.0, -2.0, 0.0), &straight(), None, &cfg);
    assert!(
        vx.hypot(vy) <= cfg.base.max_speed + 1e-12,
        "speed={}",
        vx.hypot(vy)
    );
    let (_, _, wz) = update((0.0, 0.0, PI - 0.1), &straight(), None, &cfg);
    assert!(wz.abs() <= cfg.base.max_yaw_rate + 1e-12, "wz={wz}");
    // and the clamp is a saturation, not a sign flip: facing +y off a yaw-0
    // path turns clockwise
    let (_, _, wz) = update((0.0, 0.0, PI / 2.0), &straight(), None, &cfg);
    assert!(wz < -0.5, "wz={wz}");
}

#[test]
fn fan_rotates_in_place() {
    let (vx, vy, wz) = update((0.0, 0.0, 0.0), &fan(), None, &HintedParams::default());
    assert!(wz > 0.3, "wz={wz}");
    assert!(
        vx.hypot(vy) < 0.15,
        "holds position: speed={}",
        vx.hypot(vy)
    );
}

#[test]
fn fan_done_resumes_translation() {
    let (vx, vy, _) = update((0.0, 0.0, 1.45), &fan(), None, &HintedParams::default());
    assert!(vx > 0.1 || vy != 0.0, "vx={vx} vy={vy}");
}

/// Coincident fan waypoints carry no arc, so `argmin` alone would park on the
/// fan's first pose and re-command the whole rotation every tick. The yaw
/// advance walks forward while the next pose is a better yaw match, so a
/// robot already at 0.6 rad sees the REST of the fan.
#[test]
fn fan_advances_by_yaw_progress() {
    let cfg = HintedParams::default();
    let p = fan(); // yaws 0.0 .. 1.5 in 0.3 steps, all at the origin
                   // At yaw 0.6 the carrot is 0.9, one step ON: k_yaw * 0.3. Without the
                   // advance the index stays pinned at the fan's first pose and the command
                   // is k_yaw * (0.3 - 0.6) = -0.6 -- rotating back into the fan.
    let (_, _, wz_mid) = update((0.0, 0.0, 0.6), &p, None, &cfg);
    assert!(
        (wz_mid - cfg.base.k_yaw * 0.3).abs() < 1e-9,
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
    let cfg = HintedParams::default();
    let p = straight();
    // Just above `escape_clearance`: the creep band proper. (The old case sat
    // at 0.06, which the pinch-escape leg now owns -- see the V-shape test.)
    let tight = vec![0.11; p.len()];
    let (vx, vy, _) = update((0.0, 0.0, 0.0), &p, Some(&tight), &cfg);
    let speed = vx.hypot(vy);
    let ramp = cfg.base.min_speed
        + (cfg.base.max_speed - cfg.base.min_speed) * (0.11 - cfg.base.speed_floor_clearance)
            / (cfg.base.speed_clearance - cfg.base.speed_floor_clearance);
    assert!((speed - ramp).abs() < 1e-9, "speed={speed} ramp={ramp}");
    assert!(speed < cfg.base.max_speed - 0.1, "not creeping: {speed}");
}

/// The governor's lower anchor is V-shaped, not constant: low through the
/// creep band so the clearance annotation can modulate speed and the body can
/// be precise, rising again once the room has run out, where creeping buys no
/// precision (the metric is a share of TICKS under the floor) and dwell time
/// is what turns the follower's fixed cross-track offset into a contact.
#[test]
fn governor_escape_leg_is_a_v_not_a_floor() {
    let cfg = HintedParams {
        base: Params {
            min_speed: 0.45,
            max_speed: 0.95,
            ..Default::default()
        },
        ..Default::default()
    };
    let p = straight();
    let speed = |room: f64| {
        let clr = vec![room; p.len()];
        let (vx, vy, _) = update((0.0, 0.0, 0.0), &p, Some(&clr), &cfg);
        vx.hypot(vy)
    };
    // descending into the vertex, then climbing back out as the room runs out
    let (open, knee, vertex, pinch, gone) = (
        speed(0.35),
        speed(0.15),
        speed(0.082),
        speed(0.05),
        speed(0.0),
    );
    assert!(vertex < knee && knee < open, "{vertex} {knee} {open}");
    assert!(vertex < pinch && pinch < gone, "{vertex} {pinch} {gone}");
    // the leg only ever LIFTS the cap -- it can never slow anything down
    let single_ramp = HintedParams {
        escape_speed: 0.0,
        ..cfg.clone()
    };
    for room in [0.0, 0.02, 0.05, 0.082, 0.1, 0.2, 0.35] {
        let clr = vec![room; p.len()];
        let (ax, ay, _) = update((0.0, 0.0, 0.0), &p, Some(&clr), &cfg);
        let (bx, by, _) = update((0.0, 0.0, 0.0), &p, Some(&clr), &single_ramp);
        assert!(
            ax.hypot(ay) >= bx.hypot(by) - 1e-12,
            "leg slowed room={room}"
        );
    }
    // and the declared envelope still bounds it
    assert!(gone <= cfg.base.max_speed + 1e-9, "{gone}");

    // The leg is LOCAL. The ramp previews 2 m so it can slow down before a
    // pinch; the escape must not use that preview, or it would speed the body
    // up while it is still in open space short of the gap.
    let mut far = vec![0.30; p.len()];
    let last = far.len() - 1;
    far[last] = 0.0; // a pinch at the far end of the plan, not at the body
    let uniform = vec![0.30; p.len()];
    let (fx, fy, _) = update((0.0, 0.0, 0.0), &p, Some(&far), &cfg);
    let (nx, ny, _) = update((0.0, 0.0, 0.0), &p, Some(&uniform), &cfg);
    assert!(
        fx.hypot(fy) <= nx.hypot(ny) + 1e-12,
        "a distant pinch lifted the cap: {} vs {}",
        fx.hypot(fy),
        nx.hypot(ny)
    );
}

/// The 2 m preview is a BRAKING PROFILE, not a flat minimum: a pinch the body
/// has not reached yet may only impose the speed the body can still decelerate
/// TO by the time it arrives. Three invariants, all of them load-bearing.
#[test]
fn governor_preview_brakes_rather_than_creeps() {
    let cfg = HintedParams {
        base: Params {
            min_speed: 0.45,
            max_speed: 0.95,
            ..Default::default()
        },
        ..Default::default()
    };
    let p = straight();
    let speed = |cfg: &HintedParams, clr: &[f64], px: f64| {
        let (vx, vy, _) = update((px, 0.0, 0.0), &p, Some(clr), cfg);
        vx.hypot(vy)
    };

    // 1. `brake_accel = 0` is the flat min, bit-for-bit. Every shape of
    //    annotation, not just the uniform one. Compared with the escape leg
    //    off, so the assertion is against the RAMP and nothing else.
    let flat = HintedParams {
        brake_accel: 0.0,
        brake_margin: 0.0,
        ..cfg.clone()
    };
    let pure = HintedParams {
        escape_speed: 0.0,
        ..flat.clone()
    };
    let mut ramped = vec![0.40; p.len()];
    for (k, c) in ramped.iter_mut().enumerate() {
        *c = 0.02 + 0.02 * (k as f64);
    }
    for i in [0usize, 5, 10, 20] {
        let px = i as f64 * 0.1; // waypoints are 0.1 m apart, so `i` is the argmin
        let hi = (i + 20).min(ramped.len() - 1); // + speed_lookahead = 2.0 m
        let room = ramped[i..=hi].iter().copied().fold(f64::INFINITY, f64::min);
        let want = (cfg.base.min_speed
            + (cfg.base.max_speed - cfg.base.min_speed)
                * ((room - cfg.base.speed_floor_clearance)
                    / (cfg.base.speed_clearance - cfg.base.speed_floor_clearance))
                    .clamp(0.0, 1.0))
        .min(cfg.base.max_speed);
        let got = speed(&pure, &ramped, px);
        assert!((got - want).abs() < 1e-9, "px={px} got={got} want={want}");
    }

    // 2. It can only ever RAISE the cap. Monotone in brake_accel, and never
    //    below the flat min, on every annotation shape.
    for shape in [
        &ramped[..],
        &vec![0.06; p.len()][..],
        &vec![0.5; p.len()][..],
    ] {
        for px in [0.0, 0.7, 1.4] {
            let lo = speed(&flat, shape, px);
            let hi = speed(&cfg, shape, px);
            assert!(
                hi >= lo - 1e-12,
                "braking profile SLOWED px={px}: {hi} < {lo}"
            );
        }
    }

    // 3. The waypoint the body is standing on binds exactly as hard as it
    //    ever did -- inside `brake_margin` there is no braking room to claim.
    let mut here_tight = vec![0.5; p.len()];
    here_tight[..2].fill(0.05);
    assert!(
        (speed(&cfg, &here_tight, 0.0) - speed(&flat, &here_tight, 0.0)).abs() < 1e-9,
        "a pinch AT the body was relaxed by the braking profile"
    );

    // 4. ...but a pinch 1.5 m out no longer pins the body to the creep, and
    //    the relaxation is bounded by what the assumed deceleration buys.
    let mut far_tight = vec![0.5; p.len()];
    far_tight[15..17].fill(0.05);
    let far = speed(&cfg, &far_tight, 0.0);
    assert!(
        far > speed(&flat, &far_tight, 0.0) + 0.1,
        "no lift at 1.5 m: {far}"
    );
    let reach =
        (cfg.base.min_speed.powi(2) + 2.0 * cfg.brake_accel * (1.5 - cfg.brake_margin)).sqrt();
    assert!(
        far <= reach.min(cfg.base.max_speed) + 1e-9,
        "lift exceeded the profile: {far}"
    );
}

#[test]
fn governor_is_open_room_and_a_no_op_when_blind() {
    let cfg = HintedParams::default();
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
    let cfg = HintedParams::default();
    let p = straight();
    let mut clear = vec![1.0; p.len()];
    clear[..5].fill(0.06);
    let (vx, vy, _) = update((1.5, 0.0, 0.0), &p, Some(&clear), &cfg);
    assert!(
        vx.hypot(vy) > cfg.base.min_speed + 0.1,
        "speed={}",
        vx.hypot(vy)
    );
}

#[test]
fn empty_and_single_pose_paths_hold_position() {
    let cfg = HintedParams::default();
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
        (-PI, -PI), // 3pi/tau is exactly 1.5, and half rounds to the EVEN 2 -- so the
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

/// Determinism (contract) via statelessness (NOT contract, just how the seed
/// happens to achieve it): the same tick twice is bit-identical, and no call
/// leaves a trace in the next one.
///
/// A law with memory cannot satisfy the second half and does not need to --
/// what it owes is that the same call SEQUENCE from a reset state reproduces,
/// which the `protocol` gate checks end-to-end through the python seam.
/// Replace this test when the law grows state; do not drop the determinism
/// half.
#[test]
fn stateless_and_deterministic() {
    let cfg = HintedParams::default();
    let p = straight();
    let a = update((0.3, -0.2, 0.4), &p, None, &cfg);
    update((9.0, 9.0, 3.0), &fan(), None, &cfg);
    let b = update((0.3, -0.2, 0.4), &p, None, &cfg);
    assert_eq!(a.0.to_bits(), b.0.to_bits());
    assert_eq!(a.1.to_bits(), b.1.to_bits());
    assert_eq!(a.2.to_bits(), b.2.to_bits());
}

/// A quarter circle of radius 1 m starting at the origin heading +x: the
/// plan's heading turns at exactly 1 rad per metre of arc.
fn arc_path() -> Vec<[f64; 3]> {
    (0..40)
        .map(|k| {
            let th = k as f64 * 0.05; // arc = th (radius 1)
            [th.sin(), 1.0 - th.cos(), th]
        })
        .collect()
}

/// The margin contract this law exists for: on a curving plan the heading it
/// commands is the plan's heading HERE, not the one a lookahead away. A
/// 0.85 m body pre-rotated into the next segment sweeps its corners across
/// the corridor it is still inside, which is how the seed spent 3-14 cm of
/// clearance it did not have.
#[test]
fn yaw_targets_the_heading_here_not_the_carrot_ahead() {
    let cfg = HintedParams::default();
    let p = arc_path();
    // sitting exactly on the plan at arc 0.5 (heading 0.5 rad), moving
    let (vx, vy, wz) = update((p[10][0], p[10][1], p[10][2]), &p, None, &cfg);
    let speed = vx.hypot(vy);
    assert!(speed > 0.1, "expected to be moving, speed={speed}");
    // the proportional term is zero (we are AT the plan's heading for here);
    // what is left is pure feedforward: 1 rad/m of plan turn times the speed.
    assert!(
        (wz - speed).abs() < 0.05,
        "on a 1 rad/m arc the yaw rate should be ~speed rad/s, got wz={wz} speed={speed}"
    );
    // the seed took the carrot's yaw and would have commanded k_yaw * 0.35
    // (~0.7 rad/s) here, as a standing lead rather than a rate.
    assert!(
        wz < 0.6,
        "yaw command looks like the old constant-arc lead: wz={wz}"
    );
}

/// Feedforward is a rate, so it scales with how fast the body is actually
/// commanded to go: creeping through a tight gap must not carry the same yaw
/// lead as cruising, which is exactly when the seed's lead went uncancelled.
#[test]
fn yaw_feedforward_scales_with_commanded_speed() {
    let cfg = HintedParams::default();
    let p = arc_path();
    let pose = (p[10][0], p[10][1], p[10][2]);
    let fast = update(pose, &p, None, &cfg).2;
    let tight = vec![0.05f64; p.len()]; // governor floors at min_speed
    let slow = update(pose, &p, Some(&tight), &cfg).2;
    assert!(
        slow < fast * 0.75,
        "yaw rate should fall with the governed speed: fast={fast} slow={slow}"
    );
}

/// A turn tighter than the fan classifier's own threshold does not report a
/// curvature -- `yaw_ff` reports `fan_yaw_per_m` itself. At the cruise ceiling
/// that is 3.0 * 0.95 = 2.85 rad/s against a `max_yaw_rate` of 1.4, which pins
/// the sum clamp, flattens d(wz)/d(heading error) to zero and opens the
/// heading loop. In OPEN room that feedforward is switched off.
///
/// Built as a smooth arc, so `in_fan` is false and the case really does reach
/// the curvature branch.
fn tight_arc(r: f64, step: f64) -> Vec<[f64; 3]> {
    (0..140)
        .map(|k| {
            let a = k as f64 * step / r;
            [r * a.sin(), r * (1.0 - a.cos()), a]
        })
        .collect()
}

#[test]
fn classifier_saturated_feedforward_is_silent_at_full_cruise() {
    let cfg = HintedParams {
        base: Params {
            min_speed: 0.45,
            max_speed: 0.95,
            ..Default::default()
        },
        ..Default::default()
    };
    let p = tight_arc(0.25, 0.015); // 4.0 rad/m, past fan_yaw_per_m = 3.0
    let on = p[40];
    let roomy = vec![1.0f64; p.len()]; // governor grants the full ceiling
    let (vx, vy, wz) = update((on[0], on[1], on[2]), &p, Some(&roomy), &cfg);
    // the body is ON the plan at the plan's own heading here, so the
    // proportional term is ~0 and anything left is feedforward
    assert!(
        wz.abs() < 0.1,
        "a classifier-saturated rate was fed forward at full cruise: wz={wz}"
    );
    // and it did NOT buy that by slowing down
    let speed = vx.hypot(vy);
    assert!(
        (speed - cfg.base.max_speed).abs() < 1e-9,
        "the yaw fix cost translation speed: speed={speed}"
    );
}

/// ...and the same geometry inside a pinch keeps its feedforward, because
/// there it is the term that cancels the yaw P-loop's standing error and that
/// standing error is what eats the margin. Measured: removing it here flips
/// two pinch worlds to collisions with below_floor 0.054 -> 0.725 and
/// 0.296 -> 0.516.
#[test]
fn classifier_saturated_feedforward_survives_where_the_governor_limits() {
    let cfg = HintedParams {
        base: Params {
            min_speed: 0.45,
            max_speed: 0.95,
            ..Default::default()
        },
        ..Default::default()
    };
    let p = tight_arc(0.25, 0.015);
    let on = p[40];
    let tight = vec![0.11f64; p.len()]; // the creep band: governor is limiting
    let (vx, vy, wz) = update((on[0], on[1], on[2]), &p, Some(&tight), &cfg);
    let speed = vx.hypot(vy);
    assert!(
        speed < cfg.base.max_speed - 1e-9,
        "governor was not limiting: {speed}"
    );
    assert!(
        (wz.abs() - (3.0 * speed).min(cfg.base.max_yaw_rate)).abs() < 1e-9,
        "the pinch lost its heading feedforward: wz={wz} speed={speed}"
    );
}

/// The gate keys on the CLASSIFIER, not on saturation: a turn the classifier
/// still calls a curve is fed forward exactly as before, even in open room and
/// even when it saturates the clamp. The change removes the case where the
/// demand is an artefact of the threshold, not yaw saturation in general --
/// which is why a governor floor above `max_yaw_rate / fan_yaw_per_m` = 0.467
/// would still re-admit it.
#[test]
fn a_curve_under_the_classifier_is_untouched_in_open_room() {
    let cfg = HintedParams {
        base: Params {
            min_speed: 0.45,
            max_speed: 0.95,
            ..Default::default()
        },
        ..Default::default()
    };
    let roomy = |p: &Vec<[f64; 3]>| vec![1.0f64; p.len()];

    // 1.0 rad/m: deliverable, so the command IS the feedforward
    let p = tight_arc(1.0, 0.03);
    let on = p[40];
    let (vx, vy, wz) = update((on[0], on[1], on[2]), &p, Some(&roomy(&p)), &cfg);
    let speed = vx.hypot(vy);
    assert!(
        (wz - speed).abs() < 0.15,
        "a deliverable curve lost its feedforward: wz={wz} speed={speed}"
    );

    // 2.5 rad/m: still under the classifier, still fed forward, and it still
    // saturates the clamp at the ceiling exactly as it always did
    let p = tight_arc(0.40, 0.02);
    let on = p[40];
    let (_, _, wz) = update((on[0], on[1], on[2]), &p, Some(&roomy(&p)), &cfg);
    assert!(
        (wz.abs() - cfg.base.max_yaw_rate).abs() < 1e-9,
        "expected a genuine 2.5 rad/m curve to still saturate: wz={wz}"
    );
}

// ---------------------------------------------------------------------------
// Tangent feedforward: the chord is gone
// ---------------------------------------------------------------------------

/// A right-angle corner, 8 cm waypoint spacing: the case the seed cut.
fn right_angle() -> Vec<[f64; 3]> {
    let mut p: Vec<[f64; 3]> = (0..20).map(|k| [k as f64 * 0.08, 0.0, 0.0]).collect();
    p.extend((1..20).map(|k| [1.52, k as f64 * 0.08, std::f64::consts::FRAC_PI_2]));
    p
}

/// On the line and 22 cm short of a right-angle corner, the command must be
/// essentially straight down the leg the robot is on. The seed's carrot at
/// `lookahead = 0.35` sat 13 cm PAST the corner, so it commanded a lateral
/// component of 0.13/0.22 -- 59% of forward speed, aimed diagonally across
/// the inside of the turn, 22 cm before there was any turn to make.
#[test]
fn does_not_cut_the_corner() {
    let cfg = HintedParams::default();
    let (vx, vy, _) = update((1.30, 0.0, 0.0), &right_angle(), None, &cfg);
    assert!(vx > 0.4, "lost speed on approach: vx={vx}");
    assert!(
        vy.abs() < 0.1 * vx,
        "still cutting toward the inside of the corner: vx={vx} vy={vy}"
    );
}

/// On the line on a straight, the command is the governor ceiling exactly --
/// the feedforward costs nothing in speed, which is the whole point of
/// separating it from the correction.
#[test]
fn on_line_commands_the_full_ceiling() {
    let cfg = HintedParams::default();
    let (vx, vy, _) = update((1.0, 0.0, 0.0), &straight(), None, &cfg);
    assert!(
        (vx - cfg.base.max_speed).abs() < 1e-9 && vy.abs() < 1e-12,
        "vx={vx} vy={vy}"
    );
}

/// The claim the whole experiment rests on, in closed loop: with the velocity
/// command executed perfectly at 50 Hz, steady-state cross-track on a curve
/// decays to ~zero rather than settling at the chord's sagitta. Radius 0.6 m
/// is a normal planner corner; the seed's chord error there is L^2/(8R) =
/// 2.6 cm, and it does not decay because the aim point is the error.
#[test]
fn curve_tracking_error_decays_to_zero() {
    let cfg = HintedParams::default();
    let r = 0.6;
    // three quarters of a circle, 4 cm waypoint spacing
    let n = 70;
    let path: Vec<[f64; 3]> = (0..n)
        .map(|k| {
            let a = k as f64 * 0.04 / r;
            [r * a.sin(), r * (1.0 - a.cos()), a]
        })
        .collect();

    let (dt, mut p) = (0.02, (0.0, 0.0, 0.0));
    let mut worst_late = 0.0f64;
    for step in 0..200 {
        let (vx, vy, wz) = update(p, &path, None, &cfg);
        // perfect execution: body twist straight into world motion
        let (c, s) = (p.2.cos(), p.2.sin());
        p = (
            p.0 + (c * vx - s * vy) * dt,
            p.1 + (s * vx + c * vy) * dt,
            p.2 + wz * dt,
        );
        if step > 50 {
            // settled: measure distance to the polyline the way the judge does
            let mut d = f64::INFINITY;
            for m in 0..path.len() - 1 {
                let (ax, ay) = (path[m][0], path[m][1]);
                let (ex, ey) = (path[m + 1][0] - ax, path[m + 1][1] - ay);
                let u = (((p.0 - ax) * ex + (p.1 - ay) * ey) / (ex * ex + ey * ey)).clamp(0.0, 1.0);
                d = d.min((p.0 - ax - u * ex).hypot(p.1 - ay - u * ey));
            }
            worst_late = worst_late.max(d);
        }
    }
    assert!(
        worst_late < 0.01,
        "settled cross-track {worst_late:.4} m -- the feedforward is not holding the line"
    );
}

// ---------------------------------------------------------------------------
// The command ramp. `Law` is the stateful wrapper around `update`: it exists
// because the plant applies its own per-axis rate limit to whatever the law
// asks for, so a law that steps faster than the ramp is not commanding the
// robot, it is commanding the ramp.
// ---------------------------------------------------------------------------

#[test]
fn ramp_respects_the_plant_slew_on_every_axis() {
    let cfg = HintedParams::default();
    let mut law = Law::new();
    // straight path, body parked well off it and mis-aimed: the raw law wants
    // a large step on all three axes at once, from a standstill
    let mut prev = (0.0, 0.0, 0.0);
    let (mut t, dt) = (0.5, 0.02);
    for step in 0..40 {
        let out = law.step((-1.0, 1.0, PI - 0.2), &straight(), None, &cfg, t);
        assert!(
            (out.0 - prev.0).abs() <= cfg.slew[0] * dt + 1e-12
                && (out.1 - prev.1).abs() <= cfg.slew[1] * dt + 1e-12
                && (out.2 - prev.2).abs() <= cfg.slew[2] * dt + 1e-12,
            "step {step}: {prev:?} -> {out:?} exceeds the plant slew"
        );
        assert!(
            out.0.hypot(out.1) <= cfg.base.max_speed + 1e-12,
            "step {step}: speed {} outside the envelope",
            out.0.hypot(out.1)
        );
        prev = out;
        t += dt;
    }
    // and it does get where it was going: 40 ticks is past the ramp
    assert!(prev.0.hypot(prev.1) > 0.4, "never reached cruise: {prev:?}");
}

#[test]
fn veto_stub_is_not_ramped_even_at_full_speed() {
    // A path under two poses is the planner's stop veto. Obeying it is not
    // optional and it is not something the ramp may soften.
    let cfg = HintedParams::default();
    let mut law = Law::new();
    let (mut t, dt) = (0.5, 0.02);
    for _ in 0..60 {
        law.step((0.0, 0.0, 0.0), &straight(), None, &cfg, t);
        t += dt;
    }
    let moving = law.step((0.0, 0.0, 0.0), &straight(), None, &cfg, t);
    assert!(
        moving.0 > 0.3,
        "not actually moving before the veto: {moving:?}"
    );
    t += dt;
    assert_eq!(
        law.step((0.0, 0.0, 0.0), &[[0.0, 0.0, 0.0]], None, &cfg, t),
        (0.0, 0.0, 0.0)
    );
    assert_eq!(
        law.step((0.0, 0.0, 0.0), &[], None, &cfg, t + dt),
        (0.0, 0.0, 0.0)
    );
}

#[test]
fn reset_clears_the_ramp() {
    let cfg = HintedParams::default();
    let (t, dt) = (0.5, 0.02);
    let mut a = Law::new();
    let mut first = Vec::new();
    for k in 0..30 {
        first.push(a.step((-1.0, 1.0, 0.3), &straight(), None, &cfg, t + k as f64 * dt));
    }
    // a foreign history, then reset: the replay must match a fresh instance
    for k in 0..30 {
        a.step(
            (2.0, -2.0, -1.0),
            &fan(),
            None,
            &cfg,
            t + (60 + k) as f64 * dt,
        );
    }
    a.reset();
    let mut fresh = Law::new();
    for (k, want) in first.iter().enumerate() {
        let tk = t + k as f64 * dt;
        let after_reset = a.step((-1.0, 1.0, 0.3), &straight(), None, &cfg, tk);
        assert_eq!(
            after_reset,
            fresh.step((-1.0, 1.0, 0.3), &straight(), None, &cfg, tk),
            "tick {k}: reset instance and fresh instance disagree"
        );
        assert_eq!(after_reset, *want, "tick {k}: replay after reset drifted");
    }
}

#[test]
fn a_backwards_or_stalled_clock_cannot_bank_a_larger_step() {
    let cfg = HintedParams::default();
    let mut law = Law::new();
    // same t every call, then a t far in the past, then a huge jump forward:
    // none of them may licence a step past the nominal tick's worth of slew
    let mut prev = (0.0, 0.0, 0.0);
    for t in [0.5, 0.5, 0.5, 0.1, -3.0, 900.0] {
        let out = law.step((-1.0, 1.0, PI - 0.2), &straight(), None, &cfg, t);
        let cap = cfg.slew[1] * MAX_TICK_FOR_TEST + 1e-12;
        assert!((out.1 - prev.1).abs() <= cap, "t={t}: {prev:?} -> {out:?}");
        prev = out;
    }
}

/// Mirror of `pursuit`'s private `MAX_TICK` -- the longest gap the ramp will
/// integrate over. Duplicated rather than exported: it is an implementation
/// bound, not part of the law's interface.
const MAX_TICK_FOR_TEST: f64 = 0.10;
