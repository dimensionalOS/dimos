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

//! The blind law's behavioural cases, from `motion-tc-autoresearch` branch
//! `blind_research01`.
//!
//! The seed's portrait carries over (a law that stopped driving forward or
//! stopped obeying a veto stub would be broken whatever its governor reads),
//! with ONE restatement: the envelope is a bound on the ground speed the law
//! INTENDS, not on the number in the twist. The twist is a request to a gait
//! that under-delivers it by a measured margin (see `WALK_SLIP`), so
//! `walk_command(max_speed)` -- not `max_speed` -- is the largest linear
//! command this law may emit. The bound is still a bound; what changed is
//! which quantity it is expressed in.
//!
//! The stamp cases at the bottom are written against the REFEREE's encoder
//! rather than against this law's decoder, so they fail if the wire dialect is
//! ever misread -- the failure mode that matters, because a misread stamp is
//! silently a wrong speed rather than a crash.

use std::f64::consts::PI;

use dimos_motion2_tc::geom::{ieee_remainder, TAU};
use dimos_motion2_tc::laws::blind::{
    update, walk_command, BlindParams, SLIP_RAMP, WALK_GAIN, WALK_SLIP,
};
use dimos_motion2_tc::stamps::decode_ceilings;

fn cfg() -> BlindParams {
    BlindParams::default()
}

/// The ground speed a command is worth, inverting `walk_command` above the ramp.
fn ground(cmd: f64) -> f64 {
    (cmd - WALK_SLIP) / WALK_GAIN
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
    let (vx, vy, wz) = update((0.0, 0.0, 0.0), &straight(), None, None, &cfg());
    assert!(vx > 0.3, "vx={vx}");
    assert!(vy.abs() < 1e-6 && wz.abs() < 1e-6, "vy={vy} wz={wz}");
}

#[test]
fn lateral_offset_commands_crab_back() {
    // path is at y=0, robot at +0.3: crab right while still advancing
    let (vx, vy, _) = update((1.0, 0.3, 0.0), &straight(), None, None, &cfg());
    assert!(vy < -0.1 && vx > 0.0, "vx={vx} vy={vy}");
}

#[test]
fn behind_the_path_still_drives_onto_it() {
    let (vx, vy, _) = update((-1.0, 0.0, 0.0), &straight(), None, None, &cfg());
    assert!(vx > 0.0 && vy.abs() < 1e-12, "vx={vx} vy={vy}");
}

/// RESTATED from the seed: the envelope bounds the INTENDED ground speed.
/// Asserting both ways round is the point -- the envelope did not widen, it
/// changed units.
#[test]
fn speed_and_yaw_rate_clamped() {
    let c = cfg();
    let (vx, vy, _) = update((-2.0, -2.0, 0.0), &straight(), None, None, &c);
    let ceiling = walk_command(c.base.max_speed, c.walk_gain, c.walk_slip, c.slip_ramp);
    assert!(vx.hypot(vy) <= ceiling + 1e-12, "speed={}", vx.hypot(vy));
    assert!(
        ceiling > c.base.max_speed,
        "a gait that under-delivers must be over-asked: ceiling={ceiling}"
    );
    assert!(
        (ground(ceiling) - c.base.max_speed).abs() < 1e-9,
        "the ceiling does not buy max_speed of ground: ceiling={ceiling}"
    );
    let (_, _, wz) = update((0.0, 0.0, PI - 0.1), &straight(), None, None, &c);
    assert!(wz.abs() <= c.base.max_yaw_rate + 1e-12, "wz={wz}");
    // and the clamp is a saturation, not a sign flip
    let (_, _, wz) = update((0.0, 0.0, PI / 2.0), &straight(), None, None, &c);
    assert!(wz < -0.5, "wz={wz}");
}

/// A stop request has to remain a stop: the correction fades to zero with the
/// intended speed rather than stepping off a cliff at the ramp.
#[test]
fn walk_command_is_identity_at_rest_and_continuous() {
    let (g, s, r) = (WALK_GAIN, WALK_SLIP, SLIP_RAMP);
    assert_eq!(walk_command(0.0, g, s, r), 0.0);
    assert_eq!(walk_command(-1.0, g, s, r), 0.0);
    assert!(walk_command(1e-9, g, s, r) < 1e-6, "a crawl became a lunge");
    // monotone: asking for more ground never asks the gait for less
    let mut prev = 0.0;
    for k in 0..=100 {
        let cmd = walk_command(k as f64 * 0.01, g, s, r);
        assert!(
            cmd >= prev - 1e-12,
            "not monotone at want={}",
            k as f64 * 0.01
        );
        prev = cmd;
    }
}

#[test]
fn fan_rotates_in_place() {
    let (vx, vy, wz) = update((0.0, 0.0, 0.0), &fan(), None, None, &cfg());
    assert!(wz > 0.3, "wz={wz}");
    assert!(
        vx.hypot(vy) < 0.15,
        "holds position: speed={}",
        vx.hypot(vy)
    );
}

#[test]
fn fan_done_resumes_translation() {
    let (vx, vy, _) = update((0.0, 0.0, 1.45), &fan(), None, None, &cfg());
    assert!(vx > 0.1 || vy != 0.0, "vx={vx} vy={vy}");
}

#[test]
fn fan_advances_by_yaw_progress() {
    let c = cfg();
    let p = fan();
    let (_, _, wz_mid) = update((0.0, 0.0, 0.6), &p, None, None, &c);
    assert!(
        (wz_mid - c.base.k_yaw * 0.3).abs() < 1e-9,
        "mid-fan target is not the next pose: wz={wz_mid}"
    );
    let (vx, vy, _) = update((0.0, 0.0, 1.5), &p, None, None, &c);
    assert!(
        vx.hypot(vy) > 0.05,
        "still stuck rotating: speed={}",
        vx.hypot(vy)
    );
}

/// The creep is asserted on the GROUND speed the command buys, not on the
/// command. Asserting it on the command is how the seed came to ask for a
/// speed the gait answers with 0.002 m/s -- see `WALK_SLIP`. A creep that does
/// not move the robot is not a creep, it is a stall, and it costs the whole
/// episode clock for none of the caution it looks like.
#[test]
fn governor_creeps_in_tight_room() {
    let c = cfg();
    let p = straight();
    let tight = vec![0.06; p.len()]; // barely above the precision floor
    let (vx, vy, _) = update((0.0, 0.0, 0.0), &p, Some(&tight), None, &c);
    let g = ground(vx.hypot(vy));
    assert!(g <= c.base.min_speed + 0.02, "ground={g}");
    assert!(
        g > 0.5 * c.base.min_speed,
        "governed down to a stall, not a creep: ground={g}"
    );
}

#[test]
fn governor_is_open_room_and_a_no_op_when_blind() {
    let c = cfg();
    let p = straight();
    let wide = vec![1.0; p.len()];
    let open = update((-1.0, 0.0, 0.0), &p, Some(&wide), None, &c);
    let blind = update((-1.0, 0.0, 0.0), &p, None, None, &c);
    assert_eq!(open, blind, "wide clearance changed the command");
    // a wrong-length annotation is ignored, exactly as in the python
    let stub = vec![0.06; p.len() - 1];
    assert_eq!(update((-1.0, 0.0, 0.0), &p, Some(&stub), None, &c), blind);
}

#[test]
fn governor_reads_room_ahead_not_behind() {
    let c = cfg();
    let p = straight();
    let mut clear = vec![1.0; p.len()];
    clear[..5].fill(0.06);
    let (vx, vy, _) = update((1.5, 0.0, 0.0), &p, Some(&clear), None, &c);
    assert!(
        ground(vx.hypot(vy)) > c.base.min_speed + 0.1,
        "speed={}",
        vx.hypot(vy)
    );
}

#[test]
fn empty_and_single_pose_paths_hold_position() {
    let c = cfg();
    assert_eq!(
        update((1.0, 2.0, 0.3), &[], None, None, &c),
        (0.0, 0.0, 0.0)
    );
    let stub = [[5.0, 5.0, 1.0]];
    assert_eq!(
        update((1.0, 2.0, 0.3), &stub, None, None, &c),
        (0.0, 0.0, 0.0)
    );
}

#[test]
fn stateless_and_deterministic() {
    let c = cfg();
    let p = straight();
    let a = update((0.3, -0.2, 0.4), &p, None, None, &c);
    update((9.0, 9.0, 3.0), &fan(), None, None, &c);
    let b = update((0.3, -0.2, 0.4), &p, None, None, &c);
    assert_eq!(a.0.to_bits(), b.0.to_bits());
    assert_eq!(a.1.to_bits(), b.1.to_bits());
    assert_eq!(a.2.to_bits(), b.2.to_bits());
}

// ---------------------------------------------------------------------------
// Constant time headway.
// ---------------------------------------------------------------------------

/// The carrot shortens with the governed speed. Behaviourally: on a curving
/// plan, a governed-down follower cuts less of the corner -- and the corner it
/// would cut is the side the planner curved away from.
#[test]
fn headway_shortens_the_carrot_when_governed_down() {
    let c = cfg();
    // a left-turning arc, 0.1 m discretisation like the planner's
    let p: Vec<[f64; 3]> = (0..40)
        .map(|k| {
            let th = k as f64 * 0.1 / 2.0; // radius 2 m
            [2.0 * th.sin(), 2.0 * (1.0 - th.cos()), th]
        })
        .collect();
    let wide = vec![1.0; p.len()];
    let tight = vec![0.06; p.len()];
    let pose = (p[5][0], p[5][1], p[5][2]);
    let (fx, fy, _) = update(pose, &p, Some(&wide), None, &c);
    let (sx, sy, _) = update(pose, &p, Some(&tight), None, &c);
    // Heading of the command in the body frame. On a left turn the chord to a
    // far carrot leans further LEFT than the path tangent -- that lean IS the
    // corner cut, and it is toward the inside of the turn. A nearer carrot
    // leans less, which is the whole mechanism.
    let fast_off = fy.atan2(fx);
    let slow_off = sy.atan2(sx);
    assert!(
        fast_off > 0.0,
        "the arc does not curve left: fast={fast_off}"
    );
    assert!(
        slow_off < fast_off,
        "the governed carrot did not shorten: fast={fast_off} slow={slow_off}"
    );
}

/// It must not cost speed: shortening the carrot cannot turn into a slower
/// robot, or the fix trades collisions for timeouts.
#[test]
fn headway_never_reduces_the_commanded_speed() {
    let c = cfg();
    let p = straight(); // 0 .. 3.9 m
                        // stop well before the end: near the terminus the carrot IS the endpoint
                        // and the law decelerates into it, which is arrival, not headway.
    for step in 0..20 {
        let pose = (step as f64 * 0.13, 0.0, 0.0);
        let (vx, vy, _) = update(pose, &p, None, None, &c);
        let want = ground(vx.hypot(vy));
        assert!(
            want >= c.base.max_speed - 1e-9,
            "step {step}: open-room speed dropped to {want}"
        );
    }
}

// ---------------------------------------------------------------------------
// The stamped precision profile (the blind-track governor channel).
// ---------------------------------------------------------------------------

/// `profile.governor_speed`, transcribed: clearance (m) -> speed ceiling (m/s).
fn governor_speed(clearance: f64) -> f64 {
    let frac = ((clearance - 0.05) / (0.35 - 0.05)).clamp(0.0, 1.0);
    0.2 + (0.5 - 0.2) * frac
}

/// `profile.encode_precision`, transcribed: the planner's side of the wire.
/// dt across a segment is its length over the governor speed of its TIGHTER
/// endpoint; a fan (zero-length) segment is priced by yaw span instead.
fn encode_precision(path: &[[f64; 3]], clearance: &[f64], t0: f64) -> Vec<f64> {
    let mut ts = vec![t0; path.len()];
    let mut t = t0;
    for i in 1..path.len() {
        let (dx, dy) = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
        let ds = (dx * dx + dy * dy).sqrt();
        if ds < 1e-6 {
            t += ieee_remainder(path[i][2] - path[i - 1][2], TAU).abs() / 1.4;
        } else {
            t += ds / governor_speed(clearance[i - 1]).min(governor_speed(clearance[i]));
        }
        ts[i] = t;
    }
    ts
}

/// A clearance profile that pinches in the middle of the straight path.
fn pinched(n: usize) -> Vec<f64> {
    (0..n)
        .map(|k| {
            let d = (k as f64 - 20.0).abs() / 20.0;
            0.04 + 0.5 * d * d
        })
        .collect()
}

#[test]
fn stamps_decode_back_to_the_governor_curve() {
    let c = cfg();
    let p = straight();
    let clr = pinched(p.len());
    let ts = encode_precision(&p, &clr, 17.5); // a non-zero t0 must not matter
    let got = decode_ceilings(&ts, &p, &c.base).expect("stamped path decodes");
    for k in 1..p.len() {
        let want = governor_speed(clr[k - 1]).min(governor_speed(clr[k]));
        assert!(
            (got[k] - want).abs() < 1e-9,
            "waypoint {k}: decoded {} want {want}",
            got[k]
        );
    }
    assert_eq!(got[0], got[1]);
}

/// The point of the whole exercise: the stamps are a lossless carrier for the
/// clearance annotation, so the law drives the same way with either channel.
#[test]
fn stamped_path_governs_exactly_like_the_clearance_array() {
    let c = cfg();
    let p = straight();
    let clr = pinched(p.len());
    let ts = encode_precision(&p, &clr, 0.0);
    for step in 0..30 {
        let pose = (step as f64 * 0.12, 0.01, 0.0);
        let hinted = update(pose, &p, Some(&clr), None, &c);
        let stamped = update(pose, &p, None, Some(&ts), &c);
        assert!(
            (hinted.0 - stamped.0).abs() < 1e-9
                && (hinted.1 - stamped.1).abs() < 1e-9
                && (hinted.2 - stamped.2).abs() < 1e-9,
            "step {step}: hinted {hinted:?} stamped {stamped:?}"
        );
    }
}

/// An explicit clearance array still wins: the stamps are the fallback
/// channel, not a second governor stacked on top of the first.
#[test]
fn clearance_array_takes_precedence_over_the_stamps() {
    let c = cfg();
    let p = straight();
    let wide = vec![1.0; p.len()];
    let tight_ts = encode_precision(&p, &vec![0.04; p.len()], 0.0);
    let with_both = update((0.0, 0.0, 0.0), &p, Some(&wide), Some(&tight_ts), &c);
    let with_clr = update((0.0, 0.0, 0.0), &p, Some(&wide), None, &c);
    assert_eq!(with_both, with_clr);
}

/// A producer that does not speak the dialect must not be able to slow the
/// robot down by accident -- nor speed it up.
#[test]
fn unstamped_and_nonsense_stamps_fall_back_to_cruise() {
    let c = cfg();
    let p = straight();
    let blind = update((0.0, 0.0, 0.0), &p, None, None, &c);
    let n = p.len();

    // flat stamps: no dt anywhere
    assert_eq!(
        update((0.0, 0.0, 0.0), &p, None, Some(&vec![3.0; n]), &c),
        blind
    );
    // non-monotone: the timeline goes backwards
    let mut backwards: Vec<f64> = (0..n).map(|k| k as f64 * 0.2).collect();
    backwards[7] = 0.0;
    assert_eq!(
        update((0.0, 0.0, 0.0), &p, None, Some(&backwards), &c),
        blind
    );
    // wrong length: ignored like a wrong-length clearance annotation
    assert_eq!(
        update((0.0, 0.0, 0.0), &p, None, Some(&vec![0.1; n - 1]), &c),
        blind
    );
    // default-constructed poses, stamped microseconds apart: decodes to an
    // absurd speed, clips to cruise, changes nothing
    let jittery: Vec<f64> = (0..n).map(|k| 1.7e9 + k as f64 * 1e-6).collect();
    assert_eq!(update((0.0, 0.0, 0.0), &p, None, Some(&jittery), &c), blind);
}

/// THE constraint on this channel. A decoded ceiling is an intended GROUND
/// speed, and the gait does not initiate below ~0.30 m/s COMMANDED. Every
/// governed speed therefore has to leave through `walk_command`, or the
/// governor reinstates the stall band it is supposed to be steering around.
#[test]
fn the_stamped_creep_is_a_speed_the_gait_can_realize() {
    let c = cfg();
    let p = straight();
    // the tightest thing the encoder can express: at or under the floor, so
    // every segment is stamped at min_speed
    let ts = encode_precision(&p, &vec![0.02; p.len()], 0.0);
    let (vx, vy, _) = update((0.0, 0.0, 0.0), &p, None, Some(&ts), &c);
    let cmd = vx.hypot(vy);
    assert!(
        cmd >= 0.30,
        "commanded {cmd} m/s is inside the gait dead band -- the robot marches in place"
    );
    // ...and it is still a creep, not cruise: the intended ground speed is the
    // governor floor, not max_speed.
    assert!(
        ground(cmd) <= c.base.min_speed + 0.02,
        "ground={}",
        ground(cmd)
    );
}

/// Fan segments carry yaw, not clearance, so their dt must not be read as a
/// speed: the decoder inherits across them instead.
#[test]
fn fan_segments_inherit_rather_than_decode() {
    let c = cfg();
    let p = fan();
    let clr = vec![1.0; p.len()];
    let ts = encode_precision(&p, &clr, 0.0);
    let got = decode_ceilings(&ts, &p, &c.base).expect("fan path decodes");
    assert!(
        got.iter().all(|&v| (v - c.base.max_speed).abs() < 1e-9),
        "a yaw-priced fan leaked into the speed ceiling: {got:?}"
    );
}
