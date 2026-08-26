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

//! The precision profile the planner stamps into the path's own timestamps.
//!
//! Shared facility, not a law: this is the wire dialect, and the python side
//! of it is `control/profile.py` (`encode_precision` / `decode_ceilings`),
//! which is the specification. Any law may read it; the blind track is the
//! one that has to, because it is the only channel through which required
//! precision reaches a follower that gets no clearance array.

use crate::geom::Params;

/// A segment shorter than this is a rotation in place, not a move -- the
/// python `profile._FAN_EPS`, and it has to match: the encoder prices fan
/// segments by yaw span instead of by clearance, so their dt carries no
/// precision and the decoder must skip them rather than read a speed out of
/// them.
pub const FAN_EPS: f64 = 1e-6;

/// The governor curve the ENCODER speaks, mirroring `profile.py`'s module
/// constants.
///
/// Deliberately not `Params`. Decoding takes the consumer's own band, so a
/// controller recovers the ceiling its own governor would have produced; but
/// encoding is the producer's side of a WIRE contract, and if it moved with
/// whatever config the planner process happened to hold, two robots with
/// different controller tuning would stamp the same path differently. The
/// python keeps these as module constants for the same reason, and notes that
/// they are held in step with `ControllerConfig` by hand.
pub const MAX_SPEED: f64 = 0.5;
pub const MIN_SPEED: f64 = 0.2;
pub const SPEED_CLEARANCE: f64 = 0.35;
pub const FLOOR_CLEARANCE: f64 = 0.05;
/// Prices fan segments, whose dt is a yaw span rather than a distance.
pub const MAX_YAW_RATE: f64 = 1.4;

/// Clearance (m) -> speed ceiling (m/s): creep at the floor, cruise with room.
///
/// A port of `profile.governor_speed`. Infinite clearance is expected and
/// meaningful -- no obstacles means nothing can touch the body, so the
/// fraction saturates and the waypoint gets cruise.
// `max` then `min` rather than `clamp`, which is what clippy wants here:
// f64::clamp PANICS when handed a NaN. No caller can produce one (clearance is
// a distance), but a panic in the planner tick is a far worse failure than the
// creep this degrades to, and the encoder runs on whatever the map hands it.
#[allow(clippy::manual_clamp)]
pub fn governor_speed(clearance: f64) -> f64 {
    let frac = (clearance - FLOOR_CLEARANCE) / (SPEED_CLEARANCE - FLOOR_CLEARANCE);
    MIN_SPEED + (MAX_SPEED - MIN_SPEED) * frac.max(0.0).min(1.0)
}

/// Stamp a path with its precision profile: the timestamps, in order.
///
/// A port of `profile.encode_precision`, which states the dialect:
/// `ts[i] - ts[i-1] = segment length / governor speed`, the governor evaluated
/// at the TIGHTER of the segment's two endpoints. `decode_ceilings` above is
/// the exact inverse.
///
/// Returns the stamps rather than mutating a path, because this crate has no
/// message types -- the adapter writes them onto the poses. `clearance` of the
/// wrong length is ignored and every segment gets cruise, matching the python's
/// `if len(clearance) == n` guard: a planner that could not compute room still
/// produces a well-formed path, it just carries no precision hint.
///
/// Fan segments (yaw with no displacement) are priced by yaw span at
/// `MAX_YAW_RATE` instead, which is why the decoder has to skip them -- their
/// dt is not a distance over a speed and reading one as such would invent a
/// ceiling from a rotation.
pub fn encode_precision(path: &[[f64; 3]], clearance: &[f64], t0: f64) -> Vec<f64> {
    let n = path.len();
    if n == 0 {
        return Vec::new();
    }
    let use_clearance = clearance.len() == n;
    let speed = |k: usize| {
        if use_clearance {
            governor_speed(clearance[k])
        } else {
            MAX_SPEED
        }
    };
    let mut ts = vec![t0; n];
    let mut t = t0;
    for k in 1..n {
        let (dx, dy) = (path[k][0] - path[k - 1][0], path[k][1] - path[k - 1][1]);
        let ds = (dx * dx + dy * dy).sqrt();
        if ds < FAN_EPS {
            // `np.remainder` (floor-mod), NOT the IEEE remainder the laws use
            // for angle_diff -- this mirrors the python statement exactly. The
            // two disagree only at +/-pi, and the abs() below makes even that
            // agree, but the form is kept so the port reads against its spec.
            let dyaw = path[k][2] - path[k - 1][2];
            let wrapped = (dyaw + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI;
            t += wrapped.abs() / MAX_YAW_RATE;
        } else {
            t += ds / speed(k - 1).min(speed(k));
        }
        ts[k] = t;
    }
    ts
}

/// Per-waypoint speed ceiling (m/s) recovered from the stamps, or `None` when
/// the producer does not speak the dialect.
///
/// `profile.py` states it: `ts[i] - ts[i-1] = segment length / governor speed
/// for that segment`, where the governor speed is the clearance curve
/// evaluated at the tighter of the segment's two endpoints. So the inverse is
/// division -- `ds/dt` recovers `min(gov(clr[i-1]), gov(clr[i]))` exactly, in
/// m/s, with no need to round-trip through a synthetic clearance.
///
/// A port of `profile.decode_ceilings`, deliberately statement for statement:
///
/// * Fewer than two poses, or stamps that are flat or go backwards, mean the
///   producer does not speak the dialect. Returns `None`; the caller then
///   cruises at `max_speed`.
/// * Fan segments inherit the previous ceiling.
/// * The result is clipped into `[min_speed, max_speed]`, which is what makes
///   the channel safe to trust: a stamp can only ever ask the robot to be
///   *more* careful than cruise, never faster, and garbage stamps (a path
///   whose poses were default-constructed microseconds apart, say) saturate
///   at cruise instead of commanding something absurd.
///
/// What this is NOT is a schedule. The stamps are consumed as a per-waypoint
/// speed *ceiling* keyed to arc position; the absolute times, the plan's t0
/// and the tick clock never enter. Chasing the timeline would mean
/// accelerating to make up lost time in precisely the tight passages the
/// encoding is warning about -- `profile.py` says so in as many words, and it
/// is why `t` is not marshalled into any law that reads this.
/// `ds` comes from the waypoints, NOT from differencing cumulative arc length.
/// The two are not bit-identical, the encoder used the raw segment, and the
/// python twin (`profile.decode_ceilings`) uses the raw segment too -- so
/// reconstructing it from `arcs` costs parity for nothing.
pub fn decode_ceilings(ts: &[f64], path: &[[f64; 3]], cfg: &Params) -> Option<Vec<f64>> {
    let n = path.len();
    if n < 2 || ts.len() != n {
        return None;
    }
    // `np.any(dt < 0) or not np.any(dt > 0)` -- unstamped paths (all-equal
    // ts) and anything non-monotone are rejected outright.
    let mut any_positive = false;
    for k in 1..n {
        let dt = ts[k] - ts[k - 1];
        if dt < 0.0 {
            return None;
        }
        if dt > 0.0 {
            any_positive = true;
        }
    }
    if !any_positive {
        return None;
    }
    // min/max rather than the raw fields: `f64::clamp` panics when the bounds
    // cross, and a config is free to set min_speed above max_speed.
    let lo = cfg.min_speed.min(cfg.max_speed);
    let hi = cfg.max_speed.max(cfg.min_speed);
    let mut out = vec![hi; n];
    let mut prev = hi;
    for k in 1..n {
        let (dx, dy) = (path[k][0] - path[k - 1][0], path[k][1] - path[k - 1][1]);
        let ds = (dx * dx + dy * dy).sqrt();
        let dt = ts[k] - ts[k - 1];
        if ds >= FAN_EPS && dt > 0.0 {
            let v = ds / dt;
            // NaN would propagate straight out through the twist; the python
            // cannot produce one here because its inputs are the encoder's,
            // but this law takes whatever the wire hands it.
            if v.is_finite() {
                prev = v.clamp(lo, hi);
            }
        }
        out[k] = prev;
    }
    out[0] = out[1];
    Some(out)
}

/// Speed ceilings -> the clearance that reproduces them under the ENCODER's
/// governor.
///
/// A port of `profile.ceilings_to_clearance`, and the exact linear inverse of
/// `governor_speed` on `[MIN_SPEED, MAX_SPEED]`. Deliberately keyed to this
/// module's wire constants rather than to a consumer `Params`, for the same
/// reason `governor_speed` is: it undoes the encoder, and the encoder is the
/// producer's half of a wire contract.
///
/// WHY IT EXISTS. A follower on the hinted track with no cloud of its own has
/// no clearance array, but the hinted law's only room channel IS the clearance
/// argument (`laws::hinted::update` takes `Option<&[f64]>` and no stamps). So
/// the stamps are decoded to ceilings and bent back into the clearance that
/// would have produced them, which puts the planner's precision profile under
/// a law that cannot read stamps -- without touching the law, which is
/// parity-locked. Blind needs none of this: it decodes the stamps itself.
// `max` then `min` rather than `clamp`, for `governor_speed`'s reason: the
// input is whatever the stamps decoded to, and f64::clamp PANICS on a NaN.
#[allow(clippy::manual_clamp)]
pub fn ceilings_to_clearance(ceilings: &[f64]) -> Vec<f64> {
    ceilings
        .iter()
        .map(|&v| {
            // `np.clip` order, and `max` then `min` rather than `clamp`, which
            // panics on a NaN the wire is free to hand us
            let frac = (v.max(MIN_SPEED).min(MAX_SPEED) - MIN_SPEED) / (MAX_SPEED - MIN_SPEED);
            FLOOR_CLEARANCE + frac * (SPEED_CLEARANCE - FLOOR_CLEARANCE)
        })
        .collect()
}

/// The tightest decoded ceiling within `speed_lookahead` of `arcs[i]`.
///
/// Read from `i + 1` rather than `i`. That is not an off-by-one: a decoded
/// ceiling is a property of the SEGMENT ending at its waypoint, so
/// `ceilings[k]` already carries `clr[k-1]`. Scanning `[i+1 ..]` therefore
/// reproduces `gov(min clr over [i ..])` -- the clearance governor's window
/// exactly -- whereas starting at `i` would drag in the waypoint behind the
/// robot.
pub fn ceiling_ahead(ceilings: &[f64], arcs: &[f64], i: usize, cfg: &Params) -> f64 {
    let n = arcs.len();
    let hi = arcs[i] + cfg.speed_lookahead;
    // The segment about to be traversed always counts, even if a degenerate
    // `speed_lookahead` would exclude it; at the end of the plan there is no
    // next segment and the last ceiling stands.
    let mut room = ceilings[(i + 1).min(n - 1)];
    for k in (i + 1)..n {
        if arcs[k] > hi {
            break; // arcs are non-decreasing, so nothing later qualifies
        }
        if ceilings[k] < room {
            room = ceilings[k];
        }
    }
    room
}
