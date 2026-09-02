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

//! The pieces every pursuit law is built from: arc length, progress along the
//! plan, fan detection, carrot selection, the clearance governor, and the
//! body-frame error terms.
//!
//! This module is SHARED across laws and is deliberately conservative --
//! each function reproduces one statement of `control/laws/seed.py` and the
//! laws in `laws/` compose them in their own order. A research branch that
//! wants different geometry adds a function here rather than editing one, so
//! that folding in a generation cannot silently move the baseline.
//!
//! NUMERICS. Parity with the python is per-operation, not per-formula: the
//! operation ORDER and the exact tie-breaks are preserved (`argmin` takes the
//! first minimum, `searchsorted` is side='left'), and angle wrapping is IEEE
//! remainder like `math.remainder`, never `%` or `rem_euclid`.

/// The numbers a law reads: the body's tuning plus its plant, driving inside
/// one band -- `emb::base_params` builds it from an `Emb`.
#[derive(Clone)]
pub struct Params {
    pub lookahead: f64,
    pub max_speed: f64,
    pub max_yaw_rate: f64,
    pub k_pos: f64,
    pub k_yaw: f64,
    pub fan_yaw_per_m: f64,
    pub fan_yaw_done: f64,
    pub min_speed: f64,
    pub speed_clearance: f64,
    pub speed_floor_clearance: f64,
    pub speed_lookahead: f64,
}

pub const TAU: f64 = std::f64::consts::TAU;

/// IEEE-754 remainder, i.e. `math.remainder`: the quotient rounds half to
/// EVEN, which is what puts the result in [-y/2, y/2] and what makes
/// `remainder(pi, tau)` come out `+pi` rather than `-pi`. Neither `%`
/// (truncated) nor `rem_euclid` (always non-negative) is this function.
#[inline]
pub fn ieee_remainder(x: f64, y: f64) -> f64 {
    x - (x / y).round_ties_even() * y
}

/// Shortest signed angle from `b` to `a` -- the python `_angle_diff`.
#[inline]
pub fn angle_diff(a: f64, b: f64) -> f64 {
    ieee_remainder(a - b, TAU)
}

/// Cumulative arc length along the plan.
///
/// `concatenate([[0.0], cumsum(norm(diff(xy, axis=0), axis=1))])`; cumsum
/// accumulates left to right, so the running sum stays sequential.
pub fn arcs_of(path: &[[f64; 3]]) -> Vec<f64> {
    let n = path.len();
    let mut arcs = vec![0.0f64; n];
    for k in 1..n {
        let (dx, dy) = (path[k][0] - path[k - 1][0], path[k][1] - path[k - 1][1]);
        arcs[k] = arcs[k - 1] + (dx * dx + dy * dy).sqrt();
    }
    arcs
}

/// Progress along the plan: the closest waypoint, advanced through a fan.
///
/// Inside a fan the waypoints are coincident, so the closest-point test
/// cannot separate them; advance by yaw progress instead, or the law
/// re-rotates from the fan's first pose every tick.
pub fn progress_index(path: &[[f64; 3]], arcs: &[f64], px: f64, py: f64, pyaw: f64) -> usize {
    let n = path.len();
    let mut i = 0usize;
    let mut best = f64::INFINITY;
    for (k, p) in path.iter().enumerate() {
        // strict `<`: np.argmin keeps the FIRST minimum on a tie
        let d = ((p[0] - px) * (p[0] - px) + (p[1] - py) * (p[1] - py)).sqrt();
        if d < best {
            best = d;
            i = k;
        }
    }
    while i + 1 < n
        && arcs[i + 1] - arcs[i] < 1e-6
        && angle_diff(path[i + 1][2], pyaw).abs() < angle_diff(path[i][2], pyaw).abs()
    {
        i += 1;
    }
    i
}

/// The fan target at `i`, or `None` when this is not a fan to execute.
///
/// Yaw stepping with (near-)zero displacement means the planner commands a
/// rotation here; hold the fan waypoint and rotate until the yaw error drops
/// under `fan_yaw_done`.
pub fn fan_target(
    path: &[[f64; 3]],
    arcs: &[f64],
    i: usize,
    pyaw: f64,
    cfg: &Params,
) -> Option<([f64; 2], f64)> {
    let n = path.len();
    let j = (i + 1).min(n - 1);
    let ds = arcs[j] - arcs[i];
    let dyaw = angle_diff(path[j][2], path[i][2]).abs();
    let in_fan = j > i && dyaw > 1e-6 && dyaw / ds.max(1e-6) > cfg.fan_yaw_per_m;
    if in_fan && angle_diff(path[j][2], pyaw).abs() > cfg.fan_yaw_done {
        Some(([path[i][0], path[i][1]], path[j][2]))
    } else {
        None
    }
}

/// First waypoint at or past `arcs[i] + look` -- the seed's carrot.
///
/// `np.searchsorted(arcs, s)` with the default side='left': the first index
/// whose arc is >= s, which on sorted data is the count of strictly smaller
/// entries.
pub fn carrot_snap(path: &[[f64; 3]], arcs: &[f64], i: usize, look: f64) -> ([f64; 2], f64) {
    let n = path.len();
    let s = arcs[i] + look;
    let k = arcs.partition_point(|&a| a < s).min(n - 1);
    ([path[k][0], path[k][1]], path[k][2])
}

/// The point at exactly `arcs[i] + look`, interpolated within its segment.
///
/// The plan is discretised at 0.1 m. That is fine noise against a 0.35 m
/// carrot but 70% of a 0.14 m one, so any law that shortens its lookahead
/// needs this: snapping to the next waypoint would make the carrot distance,
/// and so the commanded heading, chatter waypoint to waypoint.
pub fn carrot_lerp(path: &[[f64; 3]], arcs: &[f64], i: usize, look: f64) -> ([f64; 2], f64) {
    let n = path.len();
    let s = arcs[i] + look;
    let k = arcs.partition_point(|&a| a < s);
    if k == 0 || k >= n {
        // s is at or beyond an endpoint: pursue the endpoint itself
        let k = k.min(n - 1);
        return ([path[k][0], path[k][1]], path[k][2]);
    }
    let (a0, a1) = (arcs[k - 1], arcs[k]);
    let d = a1 - a0;
    if d <= 1e-9 {
        return ([path[k][0], path[k][1]], path[k][2]);
    }
    let u = ((s - a0) / d).clamp(0.0, 1.0);
    let x = path[k - 1][0] + u * (path[k][0] - path[k - 1][0]);
    let y = path[k - 1][1] + u * (path[k][1] - path[k - 1][1]);
    // interpolate yaw the short way round, not linearly in the raw angle, so
    // a wrap across +-pi does not spin the carrot
    let yw = path[k - 1][2] + u * angle_diff(path[k][2], path[k - 1][2]);
    ([x, y], yw)
}

/// Speed ceiling from the room ahead, or `None` when the annotation is absent
/// or the wrong length (ignored, exactly as in the python).
///
/// Cruise at `max_speed` with `speed_clearance` of room, creep at `min_speed`
/// at the precision floor, linear between; judged over the next
/// `speed_lookahead` metres of plan.
pub fn clearance_governor(
    arcs: &[f64],
    i: usize,
    clearance: Option<&[f64]>,
    cfg: &Params,
) -> Option<f64> {
    let clr = clearance?;
    if clr.len() != arcs.len() {
        return None;
    }
    // the mask (arcs >= arcs[i]) & (arcs <= arcs[i] + speed_lookahead) is not
    // a contiguous slice -- coincident fan waypoints before `i` share its arc
    // -- so scan the whole array like numpy does
    let hi = arcs[i] + cfg.speed_lookahead;
    let mut room: Option<f64> = None;
    for (k, &a) in arcs.iter().enumerate() {
        if a >= arcs[i] && a <= hi && room.is_none_or(|m| clr[k] < m) {
            room = Some(clr[k]);
        }
    }
    // arcs[i] always passes its own mask, so the window is never empty; the
    // fallback is here because the python spells it out
    let room = room.unwrap_or(clr[i]);
    let frac = (room - cfg.speed_floor_clearance)
        / (cfg.speed_clearance - cfg.speed_floor_clearance).max(1e-6);
    Some(cfg.min_speed + (cfg.max_speed - cfg.min_speed) * frac.clamp(0.0, 1.0))
}

/// Position error rotated into the body frame.
#[inline]
pub fn body_error(px: f64, py: f64, pyaw: f64, target_xy: [f64; 2]) -> (f64, f64) {
    let (ex, ey) = (target_xy[0] - px, target_xy[1] - py);
    let (c, s_) = ((-pyaw).cos(), (-pyaw).sin());
    (c * ex - s_ * ey, s_ * ex + c * ey)
}

/// Yaw rate toward `target_yaw`, clamped into the configured envelope.
///
/// `np.clip` = minimum(maximum(v, lo), hi); spelled out rather than `clamp`,
/// which panics when a config sets a negative max_yaw_rate.
#[inline]
pub fn yaw_command(target_yaw: f64, pyaw: f64, cfg: &Params) -> f64 {
    (cfg.k_yaw * angle_diff(target_yaw, pyaw))
        .max(-cfg.max_yaw_rate)
        .min(cfg.max_yaw_rate)
}
