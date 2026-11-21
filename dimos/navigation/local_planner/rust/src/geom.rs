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

//! Pursuit-law building blocks for the trajectory follower's law.

/// The numbers a law reads: the body's tuning plus its plant, driving inside
/// one band; `emb::base_params` builds it from an `Emb`.
#[derive(Clone)]
pub struct Params {
    pub lookahead: f64,
    pub max_speed: f64,
    pub max_yaw_rate: f64,
    /// The slowest turn the body executes, held past `fan_yaw_done`.
    pub min_yaw_rate: f64,
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

/// `math.remainder`: quotient rounds half to even, so `remainder(pi, tau)` is
/// `+pi`. Neither `%` nor `rem_euclid` is this function.
#[inline]
pub fn ieee_remainder(x: f64, y: f64) -> f64 {
    x - (x / y).round_ties_even() * y
}

/// Shortest signed angle from `b` to `a`.
#[inline]
pub fn angle_diff(a: f64, b: f64) -> f64 {
    ieee_remainder(a - b, TAU)
}

/// Cumulative arc length along the plan, summed left to right like `cumsum`.
pub fn arcs_of(path: &[[f64; 3]]) -> Vec<f64> {
    let n = path.len();
    let mut arcs = vec![0.0f64; n];
    for k in 1..n {
        let (dx, dy) = (path[k][0] - path[k - 1][0], path[k][1] - path[k - 1][1]);
        arcs[k] = arcs[k - 1] + (dx * dx + dy * dy).sqrt();
    }
    arcs
}

/// Meters one radian of heading counts as when matching a pose to a path.
const YAW_M: f64 = 0.4;

/// Closest waypoint, heading counted with position, so a spot the path turns
/// on or passes twice is matched by the way the body faces.
pub fn progress_index(path: &[[f64; 3]], px: f64, py: f64, pyaw: f64) -> usize {
    let mut i = 0usize;
    let mut best = f64::INFINITY;
    for (k, p) in path.iter().enumerate() {
        let d = (p[0] - px).hypot(p[1] - py) + YAW_M * angle_diff(p[2], pyaw).abs();
        if d < best {
            best = d;
            i = k;
        }
    }
    i
}

/// The fan target at `i` (hold position, rotate until under `fan_yaw_done`),
/// or `None` when this is not a fan to execute.
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

/// The point at exactly `arcs[i] + look`, interpolated within its segment;
/// a short lookahead snapped to 0.1 m waypoints would chatter the heading.
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
    // yaw the short way round, so a wrap across +-pi does not spin the carrot
    let yw = path[k - 1][2] + u * angle_diff(path[k][2], path[k - 1][2]);
    ([x, y], yw)
}

/// Speed ceiling from the room over the next `speed_lookahead` m, linear from
/// `min_speed` to `max_speed`; `None` when the annotation is absent or the wrong length.
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
    // not a contiguous slice: fan waypoints before `i` share its arc, so scan
    // the whole array
    let hi = arcs[i] + cfg.speed_lookahead;
    let mut room: Option<f64> = None;
    for (k, &a) in arcs.iter().enumerate() {
        if a >= arcs[i] && a <= hi && room.is_none_or(|m| clr[k] < m) {
            room = Some(clr[k]);
        }
    }
    // never empty since arcs[i] passes its own mask
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

/// Yaw rate toward `target_yaw`, clipped like `np.clip`; `clamp` would panic
/// on a negative `max_yaw_rate`.
#[inline]
pub fn yaw_command(target_yaw: f64, pyaw: f64, cfg: &Params) -> f64 {
    (cfg.k_yaw * angle_diff(target_yaw, pyaw))
        .max(-cfg.max_yaw_rate)
        .min(cfg.max_yaw_rate)
}
