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
//! Wire dialect, not a law: `control/profile.py` (`encode_precision` / `decode_ceilings`) is the specification.

use crate::geom::Params;

/// Segments shorter than this are rotation in place; must match `profile._FAN_EPS`.
pub const FAN_EPS: f64 = 1e-6;

/// The embodiment's governor curve (`embodiment/base.py`), never a per-process tuning.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Governor {
    pub max_speed: f64,
    pub min_speed: f64,
    /// Room at which full speed is granted (m).
    pub speed_clearance: f64,
    /// The embodiment's precision floor (m).
    pub floor: f64,
    /// rad/s, prices fan segments' dt.
    pub max_yaw_rate: f64,
}

/// Clearance (m) -> speed ceiling (m/s), `profile.governor_speed`; infinite clearance means cruise.
// `max` then `min` rather than `clamp`: f64::clamp panics on NaN.
#[allow(clippy::manual_clamp)]
pub fn governor_speed(clearance: f64, gov: &Governor) -> f64 {
    let frac = (clearance - gov.floor) / (gov.speed_clearance - gov.floor);
    gov.min_speed + (gov.max_speed - gov.min_speed) * frac.max(0.0).min(1.0)
}

/// The path's timestamps carrying its precision profile: `profile.encode_precision`, the inverse of `decode_ceilings`.
/// `clearance` of the wrong length gives every segment cruise; fan segments are priced by yaw span.
pub fn encode_precision(path: &[[f64; 3]], clearance: &[f64], t0: f64, gov: &Governor) -> Vec<f64> {
    let n = path.len();
    if n == 0 {
        return Vec::new();
    }
    let use_clearance = clearance.len() == n;
    let speed = |k: usize| {
        if use_clearance {
            governor_speed(clearance[k], gov)
        } else {
            gov.max_speed
        }
    };
    let mut ts = vec![t0; n];
    let mut t = t0;
    for k in 1..n {
        let (dx, dy) = (path[k][0] - path[k - 1][0], path[k][1] - path[k - 1][1]);
        let ds = (dx * dx + dy * dy).sqrt();
        if ds < FAN_EPS {
            // floor-mod like `np.remainder`, not the IEEE remainder angle_diff uses
            let dyaw = path[k][2] - path[k - 1][2];
            let wrapped = (dyaw + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI;
            t += wrapped.abs() / gov.max_yaw_rate;
        } else {
            t += ds / speed(k - 1).min(speed(k));
        }
        ts[k] = t;
    }
    ts
}

/// Per-waypoint speed ceiling (m/s) from the stamps, or `None` when the producer does not speak the dialect.
/// `profile.decode_ceilings` statement for statement; fan segments inherit the previous ceiling.
pub fn decode_ceilings(ts: &[f64], path: &[[f64; 3]], cfg: &Params) -> Option<Vec<f64>> {
    let n = path.len();
    if n < 2 || ts.len() != n {
        return None;
    }
    // `np.any(dt < 0) or not np.any(dt > 0)`: flat or backwards stamps are rejected
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
    // `f64::clamp` panics when the bounds cross, and a config may set min_speed above max_speed
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
            // NaN would propagate straight into the twist; the wire can hand us one
            if v.is_finite() {
                prev = v.clamp(lo, hi);
            }
        }
        out[k] = prev;
    }
    out[0] = out[1];
    Some(out)
}

/// The tightest decoded ceiling within `speed_lookahead` of `arcs[i]`.
/// Scans from `i + 1`: a ceiling belongs to the segment ending at its waypoint.
pub fn ceiling_ahead(ceilings: &[f64], arcs: &[f64], i: usize, cfg: &Params) -> f64 {
    let n = arcs.len();
    let hi = arcs[i] + cfg.speed_lookahead;
    // the next segment always counts, even under a degenerate `speed_lookahead`
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
