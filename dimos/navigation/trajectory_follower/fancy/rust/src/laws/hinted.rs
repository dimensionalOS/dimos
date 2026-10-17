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

//! The follower's law: the seed plus gait slip inverse, constant time headway,
//! and the stamped precision profile as the governor's input. Kept out of
//! `geom` so the frozen `seed` baseline cannot move.

use crate::geom::{
    arcs_of, body_error, carrot_lerp, clearance_governor, fan_target, progress_index, yaw_command,
    Params,
};
use crate::stamps::{ceiling_ahead, decode_ceilings};

/// `Params` plus the embodiment's gait calibration (`walk_gain`/`walk_slip`/
/// `walk_slip_ramp`); re-probe on a different gait before it drives hardware.
pub struct HintedParams {
    pub base: Params,
    pub walk_gain: f64,
    pub walk_slip: f64,
    /// Below this the slip correction fades to zero, so a stop stays a stop.
    pub slip_ramp: f64,
}

/// The command for a ground speed of `want`: inverse of `ground ~= gain * cmd - slip`
/// (`laws/hinted.py::walk_command`). Identity at 0, full inverse from `ramp` up.
#[inline]
pub fn walk_command(want: f64, gain: f64, slip: f64, ramp: f64) -> f64 {
    if want <= 0.0 {
        return 0.0;
    }
    let correction = (gain * want + slip) - want;
    want + correction * (want / ramp).min(1.0)
}

/// One controller tick; `(vx, vy, wz)` in the body frame. `clearance` and `ts`
/// carry the same room hint (`stamps::decode_ceilings`), either is optional.
pub fn update(
    pose: (f64, f64, f64),
    path: &[[f64; 3]],
    clearance: Option<&[f64]>,
    ts: Option<&[f64]>,
    cfg: &HintedParams,
) -> (f64, f64, f64) {
    if path.len() < 2 {
        // empty path or single-pose stub: the planner is saying "stop"
        return (0.0, 0.0, 0.0);
    }
    let base = &cfg.base;
    let (px, py, pyaw) = pose;
    let arcs = arcs_of(path);
    let i = progress_index(path, &arcs, px, py, pyaw);

    // Governor first, since the lookahead derives from it. Without a clearance
    // array the same ceiling is decoded off the stamps: alternatives, not layers.
    let vmax = clearance_governor(&arcs, i, clearance, base)
        .or_else(|| {
            let ceilings = decode_ceilings(ts?, path, base)?;
            Some(ceiling_ahead(&ceilings, &arcs, i, base))
        })
        .unwrap_or(base.max_speed);

    // Constant time headway: a fixed carrot chords turns toward the obstacle.
    // The floor keeps min(k_pos * L, vmax) saturating at vmax, so no speed is lost.
    let headway = base.lookahead / base.max_speed.max(1e-6);
    let look = (vmax * headway).max(vmax / base.k_pos.abs().max(1e-6));

    let (target_xy, target_yaw) =
        fan_target(path, &arcs, i, pyaw, base).unwrap_or_else(|| carrot_lerp(path, &arcs, i, look));

    let (bx, by) = body_error(px, py, pyaw, target_xy);
    let (mut vx, mut vy) = (base.k_pos * bx, base.k_pos * by);
    let speed = vx.hypot(vy);
    if speed > 1e-12 {
        // `want` is the intended ground speed; `cmd` is what the gait must be asked for
        let want = speed.min(vmax);
        let cmd = walk_command(want, cfg.walk_gain, cfg.walk_slip, cfg.slip_ramp);
        vx = vx / speed * cmd;
        vy = vy / speed * cmd;
    }
    (vx, vy, yaw_command(target_yaw, pyaw, base))
}
