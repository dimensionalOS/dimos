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

//! The reference pursuit law: holonomic, clearance-governed, fixed lookahead.
//!
//! Port of `control/laws/seed.py::PursuitController.update`. This law is the
//! permanent baseline -- every track's A/B is against it and every lab seeds
//! from it -- so it does NOT absorb research results. Fold those into the
//! track's own law instead; a moving baseline is not a baseline.

use crate::geom::{
    arcs_of, body_error, carrot_snap, clearance_governor, fan_target, progress_index, yaw_command,
    Params,
};

/// One controller tick. `path` is the plan as (x, y, yaw) rows; `clearance`
/// is the optional per-waypoint room annotation. Returns `(vx, vy, wz)` in
/// the body frame.
pub fn update(
    pose: (f64, f64, f64),
    path: &[[f64; 3]],
    clearance: Option<&[f64]>,
    cfg: &Params,
) -> (f64, f64, f64) {
    if path.len() < 2 {
        // empty path or a single-pose veto stub: there is nothing to
        // follow -- hold position (the planner is saying "stop")
        return (0.0, 0.0, 0.0);
    }
    let (px, py, pyaw) = pose;
    let arcs = arcs_of(path);
    let i = progress_index(path, &arcs, px, py, pyaw);

    let (target_xy, target_yaw) = fan_target(path, &arcs, i, pyaw, cfg)
        .unwrap_or_else(|| carrot_snap(path, &arcs, i, cfg.lookahead));

    // speed governor: cap cruise by the room ahead, when we know it
    let vmax = clearance_governor(&arcs, i, clearance, cfg).unwrap_or(cfg.max_speed);

    // body-frame error -> velocity
    let (bx, by) = body_error(px, py, pyaw, target_xy);
    let (mut vx, mut vy) = (cfg.k_pos * bx, cfg.k_pos * by);
    let speed = vx.hypot(vy);
    if speed > vmax {
        vx = vx / speed * vmax;
        vy = vy / speed * vmax;
    }
    (vx, vy, yaw_command(target_yaw, pyaw, cfg))
}
