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

//! The reference pursuit law: holonomic, clearance-governed, fixed lookahead. Port of
//! `laws/seed.py::PursuitController.update`. The frozen baseline: research results go
//! into the track's own law, never here.

use crate::geom::{
    arcs_of, body_error, carrot_snap, clearance_governor, fan_target, progress_index, yaw_command,
    Params,
};

/// One tick. `path` is (x, y, yaw) rows, `clearance` the optional per-waypoint room
/// annotation; returns the body-frame `(vx, vy, wz)`.
pub fn update(
    pose: (f64, f64, f64),
    path: &[[f64; 3]],
    clearance: Option<&[f64]>,
    cfg: &Params,
) -> (f64, f64, f64) {
    if path.len() < 2 {
        // empty path or single-pose veto stub: the planner says stop
        return (0.0, 0.0, 0.0);
    }
    let (px, py, pyaw) = pose;
    let arcs = arcs_of(path);
    let i = progress_index(path, &arcs, px, py, pyaw);

    let (target_xy, target_yaw) = fan_target(path, &arcs, i, pyaw, cfg)
        .unwrap_or_else(|| carrot_snap(path, &arcs, i, cfg.lookahead));

    let vmax = clearance_governor(&arcs, i, clearance, cfg).unwrap_or(cfg.max_speed);

    let (bx, by) = body_error(px, py, pyaw, target_xy);
    let (mut vx, mut vy) = (cfg.k_pos * bx, cfg.k_pos * by);
    let speed = vx.hypot(vy);
    if speed > vmax {
        vx = vx / speed * vmax;
        vy = vy / speed * vmax;
    }
    (vx, vy, yaw_command(target_yaw, pyaw, cfg))
}
