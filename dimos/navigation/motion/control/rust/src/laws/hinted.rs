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

//! The hinted track's law: clear the plant's dead zone, brake late, ramp your
//! own command.
//!
//! Mechanisms over the seed:
//!
//! 1. WALK-THRESHOLD ENVELOPE. `min_speed` 0.2 sits inside the freewalk
//!    policy's dead zone -- gait initiation is a bifurcation, not a ramp, and
//!    below ~0.28 commanded the robot stands. The envelope is 0.45/0.95
//!    commanded, carried by this law's own config default (see
//!    the embodiment's `gait_band`).
//! 2. TANGENT FEEDFORWARD + FOOT CORRECTION, replacing the seed's aim-at-the-
//!    carrot term, whose chord cut corners toward the obstacle the plan curved
//!    around. This is also what makes the governor authoritative: an on-plan
//!    tick now commands exactly `vmax`.
//! 3. BRAKE-FEASIBLE PREVIEW. The flat 2 m minimum demanded a pinch's speed
//!    2 m out; each waypoint now imposes only `sqrt(v^2 + 2 a (d - margin))`.
//!    `brake_accel = 0` reproduces the flat min bit for bit.
//! 4. SELF-RATE-LIMITED COMMAND (`Law`), so the request the law signs its name
//!    to is the request the robot executes.
//!
//! Plus the PINCH-ESCAPE leg: the governor's lower anchor rises again as the
//! room runs out, because what kills in a gap is dwell, and creeping buys no
//! precision once the corridor is already inside the embodiment's floor.
//!
//! NOT shared with the blind track on purpose. Blind cancels the same dead zone
//! with an actuator inverse (`laws/blind.rs::walk_command`) rather than by
//! moving the envelope; the two tracks are researched independently and
//! handing either lab the other's mechanism biases its search.

use crate::geom::{angle_diff, arcs_of, progress_index, Params};

/// Tick period assumed when the law has no previous tick to difference against
/// (the first call after `reset()`), and the cap on the period it will
/// integrate over. The plant's control rate is 50 Hz; a gap longer than
/// `MAX_TICK` means the caller stalled, and letting the limiter bank that time
/// would hand back exactly the unbounded step it exists to prevent.
pub const NOMINAL_TICK: f64 = 0.02;
pub const MAX_TICK: f64 = 0.10;

/// `Params` plus the terms the seed had no use for.
#[derive(Clone)]
pub struct HintedParams {
    pub base: Params,
    /// The plant's own per-axis command rate limit, in commanded units per
    /// SECOND: the embodiment's `command_slew`. A rate rather than a per-tick
    /// step so the limiter is correct at any control period.
    pub slew: [f64; 3],
    /// How far ahead of and behind the foot of the perpendicular the
    /// feedforward reads the plan's direction. NOT a lookahead in the pursuit
    /// sense -- the velocity is never aimed at these points, only the
    /// DIRECTION between them is taken, and the window is CENTRED so it carries
    /// no inward bias at all on constant curvature.
    pub tangent_preview: f64,
    /// Upper knee of the governor's pinch-escape leg: the room at or above
    /// which the leg does nothing.
    pub escape_clearance: f64,
    /// How far ahead the escape leg reads the room. Sized to COMMITMENT rather
    /// than occupancy -- about one body length plus margin -- and deliberately
    /// well short of the ramp's `speed_lookahead`.
    pub escape_preview: f64,
    /// Top of the escape leg, where the room has run out entirely.
    pub escape_speed: f64,
    /// Deceleration the preview credits the body with on its way to a previewed
    /// waypoint. 0 reproduces the flat minimum exactly.
    pub brake_accel: f64,
    /// Arc within which a previewed waypoint binds as hard as if the body were
    /// standing on it.
    pub brake_margin: f64,
}

/// Foot of the perpendicular from `(px, py)` onto the path polyline, and its
/// arc length.
///
/// Where on the LINE the body is, rather than which waypoint it is nearest --
/// the quantity the judge measures (`judge.cross_track` projects onto segments
/// too). Zero-length (fan) segments project to their own point, which is what
/// the clamp to `[0, 1]` already does.
fn project_onto(path: &[[f64; 3]], arcs: &[f64], px: f64, py: f64) -> ([f64; 2], f64) {
    let mut best = f64::INFINITY;
    let mut foot = [path[0][0], path[0][1]];
    let mut s_star = 0.0;
    for m in 0..path.len() - 1 {
        let (ax, ay) = (path[m][0], path[m][1]);
        let (dx, dy) = (path[m + 1][0] - ax, path[m + 1][1] - ay);
        let dd = dx * dx + dy * dy;
        let u = if dd > 1e-12 {
            (((px - ax) * dx + (py - ay) * dy) / dd).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (qx, qy) = (ax + u * dx, ay + u * dy);
        let d = (px - qx).hypot(py - qy);
        // strict `<` keeps the FIRST minimum, the same tie-break the seed's
        // argmin has -- a plan that doubles back must not snap forward
        if d < best {
            best = d;
            foot = [qx, qy];
            s_star = arcs[m] + u * (arcs[m + 1] - arcs[m]);
        }
    }
    (foot, s_star)
}

/// Point on the polyline at arc `s`, clamped to its ends.
fn point_at(path: &[[f64; 3]], arcs: &[f64], s: f64) -> [f64; 2] {
    let n = path.len();
    if s <= 0.0 {
        return [path[0][0], path[0][1]];
    }
    if s >= arcs[n - 1] {
        return [path[n - 1][0], path[n - 1][1]];
    }
    let k = arcs.partition_point(|&a| a < s).clamp(1, n - 1);
    let (a0, a1) = (arcs[k - 1], arcs[k]);
    let u = if a1 > a0 { (s - a0) / (a1 - a0) } else { 0.0 };
    [
        path[k - 1][0] + u * (path[k][0] - path[k - 1][0]),
        path[k - 1][1] + u * (path[k][1] - path[k - 1][1]),
    ]
}

/// Unit direction of travel at arc `s`: the chord across a window CENTRED on
/// `s`, half-width `preview / 2`.
///
/// Centred, not forward-looking, and that is the whole design of it. A forward
/// chord from `s` to `s + preview` is rotated toward the inside of a curve by
/// `preview / (2R)`, so the feedforward would push the body inward until the
/// correction balanced it -- a standing cross-track error that simply
/// reintroduces the seed's chord bug at small scale. A centred chord is the
/// average tangent over the window, which on constant curvature is the tangent
/// at `s` exactly: the bias is zero and only the anticipation survives.
fn tangent_at(path: &[[f64; 3]], arcs: &[f64], s: f64, preview: f64) -> (f64, f64) {
    let total = arcs[path.len() - 1];
    // widen if the window is degenerate (a plan made of fans has no arc to
    // difference across); the last resort is the whole plan's own direction
    for h in [preview * 0.5, preview * 1.5, total] {
        let a = point_at(path, arcs, (s - h).max(0.0));
        let b = point_at(path, arcs, (s + h).min(total));
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let len = dx.hypot(dy);
        if len > 1e-9 {
            return (dx / len, dy / len);
        }
    }
    (0.0, 0.0)
}

/// The law plus the only state it keeps: its own previous command, and the tick
/// time that command was issued at.
///
/// WHY THE LAW OWNS A RATE LIMITER AT ALL. Rate limiting and a pure transport
/// delay COMMUTE, and the plant's limiter is idempotent on a stream that already
/// satisfies it -- so feeding the plant a slew-compliant stream leaves what the
/// policy sees unchanged. What this buys is not a quieter loop but an honest
/// one: the request the law signs its name to is the request the robot executes,
/// rather than one the plant's ramp silently edits. It is not anti-churn
/// reasoning -- nothing here treats a fast command as dangerous. The claim is
/// narrower and mechanical: a command the plant cannot deliver is not a command.
#[derive(Debug, Default, Clone)]
pub struct Law {
    last_cmd: Option<(f64, f64, f64)>,
    last_t: Option<f64>,
}

impl Law {
    pub fn new() -> Self {
        Self::default()
    }

    /// Drop every tick of history. A fresh `Law` and a reset `Law` must answer
    /// an identical call sequence identically.
    pub fn reset(&mut self) {
        self.last_cmd = None;
        self.last_t = None;
    }

    /// One tick: the pure law, then the plant's own slew.
    pub fn step(
        &mut self,
        pose: (f64, f64, f64),
        path: &[[f64; 3]],
        clearance: Option<&[f64]>,
        cfg: &HintedParams,
        t: f64,
    ) -> (f64, f64, f64) {
        let raw = update(pose, path, clearance, cfg);

        // THE VETO IS NOT RATE LIMITED. A path shorter than two poses is the
        // planner saying "stop", and the deployment contract is that the law
        // obeys it by commanding zero -- not by ramping toward zero over the
        // ~12 ticks the slew would take. "Hold position" is a contract about
        // what the law ASKS for, and the limiter must not quietly turn it into
        // "coast to a halt". The remembered command follows the command
        // actually issued, so the ramp model stays a model of this law's own
        // output.
        if path.len() < 2 {
            self.last_cmd = Some((0.0, 0.0, 0.0));
            self.last_t = if t.is_finite() { Some(t) } else { None };
            return (0.0, 0.0, 0.0);
        }

        // Elapsed since the command currently standing at the plant's ramp. A
        // non-finite, zero or backwards `t` carries no information about how
        // much the ramp advanced, so fall back to the nominal period rather
        // than to an unbounded (or negative) step.
        let dt = match self.last_t {
            Some(t0) if t.is_finite() && t > t0 => (t - t0).min(MAX_TICK),
            _ => NOMINAL_TICK,
        };
        let (px, py, pw) = self.last_cmd.unwrap_or((0.0, 0.0, 0.0));
        let (sx, sy, sw) = (cfg.slew[0] * dt, cfg.slew[1] * dt, cfg.slew[2] * dt);
        // spelled out rather than `clamp`, which panics on an inverted range
        let mut vx = px + (raw.0 - px).max(-sx).min(sx);
        let mut vy = py + (raw.1 - py).max(-sy).min(sy);
        let wz = (pw + (raw.2 - pw).max(-sw).min(sw))
            .max(-cfg.base.max_yaw_rate)
            .min(cfg.base.max_yaw_rate);

        // THE DECLARED ENVELOPE IS A DISC AND THE SLEW IS A BOX, so a step
        // taken toward a point inside the disc can still leave it: `vx` and
        // `vy` each land somewhere between the previous command and the
        // request, and that rectangle has corners the disc does not contain.
        // The overshoot is tiny and correcting it always REDUCES speed, but it
        // is orders of magnitude past the parity tolerance -- dropping this
        // would put the law outside its own declared envelope.
        let speed = vx.hypot(vy);
        if speed > cfg.base.max_speed && speed > 0.0 {
            vx = vx / speed * cfg.base.max_speed;
            vy = vy / speed * cfg.base.max_speed;
        }

        if vx.is_finite() && vy.is_finite() && wz.is_finite() {
            self.last_cmd = Some((vx, vy, wz));
            self.last_t = Some(t);
        }
        (vx, vy, wz)
    }
}

/// The pure law, before the rate limiter. `path` is the plan as (x, y, yaw)
/// rows; `clearance` is the optional per-waypoint room annotation. Returns
/// `(vx, vy, wz)` in the body frame.
pub fn update(
    pose: (f64, f64, f64),
    path: &[[f64; 3]],
    clearance: Option<&[f64]>,
    cfg: &HintedParams,
) -> (f64, f64, f64) {
    if path.len() < 2 {
        // empty path or a single-pose veto stub: there is nothing to
        // follow -- hold position (the planner is saying "stop")
        return (0.0, 0.0, 0.0);
    }
    let base = &cfg.base;
    let (px, py, pyaw) = pose;
    let n = path.len();
    let arcs = arcs_of(path);
    let i = progress_index(path, &arcs, px, py, pyaw);

    // fan detection at the current position: yaw stepping with (near-)zero
    // displacement means the planner commands a rotation here
    let j = (i + 1).min(n - 1);
    let ds = arcs[j] - arcs[i];
    let dyaw = angle_diff(path[j][2], path[i][2]).abs();
    let in_fan = j > i && dyaw > 1e-6 && dyaw / ds.max(1e-6) > base.fan_yaw_per_m;
    let rotating = in_fan && angle_diff(path[j][2], pyaw).abs() > base.fan_yaw_done;

    let (foot, s_star) = project_onto(path, &arcs, px, py);

    // `yaw_ff` is the plan's turn rate per metre of arc over the window the
    // body is about to cross; multiplied by the commanded speed below it
    // becomes the open-loop yaw rate that holds the heading error at zero.
    let mut rotation_in_window = false;
    let (target_yaw, yaw_ff) = if rotating {
        // the planner commands a rotation here: hold station and turn
        (path[j][2], 0.0)
    } else {
        let s = arcs[i] + base.lookahead;
        let k = arcs.partition_point(|&a| a < s).min(n - 1);
        // POSITION still leads by `lookahead` -- the carrot is what makes the
        // holonomic body converge onto the line. ORIENTATION does not: a body
        // held at the next segment's heading while still inside this one sweeps
        // its corners across a corridor it has not left yet. So aim at the
        // heading the plan wants HERE, and cancel the P-loop's standing error
        // with feedforward instead of with lead. Estimated over a real span
        // only, and bounded by `fan_yaw_per_m`.
        let span = arcs[k] - arcs[i];
        let raw_rate = if span > 0.05 {
            angle_diff(path[k][2], path[i][2]) / span
        } else {
            0.0
        };
        rotation_in_window = raw_rate.abs() > base.fan_yaw_per_m;
        let rate = raw_rate.max(-base.fan_yaw_per_m).min(base.fan_yaw_per_m);
        (path[i][2], rate)
    };

    // speed governor: cap cruise by the room ahead, when we know it
    let mut vmax = base.max_speed;
    // Does the ramp find full cruise over its WHOLE window? That is the
    // separator the yaw feedforward's silence condition is written against, and
    // it has to keep meaning "there is room everywhere ahead" rather than "the
    // braking profile happens to permit cruise here". Blind ticks (no
    // annotation, or a length mismatch) leave it true, which is what
    // `vmax == max_speed` did for them.
    let mut ramp_unlimited = true;
    if let Some(clr) = clearance {
        if clr.len() == n {
            // -- THE PREVIEW IS A BRAKING PROFILE, NOT A MINIMUM --------------
            //
            // The flat min asked the body to already be at a pinch's speed 2 m
            // before reaching it, which is 14x the 0.14 m the 0.95 -> 0.45
            // deceleration actually costs at the executor's own slew. Each
            // previewed waypoint now imposes only what it can still impose
            // given that the body may brake at `brake_accel` on the way there.
            // At `d_k <= brake_margin` that is exactly `v(c_k)`, so the
            // waypoint the body is standing on binds as hard as it ever did.
            //
            // Still a MIN over the same window over the same annotation, so the
            // cap is monotone in `brake_accel` and `a = 0` is the seed's
            // governor bit for bit. Nothing here can make it SLOWER.
            let hi = arcs[i] + base.speed_lookahead;
            let mut room: Option<f64> = None;
            let mut cap = base.max_speed;
            let denom = (base.speed_clearance - base.speed_floor_clearance).max(1e-6);
            for (k, &a) in arcs.iter().enumerate() {
                if a >= arcs[i] && a <= hi {
                    if room.is_none_or(|m| clr[k] < m) {
                        room = Some(clr[k]);
                    }
                    let frac_k = ((clr[k] - base.speed_floor_clearance) / denom).clamp(0.0, 1.0);
                    let v_k = base.min_speed + (base.max_speed - base.min_speed) * frac_k;
                    let d_k = (a - arcs[i] - cfg.brake_margin).max(0.0);
                    let reachable = (v_k * v_k + 2.0 * cfg.brake_accel * d_k).sqrt();
                    if reachable < cap {
                        cap = reachable;
                    }
                }
            }
            // arcs[i] always passes its own mask, so the window is never empty;
            // the fallback is here because the python spells it out
            let room = room.unwrap_or(clr[i]);
            // The flat-min ramp, kept only as the feedforward's separator:
            // `vmax == max_speed` under the seed's governor was exactly
            // `room >= speed_clearance`, because the ramp is linear in the
            // clamped fraction and `max_speed > min_speed`.
            ramp_unlimited = room >= base.speed_clearance;
            vmax = cap.clamp(base.min_speed, base.max_speed);

            // -- PINCH ESCAPE: the lower anchor is not a constant
            //
            // The ramp's one lower anchor is asked to do two incompatible jobs.
            // With room to spare, a low anchor is what lets the clearance
            // annotation modulate speed at all and is what buys precision. With
            // the room gone it is a loss: what kills in a pinch is DWELL, since
            // the cross-track offset is a fixed distance and the time spent
            // inside decides whether it becomes a contact. Taken as a `max`
            // against the ramp, so the join is continuous and the leg can never
            // SLOW anything.
            let hi_here = arcs[i] + cfg.escape_preview;
            let mut room_here = clr[i];
            for (k, &a) in arcs.iter().enumerate() {
                if a >= arcs[i] && a <= hi_here && clr[k] < room_here {
                    room_here = clr[k];
                }
            }
            if cfg.escape_speed > base.min_speed && room_here < cfg.escape_clearance {
                let e = ((cfg.escape_clearance - room_here) / cfg.escape_clearance.max(1e-6))
                    .clamp(0.0, 1.0);
                let top = cfg.escape_speed.min(base.max_speed);
                vmax = vmax.max(base.min_speed + (top - base.min_speed) * e);
            }
        }
    }

    // -- the world-frame velocity request
    //
    // Aiming the whole velocity at the carrot drives down a CHORD, cutting the
    // inside of every corner by about L^2/(8R) -- a distance, not a lag, and the
    // follower's habitual cross-track drift. Splitting the two jobs removes it:
    // a FEEDFORWARD along the plan's tangent carries the body at the governor's
    // speed, and a proportional CORRECTION toward the foot of the perpendicular
    // is the only term answering to being off the line, so the error decays to
    // zero instead of settling at the chord's sagitta.
    let (wx, wy) = if rotating {
        // the fan branch is the seed's, gain included: hold the fan's own
        // position and let the yaw term do the work
        (
            base.k_pos * (path[i][0] - px),
            base.k_pos * (path[i][1] - py),
        )
    } else {
        let (tx, ty) = tangent_at(path, &arcs, s_star, cfg.tangent_preview);
        // terminal deceleration: the seed got this free from its carrot pinning
        // to the last waypoint, so reproduce it explicitly
        let v_ff = vmax.min(base.k_pos * (arcs[n - 1] - s_star).max(0.0));
        (
            v_ff * tx + base.k_pos * (foot[0] - px),
            v_ff * ty + base.k_pos * (foot[1] - py),
        )
    };

    // world -> body
    let (c, s_) = ((-pyaw).cos(), (-pyaw).sin());
    let (mut vx, mut vy) = (c * wx - s_ * wy, s_ * wx + c * wy);
    let speed = vx.hypot(vy);
    if speed > vmax {
        vx = vx / speed * vmax;
        vy = vy / speed * vmax;
    }

    // Feedforward: at `cmd_speed` along a plan turning `yaw_ff` rad per metre the
    // heading must slew at `yaw_ff * cmd_speed` just to stay aligned. Supplying
    // that open loop is what lets the proportional term settle at zero error.
    // Added inside the clamp, so the envelope still holds.
    //
    // SILENT in the one regime where it is an artefact: past `fan_yaw_per_m` --
    // the curve-vs-rotate CLASSIFIER, not a yaw-authority bound -- the value fed
    // forward is the threshold itself, which can demand more rate than the
    // ceiling allows. The sum clamp then goes FLAT in the heading error,
    // deleting the proportional term rather than attenuating it, and the heading
    // loop runs open. Gated on `ramp_unlimited`, not on `vmax >= max_speed`.
    let cmd_speed = vx.hypot(vy);
    let ff = if rotation_in_window && ramp_unlimited {
        0.0
    } else {
        yaw_ff * cmd_speed
    };
    // np.clip = minimum(maximum(v, lo), hi); spelled out rather than `clamp`,
    // which panics when a config sets a negative max_yaw_rate
    let wz = (base.k_yaw * angle_diff(target_yaw, pyaw) + ff)
        .max(-base.max_yaw_rate)
        .min(base.max_yaw_rate);
    (vx, vy, wz)
}
