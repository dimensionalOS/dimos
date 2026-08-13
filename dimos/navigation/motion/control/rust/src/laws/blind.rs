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

//! The blind track's law: follow the path the gait can actually walk.
//!
//! From `motion-tc-autoresearch` branch `blind_research01` (evo exp_0013),
//! which took the blind track from 98.89 to 110.77 on the reality-mode
//! battery -- 21 timeouts and 4 collisions became arrivals, 228/228 episodes
//! arrive, zero collisions.
//!
//! Three mechanisms over the seed, in order of what they are worth:
//!
//! 1. `walk_command`, the gait slip inverse. The twist is a REQUEST to a
//!    learned policy that under-delivers it, and the governor's creep rung sat
//!    inside the gait's dead-stall band.
//! 2. Constant time headway: the carrot distance scales with commanded speed
//!    instead of staying fixed, so the pursuit chord shrinks where the plan is
//!    tight.
//! 3. The stamped precision profile as the governor's input, since the
//!    clearance array is exactly what this track does not get.
//!
//! Only (3) is blind-specific. (1) and (2) are held here rather than shared
//! deliberately: the hinted track is researched independently, and handing its
//! lab a mechanism biases that search.

use crate::geom::{
    arcs_of, body_error, carrot_lerp, clearance_governor, fan_target, progress_index, yaw_command,
    Params,
};
use crate::stamps::{ceiling_ahead, decode_ceilings};

/// Locomotion feed-forward, in m/s -- the default calibration of
/// `ml-trajectory-research/freewalk_mcf.bin`.
///
/// The twist this law emits is not a velocity, it is a *request* to a learned
/// walking policy -- and that policy under-delivers. Measured open loop by
/// `control/probe_walk_slip.py` (flat empty world, constant command held 12 s,
/// body pose differenced after the 3 s settle):
///
/// ```text
///   cmd  0.20  0.25  0.30  0.35  0.40  0.50  0.65  0.80  1.00
///   got  0.002 0.036 0.130 0.217 0.275 0.388 0.532 0.707 0.932
/// ```
///
/// Two things fall out of that table.
///
/// Above cmd ~0.32 the map is affine and well behaved: a least-squares fit
/// over cmd >= 0.35 gives `got ~= 1.10*cmd - 0.168`, i.e. a deficit of
/// 0.11-0.13 m/s across the whole working range. The same probe run at 45 and
/// 90 degrees of heading and with |wz| up to 0.5 rad/s returns the same
/// deficit to within 0.02 -- it is gait slip, not a direction artefact.
///
/// Below cmd ~0.30 the gait does not initiate at all. A 0.20 m/s request moves
/// the body 0.002 m/s: the robot marches on the spot, and it wobbles doing it
/// (tilt p99 0.09-0.16 through the stall band against 0.06-0.07 at cruise).
///
/// `min_speed` is 0.20. So the creep rung of the clearance governor -- the
/// speed the plan's own precision profile asks for in a tight room -- sits
/// inside the stall band, and asking for it stops the robot dead. Executed
/// traces show it plainly: an episode commanding 0.207 m/s for 39 s of a 40 s
/// horizon covered 0.88 m of ground and timed out having never come within
/// 0.45 m of a wall. That is the clock the timeouts lose to. Not a follower
/// that drives too carefully -- one whose careful speeds are not speeds.
///
/// The correction is an actuator inverse, not a speed increase. `WALK_GAIN`
/// and `WALK_SLIP` are the inverse of that affine fit taken over the reachable
/// command range only -- cmd 0.35 to 0.65, which is what `want <= max_speed`
/// can ask for -- so the *intended* ground speed is still exactly the one the
/// governor chose and the correction cancels rather than exceeds the deficit.
/// Deliberately not a flat offset: `want + 0.15` matches the inverse near the
/// governor floor but overshoots it by ~9% at cruise, and a follower that
/// quietly runs 9% over the speed its own clearance annotation licensed is not
/// solving the stall, it is just driving faster.
///
/// At the floor the inverse errs slightly careful (`want = 0.20` asks 0.325,
/// worth about 0.18 m/s rather than 0.20, because the fit's linearity is
/// giving out as the stall band approaches). Erring careful in the tight rooms
/// is the right direction for the error to point.
///
/// DEPLOY. These are properties of the policy blob, not of the law: on a
/// different gait the 0.614 command becomes a ~23% over-speed. They are config
/// fields (`BlindControllerConfig`) for exactly that reason -- re-run
/// `control/probe_walk_slip.py` against the deployed gait and key them to it
/// before this drives hardware.
pub const WALK_GAIN: f64 = 0.964;
pub const WALK_SLIP: f64 = 0.132;

/// Intended speeds below this get a proportionally smaller share of the
/// correction, reaching zero with it. A stop request has to remain a stop.
pub const SLIP_RAMP: f64 = 0.08;

/// `Params` plus the gait calibration this law feeds forward through.
pub struct BlindParams {
    pub base: Params,
    pub walk_gain: f64,
    pub walk_slip: f64,
    pub slip_ramp: f64,
}

impl Default for BlindParams {
    fn default() -> Self {
        Self {
            base: Params::default(),
            walk_gain: WALK_GAIN,
            walk_slip: WALK_SLIP,
            slip_ramp: SLIP_RAMP,
        }
    }
}

/// The command that asks the gait for a ground speed of `want` m/s.
///
/// Identity at `want = 0`; the affine inverse above once `want >= ramp`.
#[inline]
pub fn walk_command(want: f64, gain: f64, slip: f64, ramp: f64) -> f64 {
    if want <= 0.0 {
        return 0.0;
    }
    let correction = (gain * want + slip) - want;
    want + correction * (want / ramp).min(1.0)
}

/// One controller tick. `clearance` is the optional per-waypoint room
/// annotation and `ts` the optional per-waypoint timestamp, which carries the
/// same information in the dialect that survives a blind track (see
/// `stamps::decode_ceilings`). Returns `(vx, vy, wz)` in the body frame.
pub fn update(
    pose: (f64, f64, f64),
    path: &[[f64; 3]],
    clearance: Option<&[f64]>,
    ts: Option<&[f64]>,
    cfg: &BlindParams,
) -> (f64, f64, f64) {
    if path.len() < 2 {
        // empty path or a single-pose veto stub: there is nothing to
        // follow -- hold position (the planner is saying "stop")
        return (0.0, 0.0, 0.0);
    }
    let base = &cfg.base;
    let (px, py, pyaw) = pose;
    let arcs = arcs_of(path);
    let i = progress_index(path, &arcs, px, py, pyaw);

    // Speed governor, hoisted above target selection because the lookahead
    // distance is now derived from it (constant time headway, below).
    //
    // THE SAME GOVERNOR, OFF THE STAMPS when no clearance array arrives: the
    // room ahead has not been withheld, only re-encoded, since the planner
    // stamps the required-precision profile into the path's own timestamps on
    // every replan, blind track or not. Decoding it puts a speed ceiling back
    // under this law at exactly the point the clearance branch occupies, so
    // the two channels are alternatives rather than layers.
    let vmax = clearance_governor(&arcs, i, clearance, base)
        .or_else(|| {
            let ceilings = decode_ceilings(ts?, path, base)?;
            Some(ceiling_ahead(&ceilings, &arcs, i, base))
        })
        .unwrap_or(base.max_speed);

    // CONSTANT TIME HEADWAY. The seed pursues a point a fixed 0.35 m along the
    // plan whatever the speed. Steering straight at a carrot that far away
    // chords the plan's curvature, and the chord always falls to the INSIDE of
    // the turn -- i.e. toward the very obstacle the planner curved around. The
    // steady-state inset grows with the square of the carrot distance, so in a
    // passage whose planned clearance is a few centimetres it is the whole
    // error budget.
    //
    // The fix is to hold the *time* headway constant instead of the distance:
    // the follower always looks cfg.lookahead / cfg.max_speed seconds ahead
    // (0.7 s at the defaults). At full cruise this is exactly the seed's
    // 0.35 m -- open rooms are untouched -- and at the governor floor it is
    // 0.14 m, cutting the inward chord by ~6x precisely where the plan has no
    // room to give.
    //
    // This costs no speed. The command magnitude is min(k_pos * L, vmax), so
    // any L >= vmax / k_pos still saturates at vmax; the floor below enforces
    // that explicitly so an odd config cannot turn a shorter carrot into a
    // slower robot. Trading collisions for timeouts is not the deal here.
    let headway = base.lookahead / base.max_speed.max(1e-6);
    let look = (vmax * headway).max(vmax / base.k_pos.abs().max(1e-6));

    let (target_xy, target_yaw) =
        fan_target(path, &arcs, i, pyaw, base).unwrap_or_else(|| carrot_lerp(path, &arcs, i, look));

    // body-frame error -> velocity
    let (bx, by) = body_error(px, py, pyaw, target_xy);
    let (mut vx, mut vy) = (base.k_pos * bx, base.k_pos * by);
    let speed = vx.hypot(vy);
    if speed > 1e-12 {
        // `want` is the intended GROUND speed -- the pursuit gain, capped by
        // the governor. Unchanged from the seed.
        let want = speed.min(vmax);
        // ...and this is what the gait has to be asked for to deliver it.
        let cmd = walk_command(want, cfg.walk_gain, cfg.walk_slip, cfg.slip_ramp);
        vx = vx / speed * cmd;
        vy = vy / speed * cmd;
    }
    (vx, vy, yaw_command(target_yaw, pyaw, base))
}
