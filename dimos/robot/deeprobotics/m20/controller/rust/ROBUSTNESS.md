# M20 controller: does any command make the robot freak out?

**0 of 2523 sim cases freak out with ours (rl_deploy's behavior: 265).** The Rust controller and its policies against
the MuJoCo sim (`tests/robustness.rs`, lockstep, one tick of policy latency):

- every gait code (17 + 0x1005 + 0x9999) to every other, standing still (306)
- every runner to every other (+ 0x9999) switched at the stop from walking fast / slow, backward, strafing, turning
  hard (2.5 s each; moving again 0.1 s after the switch), and with another switch 50 ms in flight (936)
- every code while lying, right as the 0x1001 stand-in hands over, and resent through the stand-in (57)
- stand, lie down, passive, damping, RL control, align (0xf003) from still / fast / backward / strafe / turn /
  mid-switch, every runner (432)
- A -> B -> A within 50 ms (132), stand / lie / RL spam every 0.1 s, gait spam every 50 ms (still and walking)
- axes at +-1, +-10, NaN, inf, a +1 -> -1 reversal, /REAL_STEER 100 m/s and NaN, pose params +-5 / NaN / inf, mode 5,
  each pose input alone at either end (348)
- a 10 and a 15 deg slope, facing up and across: standing, lie-down, damping, a switch, every runner (192)
- the operator falling silent while fast / turning / strafing (36); usage mode 1 -> 0 / 2 through basic_server's
  layer (48)

Each case starts standing on navi_charge (forced rl_control, 1 s), switches to its source gait, sets its start state
from 3 s, does the event at 5.5 s and is scored from then on. `M20_ROBUSTNESS_FULL=1` starts from lying instead
(stand-up, the 0x1001 stand-in, basic_flat, then the source gait): 0 of 1194 (gait from still, every motion command,
baselines, slopes, extremes standing).

## Freak-out

Scored in rl_control after the event: fell (tilt > 1 rad, or the trunk on the ground while held), a fault or joint
damping nobody asked for, a non-finite command, tilt > 30 deg, a leg joint within 0.01 rad of its stop, a leg target
jumping > 1.5 rad between ticks (calm walking: up to 0.76), a leg torque-saturated 100 ms in a row, body rate
> 6 rad/s (calm walking: 3.1), and a lie-down or damping that never lies down. Every baseline (each runner still and
at each moving start state's speed) is calm.

## Results

| family | rl_deploy's behavior | ours |
|---|---|---|
| gait switches | 133 / 1227 | 0 / 1227 |
| extreme commands | 100 / 348 | 0 / 348 |
| toggles / spam | 19 / 168 | 0 / 168 |
| motion commands | 11 / 432 | 0 / 432 |
| slopes | 2 / 192 | 0 / 192 |
| stale, usage mode, baseline | 0 / 156 | 0 / 156 |
| **total** | **265 / 2523** | **0 / 2523** |

Every row (`zcat`): `ROBUSTNESS.tsv.gz` (ours), `ROBUSTNESS_vendor.tsv.gz` (`M20_ROBUSTNESS_VENDOR=1`,
Config::rl_deploy), `ROBUSTNESS_full.tsv.gz` (ours from lying). Rebuild: `cargo test --release --test robustness --
--ignored matrix --nocapture` (~17 min on 14 sims; M20_SIM_PYTHON, M20_ROBUSTNESS_FILTER).

## What freaked out, why, and what we do instead

Each is a switch in `m20_controller.toml` defaulting to ours; `Config::rl_deploy()` turns them all off (the recording
replays and the golden test run on it).

| what (rl_deploy) | why | ours (toml) |
|---|---|---|
| basic_flat -> navi_flat / navi_orienteer: 2-3.7 rad target jump, hr_hipy past its limit (the robot faulted there too) | basic_flat's tall stance is outside their training distribution | `safe_switch.bridge_pairs`: run `bridge` (the 1.5 s six_axis_stand stand-in rl_deploy runs before 0x3001) first |
| to / from basic_backknee and height_gap, to step_move: falls, 9-17 rad/s flails, 1.6 rad jumps | backknee's hind knees bend the other way; height_gap's stance is far from the rest; step_move steps from its first step | bridged |
| a switch to a stand gait at the stop from 1-2 m/s: 40 falls | rl_deploy switches once the *command* is still; the body still rolls | `safe_switch.still` (wheels' mean rim speed, body rate), or `settle_below_m_s` / `settle_decel_m_s2`: the wheels no longer slowing under 0.3 m/s (a slope, a drift) |
| every runner switch: 1.0-1.4 rad first-step jumps | the new runner's first targets replace the last at once | `safe_switch.blend_s` 0.1 s |
| lie-down from 1-2 m/s or mid-step (extre, navi_flat, basic_plat, step_move) tipped over | rl_deploy lies down at once | `lie_down_when_still`: command zeroed, lies once settled with the legs under `lie_leg_rad_s`; step_move (`lie_bridge_from`) stands on the bridge first |
| joint damping while strafing tipped basic_flat / plat / stair over | damping drops the legs at once | `joint_damping.lie_down_first(_on_command)`: the operator's damping and every fault the motors can act on stop and lie down first, then damp; a driver status-word fault damps at once; the hardware e-stop is not this path |
| a gait sent during a stand-in is lost | the stand-in writes 0xf001 over the gait slot | `safe_switch.keep_gait_during_stand_in` |
| NaN axes / velocity: NaN targets, NaN fault | the runner's clamp keeps NaN | `safety.finite_commands` |
| pose params out of range / NaN, extre mode 5; all pose inputs at +-1 together: hipx on its stop 3 s | ours are params, not clipped axes | `param_limits`, `param_budget` (roll, pitch, yaw, dx, dy share one range) |
| extre / basic_flat / step_move at the full axes: 1.6-1.9 rad steps, saturation; basic_plat's +1 -> -1 reversal tipped 51 deg | command_max up to 3.9 m/s; no slew on most runners | `safety.command_limit` (basic_server.toml's 2.0 / 1.0 / 1.0), step_move and height_gap tighter; `default_command_slew` = basic_stair's |

## height_gap

Its height map was the sim's guess (it fell walking on its own). rl_deploy's HeightsSubCallback (0x176bf0) and the
EcanHardwareInterface ctor (0x1801c8) give the convention: h = terrain relative to the base, v = -(h + 0.46), a gap
(v > 0.18) in rows 0-19 is 0.6, the last row repeats, out column by column from the last (SPEC "HeightGap height
map"). With it height_gap stands and walks in every baseline; its 280 cases are all calm. Still [U]: the grid spacing
and which way the rows run (the sim's 0.05 m, row 0 in front).

## Drift

At a zero command the policies' wheel targets carry a common-mode bias, so rl_deploy's robot creeps (sim: 0.84 m in
15 s on navi_orienteer; basic_stair +0.04 m/s, basic_backknee +0.09). Ours (`[states.rl_control.zero_command]
cancel_wheel_drift`): while the operator's command and the runner's ramped command are both zero the four wheel
targets' mean is subtracted, the differential kept for balance (`tests/drift.rs`): 15 s still on 0x3003 / 0x3005 /
0x1004 moves 1.6-18 mm (under 1 mm/s); a 0.2 m/s shove rolls 3 cm and stops; after a 0.5 m/s walk the hold moves 6 mm;
a command from rest starts as rl_deploy's does (within 5 mm/s at every instant). The wheel position hold (`hold_kp`)
is off: in the sim at kp 3 standing on a 10-15 deg slope facing up it moved 2-4 cm where none moved 1 cm without it;
across a 15 deg slope it cut basic_stair's 18 cm slide to 3 cm (navi_flat 6 -> 0.5 cm) (`slope_hold_probe`).

## Not covered

Steps, stairs and gaps (all flat or sloped ground), payloads, IMU / joint noise and bus dropouts, the real robot's
numbers for `still` and `settle_*`, docking during these events, recovery after a fall, the hardware e-stop, and the
policies' own drift at zero cmd_vel across a slope (basic_stair slides 18 cm in 15 s across 15 deg).
