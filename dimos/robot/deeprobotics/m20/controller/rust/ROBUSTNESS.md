# M20 controller: does any command make the robot freak out?

The Rust controller and its policies against the MuJoCo sim (`tests/robustness.rs`, lockstep, one tick of policy
latency), 2295 cases (2031 without height_gap):

- every gait code (17 + 0x1005 + 0x9999) to every other, standing still (306)
- every runner to every other (+ 0x9999) switched at the stop from walking fast / slow, backward, strafing, turning
  hard (each for 2.5 s; moving again 0.1 s after the switch), and with another switch 50 ms in flight (936)
- every code while lying, right as the 0x1001 stand-in hands over, and resent through the stand-in (57)
- stand, lie down, passive, damping, RL control, align (0xf003) from still / fast / backward / strafe / turn /
  mid-switch, every runner (432)
- A -> B -> A within 50 ms (132), stand / lie / RL spam every 0.1 s, gait spam every 50 ms (still and walking)
- axes at +-1, +-10, NaN, inf, a +1 -> -1 reversal, /REAL_STEER 100 m/s and NaN, pose params +-5 / NaN / inf, mode 5,
  still and walking (312)
- the operator falling silent while fast / turning / strafing (36); usage mode 1 -> 0 / 2 through basic_server's
  layer (48)

Each case starts standing on navi_charge (forced rl_control, 1 s), switches to its source gait, sets its start state
from 3 s, does the event at 5.5 s and is scored from then on. `M20_ROBUSTNESS_FULL=1` starts from lying instead
(stand-up, the 0x1001 stand-in, basic_flat, then the source gait); run on the gait-from-still, fast motion and
baseline cases (404) it agrees: 0 freak-outs.

## Freak-out

Scored in rl_control after the event: fell (tilt > 1 rad, or the trunk on the ground while held), a fault or joint
damping nobody asked for, a non-finite command, tilt > 30 deg, a leg joint within 0.01 rad of its stop, a leg target
jumping > 1.5 rad between ticks (the policies' own steps at the full command reach 1.2 rad; calm walking 0.76), a leg
torque-saturated 100 ms in a row, body rate > 6 rad/s (calm walking: 3.1). Baselines (every runner still and at
each moving start state's speed) are all calm except height_gap.

## Results (excluding height_gap)

| family | rl_deploy's behavior | ours |
|---|---|---|
| gait switches | 106 / 1052 | 2 / 1052 |
| motion commands | 11 / 396 | 5 / 396 |
| toggles / spam | 17 / 154 | 1 / 154 |
| extreme commands | 93 / 286 | 16 / 286 |
| stale, usage mode, baseline | 0 / 143 | 0 / 143 |
| **total** | **227 / 2031** | **24 / 2031** |

Every row (`zcat`): `ROBUSTNESS.tsv.gz` (ours), `ROBUSTNESS_vendor.tsv.gz` (rl_deploy's behavior: the controller
before these changes), `ROBUSTNESS_full.tsv.gz` (ours from lying). Rebuild: `cargo test --release --test robustness --
--ignored matrix --nocapture` (~15 min on 14 sims; M20_SIM_PYTHON, M20_ROBUSTNESS_FILTER, M20_ROBUSTNESS_FULL=1).

## What freaked out, why, and what we do instead

Each difference is a switch in `m20_controller.toml`; removing `[states.rl_control.safe_switch]` and setting
`finite_commands = false`, no `default_command_slew` and no `param_limits` gives rl_deploy's behavior back.

| what (rl_deploy) | why | ours (toml) |
|---|---|---|
| basic_flat -> navi_flat / navi_orienteer: 2-3.7 rad target jump, hr_hipy past its limit (the robot faulted there too) | basic_flat's tall stance is outside their training distribution | `safe_switch.bridge_pairs`: run `bridge` (the 1.5 s six_axis_stand stand-in rl_deploy runs before 0x3001) first |
| to or from basic_backknee: falls (-> navi_orienteer, step_move), 9-17 rad/s flails | its hind knees bend the other way; the handover swings them through | bridged (`["basic_backknee", "*"]`, `["*", "basic_backknee"]`) |
| switch to a stand gait (0x3001's stand-in, 0x3004, 0xf001, 0xf002, extre) at the stop from 1-2 m/s: 40 falls | rl_deploy switches once the *command* is still; the body is still rolling | `safe_switch.still`: the switch also waits for the wheels' mean rim speed < 0.1 m/s and body rate < 0.5 rad/s |
| every runner switch: 1.0-1.4 rad first-step jumps | the new runner's first targets replace the last ones at once | `safe_switch.blend_s`: 0.1 s linear blend from the last command |
| lie-down from 1-2 m/s: extre, navi_flat, basic_plat, step_move tipped over | rl_deploy lies down at once | `safe_switch.lie_down_when_still`: command zeroed, lies once still |
| a gait sent during a stand-in is lost; a 0x3001 between two gaits or a spam ending on one left the robot on navi_charge | rl_deploy's stand-in writes 0xf001 over the gait slot | `safe_switch.keep_gait_during_stand_in` |
| NaN axes / velocity: NaN joint targets, NaN fault into damping (falls on extre, step_move, the stand gaits) | the runner's clamp keeps NaN | `safety.finite_commands`: non-finite -> 0, axes clipped, non-finite params dropped |
| pose params out of range / NaN (0x3004, 0xf001, 0xf002), extre mode 5 / inf: falls | rl_deploy's pose inputs come from clipped axes; ours are params | `param_limits` per runner ([0, 1] height, [-1, 1] the rest, extre mode 0) |
| basic_plat axis +1 -> -1 reversal: 51 deg tip | basic_plat (and the others without one) has no command slew | `safety.default_command_slew` = basic_stair's (0.04, 0.04, 0.08) |

## Still freaking out (ours)

- joint damping while strafing at 0.8 (basic_flat, basic_plat, basic_stair): the body tips as it drops. Damping is
  the soft e-stop: left alone (another agent owns e-stop handling).
- step_move: a lie-down while it steps in place (still, strafing, turning) and stand / lie spam on it tip it over; a
  switch to 0x3004 at the stop from fast reaches 38 deg; navi_flat -> step_move after strafing jumps 1.7 rad.
  basic_plat lie-down right after strafing falls.
- all six pose inputs at their clamped maximum at once (0x3004, 0xf001, 0xf002): a hipx on its stop for 3 s, calm
  otherwise; extre and basic_flat at the full command (+-1 on every axis): 1.6-1.9 rad target steps, extre 120-270 ms
  torque-saturated. These are the policies at their own command limits.
- height_gap (0x5001): falls walking on its own in the sim (SPEC: the height-map convention is unknown), so its 264
  cases (83 still freak out, 154 before) say little about the robot. Every switch to or from it is bridged.

## Not covered

Terrain (all flat ground), payloads, IMU / joint noise and bus dropouts, the real robot's numbers for `still`, docking
during these events, recovery after a fall (align from a fallen pose), e-stop / HES, and the policies' own drift at
zero cmd_vel (not seen on flat sim ground in the offroad gaits' baselines; left to its own investigation).
