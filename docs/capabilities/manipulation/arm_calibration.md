# Calibrate a Damiao arm

`dimos arm calibrate` is a guided calibration of one Damiao arm. It fits a gravity model and kp/kd gains, and saves one file per arm that the arm loads at start.
It works with `openyam`, `dual-openyam`, `metal`, `dual-metal`, `b601-dm` and `openarm`. A step the arm's connection cannot do is skipped, and the plan says why.

## Run it

```bash
dimos arm calibrate dual-openyam --arm left --floor-z-m F --min-x-m X \
    --dualopenyamconnection.left-can-port follower_l \
    --dualopenyamconnection.right-can-port follower_r
```

Add `--dry-run` first. It prints the exact `dimos run` command and the steps, and starts nothing.

| Key | What happens |
|-----|--------------|
| Enter | Go on |
| s | Skip this pose, joint or step |
| q then Enter, or Ctrl-C | Abort. The arm holds where it is with its motors on |
| At the end | Enter (or h) goes home, then finishes. k finishes without going home |

Before you start:

- Clear the workspace. Stand beside the arm so you can catch it.
- OpenYAM arms start folded on their supports. The poses lift them off.
- Know the CAN port of every arm on the robot, including the one you are not calibrating.
- No other dimos run is active (`dimos status`).
- Every arm needs `--floor-z-m` and `--min-x-m` (metres from the base). On a two-arm robot each arm also stays on its side of the midline between the two bases in the URDF.

Other robots:

| Robot | Connection flags |
|-------|------------------|
| `openyam` | `--openyamconnection.can-port can0 --floor-z-m F --min-x-m X` |
| `metal` | `--metalconnection.can-port can0 --floor-z-m F --min-x-m X` |
| `dual-metal` | `--dualmetalconnection.left-can-port can0 --dualmetalconnection.right-can-port can1 --arm left --floor-z-m F --min-x-m X` |
| `b601-dm` | `--b601dmconnection.can-port can0 --floor-z-m F --min-x-m X` |
| `openarm` | `--openarmconnection.left-can-port can0 --openarmconnection.right-can-port can1 --arm left --floor-z-m F --min-x-m X` |

## What you get

The file is `~/.local/state/dimos/arms/<arm id>/calibration.json`, for example `~/.local/state/dimos/arms/dual_openyam/left/calibration.json`. The file it replaces goes to `history/` next to it.

```bash
dimos arm show dual-openyam --arm left       # path, gains, link masses, checks, history
dimos arm bake dual-openyam --arm left --urdf in.urdf --out calibrated.urdf
```

`bake` writes a copy of the URDF with the fitted masses and centres of mass.

The arm's connection reads the file at every start. Its `calibration` setting is `auto`, `off` or a path. For example, `--dualopenyamconnection.calibration off` runs without it.
If the file was made for a different arm or URDF (the URDF hash does not match), the connection refuses it. `bake` refuses such a URDF too.
Saving goes through the connection, which always writes `~/.local/state/dimos/arms/<arm id>/calibration.json`, even when it was started with `off` or a path.

## Options you may need

Pass them to the command as flags.

| Option | Meaning |
|--------|---------|
| `--arm` | `left` or `right` on a two-arm robot |
| `--poses N` | Number of gravity poses (default 15) |
| `--steps a,b,c` | Run only these steps |
| `hold_s` | Rest time at each pose (default 2.0 s) |
| `--speed-deg-s S` | Peak joint speed (default 10, at most 30) |
| `kp_scales`, `kd_scales` | Multiples of each joint's kp and kd the sweep tries, every kp with every kd (default 0.6, 1, 1.5 and 1, 2, 3.5; kd at most 5) |
| `--floor-z-m`, `--min-x-m` | Table height and the line the fingertip stays in front of |
| `--no-confirm-each` | Stop asking before every pose and joint. Not for a first run |

Anything else is passed on to `dimos run <robot>-calibrate`. That is how the CAN ports get there.

## How it works

The command starts the robot in the background, talks to it, brings the arm home, then stops that run. Every step asks before it moves.

1. **check.** Connection, joint readings and recorder work. Then move home.
2. **gravity_poses.** Rest at each pose, nudge each joint up and down, far enough to beat its friction.
3. **gravity_fit.** Fit link masses. Shows `URDF -> running -> new` error per joint. Enter saves and applies.
4. **float_check.** At each pose the arm rests, then one joint goes limp for about ten seconds. Be ready to support the arm.
5. **step_sweep.** Each joint steps at up to nine kp and kd pairs at the sweep pose. The prompt says how long it takes.
6. **pick_gains.** Shows the proposed gains and every rejected try. Enter saves and applies.
7. **confirm_tracking.** Revisit the held-out poses and compare rests before and after.
8. **summary.** Lists the file, the files it replaced and what to do next.

Saves happen with the arm at home. If applying a save would change a joint's gravity torque by more than 2 N m, the connection writes nothing and the wizard asks again: Enter forces it, s saves the file without applying it.

The method:

- Each joint is nudged up and down at every pose, by a fifth of its torque limit over its kp, 1 to 5 degrees. The pair cancels friction, so what is left is gravity. A pair where the joint moved less than half the nudge is stuck: it is left out and counted.
- The link masses are fitted by least squares. Held-out poses give the cross validation error. A joint that is never nudged keeps its URDF gravity torque.
- The fit is offered only when it is no worse than the URDF and the running file on every joint.
- The step sweep rejects a kp and kd pair when a step sticks (moves less than half its size), overshoots more than 20 % or buzzes. Of the rest it picks the lowest worst rest error over max(0.1 deg, 1.5 x friction / kp) plus worst settle time over 0.5 s. Each joint's friction comes from its 3 degree steps and its nudges.
- A joint where nothing passes keeps its robot gains.
- The float check sets kp to 0 for one joint at a time, at the poses where it carries the most gravity with the masses the arm runs. The fingertip drift counts from the moment the joint goes limp; more than 2 mm fails. A joint that drifts more than 5 degrees or moves faster than 20 degrees per second is held again and fails.

Safety:

- The arm moves only after you confirm.
- An abort never moves the arm. It holds in place and offers to go home.
- A joint that lags its target by more than 10 degrees stops the step. So does a fingertip that would cross the guard.
- `dimos stop` from another terminal gives the run about 5 seconds before it kills it. That is too short to park the arm, so finish through the wizard.

## How the wizard drives the arm

A `<robot>-calibrate` blueprint runs three modules: the arm's connection, a control coordinator and the calibration module.

The calibration module publishes the arm's targets 50 times a second on `calibration_joint_command`. The coordinator's pass-through task sends each one to the connection's `position_command` on its next tick, unchanged: nothing limits, interpolates or smooths it, so a step stays a step. When no target has come for 0.1 s, or the wizard asks it to `hold`, the coordinator sends the arm nothing, and 0.1 s later the connection's deadman holds the arm where it is. If the wizard stops, the arm is held within about 0.2 s.

The wizard changes how the arm is driven through the config contract, four calls every connection has. A patch names a target, the arm id, and values whose keys the connection defines; an arm's keys are those of `ArmConfigPatch` in `dimos/control/calibration/schema.py`:

| Call | What it does |
|------|--------------|
| `config_keys` | Says what the connection can read and change. Steps it cannot do are skipped |
| `config_get` | How each arm is driven now: gains, floating joints, link masses and the file it runs |
| `config_set` | Changes gains or floats a joint for the session, or saves to the arm's file (`persist`). An empty patch only renews the session's lease |
| `config_reset` | Undoes every live change and ends the session |

Only a connection started by a `<robot>-calibrate` blueprint changes anything (`config_sessions`). It takes position commands during a session as at any other time. The session has a 2 second lease, which the wizard renews twice a second with an empty `config_set` carrying its id; reading the arm's state does not renew it. When the session ends, because the wizard finished, aborted or stopped, or the robot halted or stopped, the connection undoes every live change: floating joints hold where they are and the gains go back to the file's. The wizard has the coordinator hold the arm first, so the last target does not pull it back.

The joint readings come from the connection's `joint_state` stream. The commands the arm was sent are recorded as `arm_position_command`, and each step is timed from the first command that carries its target.
