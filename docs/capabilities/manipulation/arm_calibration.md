# Calibrate a Damiao arm

`dimos arm calibrate` is a guided calibration of one Damiao arm. It fits a gravity model and kp/kd gains, and saves one file per arm that the arm loads at start.
It works with `openyam`, `dual-openyam`, `metal`, `dual-metal`, `b601-dm` and `openarm`. A step the arm's connection cannot do is skipped, and the plan says why.

## Run it

```bash
dimos arm calibrate dual-openyam --arm left \
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
- Arms other than the dual OpenYAM bench need `--floor-z-m` and `--min-x-m` (metres from the base).

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
| `kp_scales` | Multiples of each joint's kp the sweep tries (default 1.0, 1.5, 2.0) |
| `--floor-z-m`, `--min-x-m` | Table height and the line the fingertip stays in front of |
| `--no-confirm-each` | Stop asking before every pose and joint. Not for a first run |

Anything else is passed on to `dimos run <robot>-calibrate`. That is how the CAN ports get there.

## How it works

The command starts the robot in the background, talks to it, brings the arm home, then stops that run. Every step asks before it moves.

1. **check.** Connection, joint readings and recorder work. Then move home.
2. **gravity_poses.** Rest at each pose, nudge each joint up and down.
3. **gravity_fit.** Fit link masses. Shows `URDF -> running -> new` error per joint. Enter saves and applies.
4. **float_check.** Each joint goes limp for about ten seconds. Be ready to support the arm.
5. **step_sweep.** Each joint steps at several stiffnesses at the sweep pose.
6. **pick_gains.** Shows the proposed gains and every rejected try. Enter saves and applies.
7. **confirm_tracking.** Revisit the held-out poses and compare rests before and after.
8. **summary.** Lists the file, the files it replaced and what to do next.

Saves happen with the arm at home. If applying a save would change a joint's gravity torque by more than 2 N m, the connection writes nothing and the wizard asks again: Enter forces it, s saves the file without applying it.

The method:

- Each joint is nudged up and down at every pose. The pair cancels friction, so what is left is gravity.
- The link masses are fitted by least squares. Held-out poses give the cross validation error.
- The fit is offered only when it is no worse than the URDF and the running file on every joint.
- The step sweep picks the stiffest kp and kd that settle without overshoot or buzz, and only when every softer one passed too. kd grows with the square root of the kp multiple.
- A joint where nothing passes keeps its robot gains.
- The float check sets kp to 0 for one joint at a time. It fails the joint if it drifts more than 5 degrees or moves faster than 20 degrees per second.

Safety:

- The arm moves only after you confirm.
- An abort never moves the arm. It holds in place and offers to go home.
- A joint that lags its target by more than 10 degrees stops the step. So does a fingertip that would cross the guard.
- `dimos stop` from another terminal gives the run about 5 seconds before it kills it. That is too short to park the arm, so finish through the wizard.

## How the wizard drives the arm

The calibration module talks to the arm's connection through the arm config contract, five calls every connection has:

| Call | What it does |
|------|--------------|
| `config_keys` | Says what the connection can read and change. Steps it cannot do are skipped |
| `config_get` | How each arm is driven now: gains, floating joints, link masses and the file it runs |
| `config_set` | Changes gains or floats a joint for the session, or saves to the arm's file (`persist`) |
| `config_command` | Sends the arm's targets, 50 times a second |
| `config_reset` | Undoes every live change and ends the session |

Only a connection started by a `<robot>-calibrate` blueprint changes anything (`config_sessions`). While the wizard's session lasts, the connection drops every other position command. The session has a 2 second lease that every call renews. When it ends, because the wizard finished, aborted or stopped calling, or the robot halted or stopped, the connection undoes every live change: floating joints hold where they are and the gains go back to the file's. The joint readings come from the connection's `joint_state` stream.
