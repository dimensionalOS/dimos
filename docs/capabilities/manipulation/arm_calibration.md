# Calibrate a Damiao arm

`dimos arm calibrate` walks you through calibrating one arm of an OpenYAM,
Metal, B601-DM or OpenArm robot, one step at a time, in your terminal. It
measures what the model gets wrong and saves the corrections to a file that
the robot loads every time it starts.

## What it does

- **Gravity.** The URDF masses and centres of mass are never exactly right (a
  camera, a bracket, a heavier forearm). The arm visits a set of poses, nudges
  each joint a little, and the program fits the link masses that explain the
  torque it needed. Gravity compensation then holds the arm up more evenly.
- **Gains.** Every joint steps a degree or three at several stiffnesses. The
  program proposes, per joint, the stiffest kp and kd whose steps settle without
  overshoot or buzz. A joint where nothing passes keeps its robot gains.
- **Checks.** A float check lets one joint at a time go limp and measures how far
  the fingertip drifts, and a tracking check compares how closely the arm follows
  a few random poses before and after.

Every step asks first. Nothing moves until you press Enter.

## Before you start

- The arm has a clear workspace: nothing on the table inside its reach, nobody
  standing where it swings. Stand beside the arm, within reach of the posture it
  floats at, so you can catch it.
- OpenYAM arms start folded on their supports. The poses lift the arm off them.
- You know the CAN interface (or USB-CAN serial port) of each arm. The connection
  needs every one of its ports, including the arm you are not calibrating, which
  is held where it is.
- No other dimos run is active. The calibration owns the arm and refuses to start
  next to someone else's run (`dimos status` shows it).
- The robot has a workspace guard: the table height (`--floor-z-m`) and the line
  the fingertip stays in front of (`--min-x-m`), metres from the arm's base.
  The dual OpenYAM bench has its own. Every other arm needs both flags, and the
  command stops with the reason when they are missing.

## The command

```bash
dimos arm calibrate dual-openyam --arm left \
    --dualopenyamconnection.left-can-port follower_l \
    --dualopenyamconnection.right-can-port follower_r
```

Add `--dry-run` first: it prints the exact `dimos run` command and the steps and
starts nothing.

Options: `--arm` (left or right on a two-arm robot), `--floor-z-m` and
`--min-x-m`, `--poses N` (default 8), `--steps a,b,c` (run only these),
`--speed-deg-s S` (peak joint speed, at most 30), `--no-confirm-each` (stop
asking before every single pose and every joint of the sweep; not recommended
for a first run). Everything the command does not know is passed on to
`dimos run <robot>-calibrate`; that is how the CAN ports get there.

| Robot | Connection flags |
|-------|------------------|
| `openyam` | `--openyamconnection.can-port can0 --floor-z-m F --min-x-m X` |
| `dual-openyam` | `--dualopenyamconnection.left-can-port can0 --dualopenyamconnection.right-can-port can1` |
| `metal` | `--metalconnection.can-port can0 --floor-z-m F --min-x-m X` |
| `dual-metal` | `--dualmetalconnection.left-can-port can0 --dualmetalconnection.right-can-port can1 --arm left --floor-z-m F --min-x-m X` |
| `b601-dm` | `--b601dmconnection.can-port can0 --floor-z-m F --min-x-m X` |
| `openarm` | `--openarmconnection.left-can-port can0 --openarmconnection.right-can-port can1 --arm left --floor-z-m F --min-x-m X` |

The command starts the robot in the background, talks to it, and stops that run
when it ends. It never stops a run it did not start.

## The eight steps

At every prompt: **Enter** goes on, **s** skips what the prompt asks for (one
pose, one joint, or the rest of the step), **q** or Ctrl-C aborts.

1. **check.** Confirms the connection works and the joints read sane values, then
   asks to move home. Enter to go home, or `s` to stay where you are.
2. **gravity_poses.** Asks once, then asks before every pose. At each pose the arm
   rests, then each joint nudges up and down. Keep clear.
3. **gravity_fit.** Fits the masses and shows the hold-out error before and after
   for each joint, plus any link at a bound. Enter saves and applies it. If the
   fit looks wrong, `s` leaves the old file in place.
4. **float_check.** The arm moves to the first gravity pose, not home, because at
   a folded rest pose nothing can drift. Then each joint goes limp in turn for
   about ten seconds: be ready to support the arm. A joint that drifts is held
   again at once and fails.
5. **step_sweep.** Each joint steps at several stiffnesses, asking before each
   joint.
6. **pick_gains.** Shows the proposed gains with the reason for each, and every
   rejected try. Enter saves and applies them.
7. **confirm_tracking.** The arm visits a few random poses (asking before each)
   and reports its tracking error against the same measure from step 2.
8. **summary.** Lists the file, the files it replaced and what to do next.

Steps that need the run's recording (gravity_poses, gravity_fit, step_sweep,
pick_gains) are listed as unavailable by the plan when no recording is on; the
walk-through's own robot always records.

After the last step, or after a failure or an abort, the arm holds where it is.
Press **h** to ramp it home (it asks first) or Enter to finish.

## Safety

- The arm only moves after you confirm, pose by pose unless you gave
  `--no-confirm-each`.
- **q**, Ctrl-C or closing the terminal input aborts: no new targets are sent and
  the arm holds where it is. It is never moved home by an abort. Support it if a
  joint was carrying a heavy load.
- A joint that lags its target by more than 10 degrees, or a fingertip that would
  cross the guard, stops the step and holds the arm.
- Ramps are limited to 30 degrees per second.

## Where the files go

The calibration is saved at
`~/.local/state/dimos/arms/<arm id>/calibration.json`, for example
`~/.local/state/dimos/arms/dual_openyam/left/calibration.json`. Each time it is
replaced, the old file is copied to `history/` next to it, named by its creation
time. The robot's connection reads the file at every start (`calibration: auto`)
and refuses one that was made for a different arm or a different URDF.

## Inspect, undo, re-run

```bash
dimos arm show dual-openyam            # every arm of the robot; "none" when missing
dimos arm show dual-openyam --arm left
```

It prints the path, when it was made, the model it belongs to, the gains, the
link masses, the checks and the history.

To run without the calibration, start the robot with
`--dualopenyamconnection.calibration off` (any robot: its connection's
`calibration` option), or delete the file; the earlier one is in `history/`.
To calibrate again, run the command again; the file it replaces goes to `history/`.

## Bake the masses into a URDF

```bash
dimos arm bake dual-openyam --arm left --urdf in.urdf --out calibrated.urdf
```

Writes a copy of the URDF whose link masses and centres of mass are the fitted
ones, for tools that read a URDF and not this file. It refuses a URDF that is not
the one the calibration was made against.
