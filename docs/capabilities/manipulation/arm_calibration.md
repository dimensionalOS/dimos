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
  overshoot or buzz, and only when every softer stiffness passed too. A joint
  where nothing passes keeps its robot gains.
- **Checks.** A float check lets one joint at a time go limp where it carries
  the most weight and measures how far the fingertip drifts, and a tracking
  check revisits the poses the fit never saw and compares how closely the arm
  rests there before and after.

Every step asks first. Nothing moves until you press Enter. The one exception is
parking, below: when the calibration stops while the arm is away from home, it
backs the arm out home on its own, slowly and within a few seconds.

## Before you start

- The arm has a clear workspace: nothing on the table inside its reach, nobody
  standing where it swings. Stand beside the arm, within reach of the posture it
  floats at, so you can catch it.
- OpenYAM arms start folded on their supports. The poses lift the arm off them.
- You know the CAN interface (or USB-CAN serial port) of each arm. The connection
  needs every one of its ports, including the arm you are not calibrating, which
  is held where it is.
- No other dimos run is active. The calibration owns the arm and refuses to start
  next to another run: one in your run registry (`dimos status` shows it), any
  dimos coordinator answering on the bus (a run still starting, or another
  user's), or another `dimos arm calibrate` of the same robot.
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
`--min-x-m`, `--poses N` (default 15), `--steps a,b,c` (run only these),
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

The command starts the robot in the background, talks to it, brings the arm home
and only then stops that run (see Finishing, below). Global options given before
`arm`, such as `--transport` or `--zenoh-connect`, go to the run too, and the
command talks to the run on the same bus. While it runs it holds a lock,
`~/.local/state/dimos/arms/<robot>.calibrate.lock`, so a second walk-through of
the same robot cannot start next to it.

It signals only the run it started. It knows that run by the `Run ID:` line
`dimos run` prints, and it starts `dimos run` with a token of its own in the
environment. Before it stops the run, it checks that the run's process still
carries that token and started after the launch; a registry entry that names any
other process is reported, not signalled. If the launch fails, times out or is
interrupted, it stops what that launch left running, again only processes that
carry its token.

## The eight steps

At every prompt: **Enter** goes on, **s** skips what the prompt asks for (one
pose, one joint, or the rest of the step). At any time, not only at a prompt,
**q** then Enter aborts. So do **p** (the older calibration scripts' "park and
quit") and **Ctrl-C**. Closing the terminal window sends SIGHUP, and that aborts
too, as does closing the input. An abort stops sending new targets: the arm
holds where it is with its motors on. The abort does not move it home; going
home is offered next (see Finishing).

1. **check.** Confirms the connection works, the joints read sane values and the
   recorder is writing (a test marker must land in the recording within two
   seconds, or the step fails before anything moves), then asks to move home.
   Enter to go home, or `s` to stay where you are.
2. **gravity_poses.** Asks once, then asks before every pose. At each pose the arm
   rests, then each joint nudges up and down. Keep clear.
3. **gravity_fit.** Fits the masses and shows, per joint, the hold-out error of
   the URDF, of the file the arm runs now, and of the new fit
   (`URDF -> running -> new`), plus any link at a bound. The fit is offered only
   when it is no worse than both on every joint. Enter saves and applies it; `s`
   leaves the old file in place. If the file on disk holds other masses than the
   arm runs (a fit saved earlier but never applied), the step does not fit and
   says so: restart the robot or apply the file first, then run it again.
4. **float_check.** Each joint goes limp at up to two gravity poses where it
   carries the most weight (and is at least 5 degrees from its limits), for about
   ten seconds each. It asks before every move: be ready to support the arm. A
   joint that carries almost nothing at every pose (under 0.2 Nm) is skipped as
   "no gravity load to test". A joint that drifts more than 5 degrees, or moves
   faster than 20 degrees per second, is held again at once and fails.
5. **step_sweep.** The arm moves to the sweep pose, the gravity pose farthest
   from every joint limit (home sits on a limit on several arms), and each joint
   steps at several stiffnesses, asking before each joint; kd grows with the
   square root of the kp multiple. Then the arm goes home.
6. **pick_gains.** Shows the proposed gains with the reason for each, and every
   rejected try. Enter saves and applies them.
7. **confirm_tracking.** The arm revisits the poses the fit held out (asking
   before each) and reports how closely it rests there, against its rests at the
   same poses in step 2.
8. **summary.** Lists the file, the files it replaced and what to do next.

Steps that need the run's recording (gravity_poses, gravity_fit, step_sweep,
pick_gains) are listed as unavailable by the plan when no recording is on; the
walk-through's own robot always records.

## Finishing

After the last step, a failure or an abort, the arm holds where it is with its
motors on, and the command asks:

```
[Enter or h] go home, then finish   [k] finish without going home
```

**Enter** (or **h**) ramps the arm home. The calibration asks once more before
it moves; Enter to go. The command stops its run only once the arm is at home.

If going home fails or is refused, or you skip it or press **k**, it asks:

```
Stop anyway: the arm will go limp where it is. Are you supporting it or is it on its supports? [yes/No]
```

Only `yes` stops the run; anything else goes back to the offer to go home. Even
after `yes`, stopping still tries the backstop park (Parking, below), so the arm
may move toward home on its own for a few seconds before it goes limp. If
nobody can answer (the input or the terminal was closed, or Ctrl-C at these
prompts), the command leaves the run running, with the arm held where it is,
and says so. Support the arm, then stop the run with `dimos stop`.

## Parking

Two things move the arm home without asking, at up to 20 degrees per second and
within 8 seconds:

- **Stopping the run** (`dimos stop`, or the command stopping it after your
  `yes`). The arm's connection would otherwise switch the motors off where the
  arm is, and a stalled arm at a pose would drop onto the table. The
  calibration first aborts what it was doing and backs the arm out home; the
  connection waits up to 10 seconds for that before it switches the motors off.
- **A prompt left unanswered for 10 minutes.** The run aborts and the arm parks.

The way home is the same guarded path every ramp uses: straight, or through
home when straight would come too close to the table. When there is no such
path, or it would take longer than 8 seconds (a pose more than about 85 degrees
from home), the arm is **not** moved: it holds where it is, the log says why in
an error line, and it drops when the connection stops. The status's `park`
field says what the last park did.

Parking on stop is a backstop, not the way to end a session: `dimos stop` gives
the run's processes about 5 seconds before it kills them, so a park that needs
longer is cut off where the arm is. That is why the command brings the arm home
itself (Finishing, above) before it stops the run.

## Safety

- The arm only moves after you confirm, pose by pose unless you gave
  `--no-confirm-each`; parking (above) is the only exception.
- **q**, **p**, Ctrl-C, closing the terminal (SIGHUP) or closing its input aborts,
  at any time: no new targets are sent and the arm holds where it is, its motors
  on. It is never moved home by an abort. Going home is offered next, and the
  run is stopped only with the arm at home or after your `yes`. Support the arm
  if a joint was carrying a heavy load.
- A joint that lags its target by more than 10 degrees while holding stops the
  step and holds the arm. While ramping, 5 degrees more than a joint normally
  trails at that speed stops it sooner. So does a fingertip that would cross the
  guard, a target past a joint limit, or a reading more than 0.25 seconds old.
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
