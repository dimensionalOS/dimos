# Calibrate a Damiao arm

`dimos arm calibrate` walks one Damiao arm through a calibration from your terminal. It fits the arm's link masses and its kp/kd gains, and saves them to one file per arm that the arm loads every time it starts.
It works with `openyam`, `dual-openyam`, `metal`, `dual-metal`, `b601-dm` and `openarm`.

## Before you start

- Clear the workspace. Stand beside the arm so you can catch it.
- Measure the table height (`--floor-z-m`) and the line the fingertip must stay in front of (`--min-x-m`), in metres from the arm's base. Both are required. On a two-arm robot each arm also stays on its side of the midline between the two bases.
- Know the CAN port of every arm on the robot, including the one you are not calibrating.
- Nothing else may be using the arm. If something is, the connection fails to open the CAN bus and the command shows that error.

## Run it

```bash
dimos arm calibrate dual-openyam --arm left --floor-z-m 0 --min-x-m 0.1 \
    --dualopenyamconnection.left-can-port can0 \
    --dualopenyamconnection.right-can-port can1
```

Add `--dry-run` first. It prints the plan and the exact `dimos run` command, and starts nothing. Flags the command does not know go to `dimos run <robot>-calibrate`, which is how the CAN ports get there. The other robots take `--openyamconnection.can-port`, `--metalconnection.can-port`, `--b601dmconnection.can-port`, or `--dualmetalconnection.` / `--openarmconnection.` with `left-can-port` and `right-can-port`.

| Key | What happens |
|-----|--------------|
| Enter | Go on (the first choice shown) |
| s | Skip this move, joint or step |
| q, or Ctrl-C | Abort. The arm stops and holds where it is, and you are asked whether to go home |

The command asks before every move. With `--no-confirm-each` it asks only at the start of each step. If the arm is not home when the calibration ends, the command waits for Enter before it stops the robot, because stopping turns the motors off.

## What each step does

1. **check.** Watches the arm at rest for one rest window to measure its noise and how often readings arrive, checks every joint is inside its limits, then moves home.
2. **gravity_poses.** Moves to each gravity pose, lets it settle, then nudges each gravity-loaded joint up and down by the dither and lets it settle each time.
3. **gravity_fit.** Fits the link masses. It shows each joint's gravity torque error for the URDF, the running model and the fit, each pose held out. If the fit is no better, it is not offered. Otherwise Enter saves and applies it.
4. **float_check.** Each gravity-loaded joint in turn goes limp (kp 0, gravity still compensated) at the pose where it carries the most gravity. It passes if it settles with the fingertip inside the drift limit. Be ready to support the arm. Skipped on robots that do not compensate gravity.
5. **step_sweep.** At the pose farthest from the joint limits, each joint steps by each step delta at each kp scale (kd times the square root of the scale). A gain that keeps the joint buzzing stops that joint's sweep.
6. **pick_gains.** Proposes, per joint, the stiffest gains whose steps all passed, with every softer one passing too. Enter saves and applies them.
7. **confirm_tracking.** Revisits some gravity poses with the new calibration and compares how far each joint rests from its target with the first visit.
8. **summary.** Shows the file the arm runs now.

A hold ends once the arm has settled: one rest window of readings stays inside the noise measured in check. A step or a float also ends if it stops settling (each window moves as much as the one before), and that counts as a failure. A motion stops with an error if a joint falls behind its target by more than its motor can push through, or the arm reaches the table or the other arm's side.

The calibration moves the arm only by streaming targets to its connection, one per reading. While it waits for you or computes a fit, it sends the arm nothing, so the connection holds the arm where it is and undoes any temporary gain change or limp joint. The same happens if the command or the calibration stops.

## What you get

The file is `~/.local/state/dimos/arms/<arm id>/calibration.json`, for example `~/.local/state/dimos/arms/dual_openyam/left/calibration.json`. The file it replaces goes to `history/` next to it.

```bash
dimos arm show dual-openyam --arm left       # path, gains, link masses, replaced files
dimos arm bake dual-openyam --arm left --urdf in.urdf --out calibrated.urdf
```

`bake` writes a copy of the URDF with the fitted masses and centres of mass. It refuses a URDF other than the one the calibration was made with.

The connection applies the file at every start. Its `calibration` setting is `auto`, `off` or a path; `--dualopenyamconnection.calibration off` runs without it. A file that does not load or does not fit the arm makes the connection fail to start.

## Settings

Each is a flag of `dimos arm calibrate` (and a `--calibrationmodule.` option of the blueprint).

| Flag | Default | Meaning |
|------|---------|---------|
| `--arm` | | `left` or `right` on a two-arm robot |
| `--floor-z-m`, `--min-x-m` | | Table height, and the line the fingertip stays in front of, m |
| `--poses` | 12 | Gravity poses to visit (at least 3) |
| `--speed-deg-s` | 10 | Peak joint speed of every move; at most the URDF velocity limits |
| `--rest-window-s` | 0.5 | How long the arm sits still to measure its noise, and how long it must stay inside that noise to count as settled |
| `--dither-deg` | 1 | How far each joint is nudged either way at a gravity pose |
| `--kp-scales` | 1,1.5,2 | kp multiples the sweep tries; must include 1 |
| `--step-deltas-deg` | 1,-1,3,-3 | The sweep's steps, in turn |
| `--max-overshoot-pct` | 20 | A step that overshoots this share of itself fails |
| `--float-drift-limit-mm` | 5 | How far the fingertip may drift while a joint is limp |
| `--confirm-poses` | 3 | Poses `confirm_tracking` revisits |
| `--grid-points` | 3000 | About how many candidate poses the pose search tries |
| `--steps` | all | Steps to run after check, comma-separated |
| `--no-confirm-each` | | Ask only at the start of each step. Not for a first run |
