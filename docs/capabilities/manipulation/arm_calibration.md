# Calibrate a Damiao arm

The calibration walks one Damiao arm through fitting its link masses and its kp/kd gains, and saves them to one file per arm that the arm loads every time it starts. `dimos run <robot>-calibrate` runs it and `dimos shell` drives it.
It works with every robot that has a `<robot>-calibrate` blueprint: `openyam`, `dual-openyam`, `metal`, `dual-metal`, `b601-dm` and `openarm`.

## Before you start

- Clear the workspace. Stand beside the arm so you can catch it.
- Measure the table height (`floor-z-m`) and the line the fingertip must stay in front of (`min-x-m`), in metres from the arm's base. Both are required. On a two-arm robot each arm also stays on its side of the midline between the two bases.
- Know the CAN port of every arm on the robot, including the one you are not calibrating.
- Nothing else may be using the arm. If something is, the connection fails to open the CAN bus and `dimos run` shows that error.

## Run it

In one terminal, start the calibration of one arm, here the right arm of a Dual OpenYAM:

```bash
dimos run dual-openyam-calibrate --calibrationmodule.arm right \
    --calibrationmodule.floor-z-m 0 --calibrationmodule.min-x-m 0.1 \
    --dualopenyamconnection.left-can-port can0 \
    --dualopenyamconnection.right-can-port can1
```

In another, open `dimos shell` and drive it:

```python
cal = app.CalibrationModule
cal.run()                # "started": check, then every step
print(cal.status())      # the plan, the latest message, and the question with its answers
cal.answer("go")         # answer the question, then print(cal.status()) again
cal.abort()              # the arm stops and holds where it is; you are asked whether to go home
```

Stop the run with Ctrl-C and start it again with `--calibrationmodule.arm left` for the left arm. `cal.run(["gravity_poses", "gravity_fit"])` runs only those steps after check. The other robots take `--openyamconnection.can-port`, `--metalconnection.can-port`, `--b601dmconnection.can-port`, or `--dualmetalconnection.` / `--openarmconnection.` with `left-can-port` and `right-can-port`.

| Answer | When |
|--------|------|
| `go` | Go on |
| `skip` | Skip this move, joint or step |
| `keep`, `save` | Keep what the arm runs, or save a fit that is worse over all |
| `home`, `stay` | After a failure or an abort: go home, or hold where it is |

`run`, `answer` and `abort` return at once: `"started"` or `"ok"`, or why not. The calibration asks before every move; with `--calibrationmodule.confirm-each false` it asks only at the start of each step. A question waits until it is answered, and the arm holds meanwhile. `cal.status().log` keeps the last 50 messages, questions and answers. If the arm is not home when the calibration ends, the last message says so: support it before you stop `dimos run`, because stopping turns the motors off.

## What each step does

1. **check.** Watches the arm at rest for one rest window to measure its noise and how often readings arrive, checks every joint is inside its limits, then moves home.
2. **gravity_poses.** Moves to each gravity pose, lets it settle, then nudges every joint whose gravity torque can vary up and down, holding each nudge 1 s. Each joint's nudge is 0.3 of its torque cap over its kp, at least the dither and at most 5 deg. A pair the joint followed less than halfway is redone once at twice the size. One still stuck stays in the fit and is listed; a joint whose every pair stuck gets a "friction not cancelled" warning.
3. **gravity_fit.** Fits the link masses, holding the joints that were not nudged at their URDF torque. It shows each joint's gravity torque error for the URDF, the running model and the fit, each pose held out, and the total over all joints. If the total is no better than the running model's, it lists the joints it makes worse and asks: `keep` keeps what the arm runs, `save` saves the fit anyway. Otherwise `go` saves and applies it, with each joint's friction.
4. **float_check.** Each gravity-loaded joint in turn rests 2 s, then goes limp (kp 0, gravity still compensated) for 10 s at the pose where it carries the most gravity. It passes if the fingertip ends within 2 mm of where it went limp. A joint that turns 5 deg is held again and fails. Be ready to support the arm. Skipped on robots that do not compensate gravity.
5. **step_sweep.** At the pose farthest from the joint limits, each joint steps by each step delta at its own gains, then at each kp scale by each kd scale, kd at most 5 and kp/4. A step that buzzes stops that candidate only; the joint gets its gains back and goes on.
6. **pick_gains.** Proposes, per joint, the passing gains with the lowest score: worst rest error over its limit (1.5 times friction over kp, at least 0.1 deg) plus worst settle over 0.5 s. A largest step that barely moved rejects a candidate; a smaller one is left out. Nothing is saved here.
7. **confirm_tracking.** Revisits some gravity poses with the new calibration and the proposed gains, and compares how far each joint rests from its target with the first visit. The gains are saved only if no joint stalls and each rests within 1.5 times its first visit or its rest limit; otherwise they are undone.
8. **summary.** Shows the file the arm runs now.

A hold ends once the arm has settled: one rest window of readings stays inside the noise measured in check. A hold also ends if it stops settling (it oscillates in place) or has not settled after 5 s, and that counts as a failure. A motion stops with an error if a joint falls behind its target by more than its motor can push through, or the arm reaches the table or the other arm's side.

The calibration moves the arm only by streaming targets to its connection, one per reading. While it waits for you or computes a fit, it sends the arm nothing, so the connection holds the arm where it is and undoes any temporary gain change or limp joint. The same happens if the calibration stops.

## What you get

The file is `~/.local/state/dimos/arms/<arm id>/calibration.json`, for example `~/.local/state/dimos/arms/dual_openyam/right/calibration.json`. The file it replaces goes to `history/` next to it. `cat` it to see the gains and link masses.

```python
cal.bake("/path/to/in.urdf")                        # writes calibrated.urdf beside the file
cal.bake("/path/to/in.urdf", "/path/to/out.urdf")
```

`bake` writes a copy of the URDF with the arm's fitted masses and centres of mass and returns its path; the paths are those of the machine running `dimos run`. It refuses a URDF other than the one the calibration was made with.

The connection applies the file at every start. Its `calibration` setting is `auto`, `off` or a path; `--dualopenyamconnection.calibration off` runs without it. A file that does not load or does not fit the arm makes the connection fail to start.

## Settings

Each is an option of `dimos run <robot>-calibrate`, given as `--calibrationmodule.<name>`; a list is JSON, e.g. `--calibrationmodule.kp-scales '[0.6,1,1.5]'`.

| Name | Default | Meaning |
|------|---------|---------|
| `arm` | | `left` or `right` on a two-arm robot |
| `floor-z-m`, `min-x-m` | | Table height, and the line the fingertip stays in front of, m |
| `poses` | 12 | Gravity poses to visit (at least 3) |
| `speed-deg-s` | 10 | Peak joint speed of every move; at most 60 and the URDF velocity limits |
| `rest-window-s` | 0.5 | How long the arm sits still to measure its noise, and how long it must stay inside that noise to count as settled |
| `dither-deg` | 1 | The smallest nudge either way at a gravity pose |
| `kp-scales` | [0.6,1,1.5] | kp multiples the sweep tries; must include 1 |
| `kd-scales` | [1,2.5,5] | kd multiples tried with each kp; must include 1 |
| `step-deltas-deg` | [1,-1,3,-3] | The sweep's steps, in turn |
| `max-overshoot-pct` | 20 | A step that overshoots this share of itself fails |
| `float-drift-limit-mm` | 2 | How far the fingertip may end from where the joint went limp |
| `confirm-poses` | 3 | Poses `confirm_tracking` revisits |
| `grid-points` | 3000 | About how many candidate poses the pose search tries |
| `steps` | all | Steps to run after check when `run()` is given none |
| `confirm-each` | true | `false` asks only at the start of each step. Not for a first run |

The other numbers above are settings too: `max-hold-s`, `max-dither-deg`, `dither-torque-fraction`, `dither-hold-s`, `stuck-fraction`, `dither-span-nm`, `hold-s`, `float-window-s`, `float-trip-deg`, `kd-per-kp`, `rest-limit-deg`, `friction-rest-factor`, `settle-s` and `tracking-rest-ratio`.
