# Recovered Go2 FREE: Stair And Stability Study

2026-10-07, Apple M4 Max, MuJoCo 3.10.0. Isolated simulation only; no physical
robot or existing user process was controlled. This follows the gait diagnosis
in [README.md](README.md), not a replacement policy or controller architecture.

## Result

The unchanged recovered FREE policy can climb stairs without lidar. Through
the actual sim2 physics owner and SHM motor/IMU device, it climbed four 18 cm
risers (72 cm total), cleared the last edge with all four feet, and stayed on
the landing for three seconds after the command returned to zero.

This is not complete locomotion acceptance. Small yaw/lateral commands still
under-track, 20 cm risers were unreliable, and final body tilt in the recorded
18 cm run was 9 degrees. Descent, pushes, narrow turns, varied materials and
the full multi-process blueprint under sensor load were not tested here.

## What Changed

One physical-model setting: Menagerie Go2 passive joint damping **2.0 -> 0.1**,
matching Unitree's `unitree_robots/go2/go2.xml` at local revision `1a37b05`.
This reduces simulated passive resistance at moving joints. It is separate
from the recovered controller's unchanged motor PD gains, **kp=40, kd=1**.

No policy weights, observations, expert selection, command gains, inertias,
armature, friction loss, contact settings or actuator limits were changed.
The asset README records the deviation and the LFS archive contains it.
This is a source-grounded simulation adjustment, not measured hardware
calibration. Lower damping improved drift, but did not improve every metric.

## Why Stairs Work Without A Scan

FREE receives velocity commands, body angular velocity, projected gravity,
joint positions/velocities and previous actions, with six history frames.
It does not receive terrain height, lidar, contact forces or a stair detector.
During these trials the same walking policy ran throughout: no stair-specific
mode, foot-placement planner, height override or hand-coded lift was added.

Contact with a step changes the sensed joint/body motion. A learned policy can
react to that motion and its recent history. That explains how a blind walker
can climb; these experiments do not establish the original training curriculum
or prove that we reproduced Unitree's full onboard policy-selection system.

Go2Web also contains a separate `b_v2` blind walker. It climbed some of these
stairs, but was not a consistently better replacement. Go2Web's FPS auto-mode
selection between FREE and FPSC is a separate unresolved deployment question;
the current DimOS connection still uses FREE only.

## Reference And Method

Private Go2Web revision: `1ffe3ee6ab17f387c4fc6d6abf84b99d9748e6a7`.
FREE weights SHA-256:
`406e735c32a8e5501c9122ef83fa61d67c70aa9bfc9a4dae266b321db1458330`.
Weights remain outside this repository. [reference_ffi.rs](reference_ffi.rs)
wraps the original Rust controllers, including their observation/history and
joint-order handling; it does not copy their implementation.

In a matched 14-second, 8 cm stair rollout, the original Rust FREE controller
and Python port differed by at most **0.027 mm** in base position. That checks
closed-loop port agreement, not agreement with a physical Go2. This early
fixture had a short landing; both continued off its end, so it is not counted
as a successful climb-and-stop trial.

Subsequent stair trials used four risers, 1.2 m width and a long landing.
Physics: 400 Hz, implicitfast. Inference: 50 Hz. One-second pose hold, two
seconds zero-command policy, then a slewed forward command. FREE requested
0.5 m/s. `b_v2` requested stick=1, which has different scaling, so its count
is not an equal-speed ranking of the policies.

The probe uses ground truth only to score and to request stopping once the
base clears the top edge by 0.7 m. The policy does not see that geometry.
Pass: all feet beyond the last edge, inside the stair width and at landing
height, base above landing+0.20 m, and no tilt over 60 degrees through a
three-second stopping interval. Merely reaching the top is insufficient.

## Matched Results

[stair-results.json](stair-results.json) retains the summarized measurements.

| Cases | Original damping 2.0 | Damping 0.1 |
| --- | --- | --- |
| 12/16/18 cm risers, two approach distances, 30 cm treads | 5/6 passed | 6/6 passed |
| Held-out 14/17/20 cm risers, varied treads, offset and +/-4 degree approaches | 5/6 passed | 5/6 passed |
| Flat forward 0.5 m/s: measured forward speed | 0.525 m/s | 0.413 m/s |
| Same forward trial: unintended sideways speed | 0.132 m/s | 0.008 m/s |
| Same forward trial: maximum body tilt | 14.9 degrees | 6.9 degrees |
| Flat yaw request 0.3 rad/s: measured yaw | 0.098 rad/s | 0.088 rad/s |
| Flat lateral request 0.2 m/s: measured lateral speed | 0.109 m/s | 0.065 m/s |

The first six stair cases used source impratio=100; held-out cases, flat
comparisons and the sim2 trial used impratio=10, as in the blueprint's world.
All used elliptic friction cones. Flat runs lasted 18 seconds; speeds are
averaged over the last four seconds. These are deterministic, small geometry
sets, not statistical estimates of stair success on arbitrary environments.

The failed 20 cm positive-yaw case failed with both damping values. The original
model fell off the side; the adjusted model did not fall but did not finish
with all feet on the landing. The recovered `b_v2` passed 4/6 of the first
set. Removing dry friction or replacing the whole robot with Unitree's MJCF
also passed 6/6, but neither change was adopted.

Actual sim2 + SHM trial: top clearance at 10.06 simulation seconds, end at
13.08 seconds, final base (2.674, -0.044, 0.980) m, maximum tilt 33.4 degrees.
The feet were all on the 0.72 m landing. This exercised `SimulationRuntime`
and `WholeBodyAdapter`; it did not deploy ControlCoordinator, Zenoh streams
or camera/lidar worker modules. Wall duration is not a real-time benchmark.

## Repeat And Inspect

From this worktree, the existing blueprint can use [stairs.xml](stairs.xml):

```bash
uv run dimos --simulation mujoco --transport zenoh --viewer rerun \
  --scene-package "$PWD/experiments/go2_freewalk/stairs.xml" \
  run unitree-go2-freewalk
```

No automatic climbing script runs in that blueprint. The stair approach is
along +X; the measured probe requests 0.5 m/s and then stops on the landing.

For the isolated, repeatable probe, check out the pinned private Go2Web source
using your own GitHub credentials. Cargo is needed only for Rust comparisons;
video export needs imageio/PyAV. The probe runs faster than wall time:

```bash
uv run python experiments/go2_freewalk/demo_stairs.py \
  --reference-root /path/to/go2web --sim2 \
  --height 0.18 --width 1.2 --cone elliptic --impratio 10 \
  --command 0.5 0 0 --seconds 24 \
  --output /tmp/go2-stairs.json --video /tmp/go2-stairs.mp4
```

Omit `--sim2` for direct-model comparisons. `--damping 2` restores the original
joint value for a matched baseline. `--policy rust-free` or `rust-blind` uses
the original controller through the FFI, not a second maintained DimOS policy.
Each result includes settings, hashes, and measured outcomes; its companion
NPZ stores time/base state/targets, qpos and commands. Some early exploratory
NPZ files predate qpos recording and contain only `trace`.

Local raw evidence and video: `~/Desktop/sim2-go2-stair-study-20261007/`.
`sim2-18cm.mp4` is the final sim2/SHM video. Earlier exploratory outputs are
retained separately in that directory, including a failed empty video export;
they are not additional acceptance claims. No private weights are included.

Focused policy/connection regression: **17 passed**. Scoped Ruff passed.
