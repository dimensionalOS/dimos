# Control Autotune

Autotune takes a robot you describe and gives back its plant model and tuned
velocity-controller gains. You declare what the robot is; it runs a short
battery of motion tests, fits a model per axis, and writes out the gains in the
format a control task already reads.

The model it fits is a first-order-plus-deadtime (FOPDT) model: a steady-state
gain `K`, a time constant `tau`, and a lumped deadtime `L` per axis. That is
enough to derive bandwidth and to tune a PI controller analytically.

This module supersedes the older `dimos/utils/characterization` and
`dimos/utils/benchmarking` trees for plant identification and tuning.

## What you declare: the RobotProfile

Autotune does not guess what your robot is. You declare it:

- `command_interface` — how the robot is commanded (`twist` today).
- `odom_type` — whether feedback is body `velocity` or world-frame `pose`.
- `channels` — the controllable axes and each one's saturation limit (`vmax`).
- `fitter` — `velocity` or `pose` (see below). You choose; the module advises.
- `controller_form` — what to tune (`velocity_pi`).
- `command_stream` / `feedback_stream` — the coordinator topics to use.

The excitation battery is driven off the profile, not hardcoded. Test
amplitudes are fractions of each channel's `vmax` (default 25/50/75%), so the
same battery scales to a slow robot or a fast one without edits.

## Velocity fitter vs pose fitter

You pick which domain to identify in. The two fitters exist because odometry
rate matters:

- The velocity fitter reconstructs body velocity from odometry and fits the
  step response. It needs odometry that is fast relative to `tau`.
- The pose fitter fits the model against raw pose without differentiating. On a
  legged base whose odometry is slow, differentiating smears the step and
  inflates `tau`; the pose fitter avoids that.

The passive probe reports how many feedback samples land inside one time
constant. A handful means the velocity fitter will struggle. That is advice —
it does not switch the fitter for you. You know your robot; the module does not.

## How a run goes

1. Declare the `RobotProfile`.
2. Passive probe: with the robot still, measure each stream's rate, timing
   jitter, and noise floor. No motion. Read the samples-per-`tau` advice.
3. Drive the battery through the coordinator while the learning recorder
   captures each run as an episode. One excitation run is one episode; the
   amplitude-and-repeat battery is a multi-episode collection.
4. Fit the FOPDT per axis from the recorded episodes (offline — you can re-fit a
   collection without re-driving the robot).
5. Derive bandwidth from the fit, and emit Bode and pole-zero plots.
6. Lambda-tune the PI gains, run a robustness sweep, and score a verdict per
   axis.
7. Write the tuned artifact and the characterization report.

## Bandwidth, Bode, and pole-zero

Bandwidth is derived from the fit, not measured with a frequency sweep. For an
FOPDT the -3 dB point is `1 / (2*pi*tau)`. Bandwidth is withheld when the fit
quality is too low to trust.

The Bode plot is closed-form from `K`, `tau`, `L`. The pole-zero plot shows the
one physical pole at `-1/tau`. The deadtime has no rational pole-zero form, so
the plot approximates it with a first-order Padé term, which adds a pole and a
right-half-plane zero. Those two are labeled as approximation artifacts so they
are not read as real plant roots.

## What you get out

Two separate files:

- The tuned artifact (`tuned_config.json`) — the gains and limits in the exact
  shape a control task's `from_artifact` reads. A deployed task points at this
  file by path. Autotune is the producer; the task is the consumer.
- The characterization report (`characterization_report.json`) — the measured
  properties (FOPDT per axis, bandwidth, deadzone, direction asymmetry,
  cross-axis coupling, stream timing, per-axis verdict). This is durable robot
  metadata, kept apart from the gains.

The artifact's channels are the fixed slots `vx`, `vy`, `wz`. A profile that
declares other channel names cannot be written to this contract and is
rejected — the consuming task indexes those exact names.

Gains from data that is not hardware-sourced are marked `valid_for_tuning:
false` with a do-not-tune note. Sim data is for checking the pipeline, not for
deploying gains.

## Running it

The live run is a blueprint that boots the coordinator with the robot's adapter,
the same way the `unitree-go2-characterization` blueprint does, drives the
battery, and records episodes. It must run on the robot or in simulation.

The fit, tune, and emit steps are offline and need no robot. Call them directly
on a recorded collection:

```python
from dimos.control.autotune.runner import autotune_offline
outputs = autotune_offline(profile, segments_by_channel, robot_id="my-base", sim_or_hw="hw")
```

`outputs` carries the populated profile, the tuned artifact, the characterization
report, and the per-axis tuning records. You can re-run this on a collection
captured once without re-driving the robot.

## What it does not measure

By design, to avoid effort for little return:

- Isolated measurement latency. It is already lumped into the deadtime `L` that
  the controller compensates; separating the sensor share needs an external
  clock for negligible payoff.
- Hysteresis.
- The choice of controller structure. Autotune reports the gain-versus-amplitude
  curve and flags nonlinearity, but whether a linear PI suffices is your call.
