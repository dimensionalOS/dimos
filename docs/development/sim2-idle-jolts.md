# G1 idle jolts: captured evidence

**2026-10-29, macOS. Open investigation; no fix established.** G1 GR00T WBC,
`unitree-g1-groot-wbc-unreal`, MuJoCo physics, Zenoh, Konbini scene, Unreal camera
and a separate native MuJoCo inspection viewer. Nominal physics is 200 Hz and
policy control is 50 Hz. Performance changes from `df19aa9a61` were present;
viewer integration is recorded at `f553fbb9d8`.

## Observations

The user reported that changing macOS to High Power removed the visible jolts.
A subsequent 20-second capture in the run started at 22:02:46 (PID 6950) showed simulation
time / wall time = **1.00001**, continuous physics samples and approximately
50 Hz commands. The user was moving during this capture, so it is not a
controlled comparison with the idle recordings below.

After the user restarted, run `20261029-221511-4bc0-unitree-g1-groot-wbc-unreal`
was recorded in Low Power:

| Capture | Measured behavior |
| --- | --- |
| 60 seconds, viewer opened during recording | Before the viewer-start marker: simulation / wall time **0.9823**, essentially motionless. After the marker: **0.7726**, with physical jolts. First disturbance occurred 5.10 seconds after the marker. Timing association does not establish viewer causation. |
| 30 seconds, viewer already open, user idle | Simulation / wall time **0.8946** while policy commands continued at approximately **50 Hz**. Generated command gaps: p99 **28.7 ms**, maximum **36.9 ms**. No messages arrived on the coordinator's `twist_command` input. |

In the second recording, four jolts began at **+4.411, +6.625, +13.654 and
+27.782 seconds**. At each onset, knee velocity jumped from near rest to almost
the same **0.364 rad/s in one 5 ms physics step**. Commanded position changes
immediately beforehand were tiny; larger policy corrections followed the
physical disturbance. Peak joint speed reached **2.125 rad/s** and pelvis
height dipped approximately **19 mm**.

Both Low Power recordings contain consecutive snapshot sequences, simulation
time increasing by exactly 5 ms per sample, and unchanged episode 1. Joint
position changes agree with integrated velocities: these are physical impulses,
not pose resets or just viewer display jumps. No periodic policy reset/reseed
path was found. Repeated feedback timestamps observed beside arriving commands
alone do not prove stale policy input: transport callbacks can arrive in batches.

## Hypothesis and missing evidence

The repeated impulse before a position-target change suggests a temporary
change in applied torque or gains. **Watchdog damping is a candidate, not a
confirmed cause.** `SimulationRuntime` expires commands using wall-clock time
and switches effort motors to damping; command acceptance waits on the same
lock held through physics stepping and publication. Catch-up stepping under
load could delay acceptance even while the policy publishes regularly. The
connection also has a separate deadman.

The traces lack command gains, applied effort and simulator acceptance timing.
They observe commands upstream of G1Connection forwarding, so regular command
generation does not establish regular delivery to the simulator.

Next passive recording: capture **both sides of the connection forwarder**,
including command timestamps and all five arrays (`q`, `dq`, `kp`, `kd`, `tau`),
plus `JointState.effort`, position, velocity, acquisition timestamps and snapshot
metadata. Check whether load-bearing effort disappears at each impulse despite
unchanged upstream gains. If still ambiguous, measure accepted-command age and
watchdog transitions at physics ticks.

## Architecture consequence

Clock ownership, feedback freshness, command acceptance and overload behavior
belong in the simulation/control runtime. They are not URDF-manager or robot
asset responsibilities. Preserve acquisition timestamps and define how physics
and policy progress together under load. **Increasing the watchdog timeout is
not a correctness fix.** This note does not prescribe a new protocol before the
missing measurements are collected.

Relevant code: [`world/runtime.py`](/dimos/sim2/world/runtime.py),
[`sim2/module.py`](/dimos/sim2/module.py),
[`control/connection/connection_module.py`](/dimos/control/connection/connection_module.py),
[`whole_body/transport/adapter.py`](/dimos/hardware/whole_body/transport/adapter.py),
[`tick_loop.py`](/dimos/control/tick_loop.py),
[`g1_groot_wbc_task.py`](/dimos/control/tasks/g1_groot_wbc_task/g1_groot_wbc_task.py).

Local evidence directory: `~/.local/state/dimos/unreal/konbini-store/`
(machine-local files, not checked into the repository):

- `g1-high-power-timing.json`
- `g1-low-power-timing.json`
- `low-power-viewer-start.json`
- `g1-low-power-viewer-timing.json`
