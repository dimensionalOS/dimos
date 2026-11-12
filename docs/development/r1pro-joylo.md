# OpenArm + Joy-Con control of R1 Pro in sim2

This integration uses the existing seven-motor Feetech OpenArm leaders, with
a Nintendo Joy-Con attached to each leader. It does not require the pending
Dynamixel hardware. The first follower is R1 Pro in MuJoCo through sim2;
none of these blueprints opens a vendor ROS connection to a physical robot.

## Dependency and review structure

The integration branch preserves CC's and Krishna's original commits through
merges, with the robot-specific adapter and profile added above them:

| Layer | Pinned source | Role |
|---|---|---|
| Current control contract | `391ae2d0ef`, main / #5016 | Joint/interface routing and connection ownership |
| CC's R1 Pro stack | `3920bf8b1d0517476be8aaa891dac2075a1469e6`, #5017 + #5019 | R1 joint declarations, limits, base and canonical grippers |
| Krishna's input stack | `cedc04d1f2f23b249c61acab25881b5e476bc5f5`, #5177 + #5182 + #5198 | Feetech calibration/readers, leader streaming, Joy-Con backend |
| sim2 foundation | `c07ca335fa` | Simulator, sensor and action contracts |
| Existing R1 Pro sim work | `47713be18a` | Reused committed `sim/model.py`, `config.py`, `control.py`, tests, scene collision exclusions and startup timeout |

The R1 sim files are ported from that committed snapshot; uncommitted work in
the existing sim worktrees is not part of this branch. Its older control and VR
integration are not prerequisites. The model uses a separate
`~/.cache/dimos/sim2/r1pro-joylo/` directory.

This is a draft integration branch targeting main. Before making a small
mergeable feature PR, land the CC, Krishna and sim2 dependencies, then carry
the JoyLo-specific commits onto their merged versions. The inherited sim2
`dimos-lcm` Git pin remains necessary for its simulation message types.

## Control path

```text
OpenArm Feetech encoders -> Krishna LeaderTeleopModule -> raw leader joints --+
Joy-Con L + R -> Krishna JoyConTeleopModule -> full buttons/sticks/presence --+
                                                                          |
                                                               R1ProJoylo profile
                                                                          |
                                                    CC ControlCoordinator tasks
                                                                          |
                                                          R1ProSimConnection
                                                                          |
                                                         R1ProSimControl -> sim2
```

The full Joy-Con message is versioned `sensor_msgs.Joy`, including presence
bits and all printed button labels. The generic reader's existing command
outputs are disabled only in this profile so there is one writer for base,
jaws and arm authority. Its defaults remain compatible with Krishna's OpenArm
blueprints.

The coordinator owns one trajectory task covering both arms and torso, two
gripper tasks and one base velocity task. The sim connection uses CC's same
joint/interface declarations. It converts canonical 0–0.1 m jaw opening into
the simulated fingers' 0–0.104 m aperture, and converts measured world-frame
base velocity into body-frame feedback. The servo handles the physical wheel
steering and wheel speeds.

## R1 Pro JoyLo controls

The layout follows the seven-joint R1 Pro implementation in
[BEHAVIOR-1K JoyLo at b544acbbb76b](https://github.com/StanfordVL/BEHAVIOR-1K/tree/b544acbbb76b/joylo).
The [older JoyLo overview](https://behavior-robot-suite.github.io/docs/sections/joylo/overview.html)
describes the earlier system; its six-joint mappings should not be substituted
for the R1 Pro profile.

| Control | Effect |
|---|---|
| Release, then press X | Align the leaders to measured follower joints and enable |
| Home | Pause all following and zero base velocity |
| Minus / plus | Toggle the left / right arm lock |
| Hold L / R | Hold the corresponding three wrist joints |
| ZL / ZR | Toggle the corresponding jaw open/closed |
| Left stick | Body-frame forward/back and lateral motion |
| Left stick click | Double the base speed |
| Right stick horizontal | Base yaw |
| Right stick vertical | Torso pitch trim |
| Right stick click | Suppress yaw and pitch, matching the pinned reference implementation |
| D-pad up / down | Raise / lower the folding torso while preserving upper pitch |

Locks hold follower targets in software. They do not energize the Feetech
leaders or reproduce Dynamixel torque locks/haptics. Unlocking reanchors
the joints so a leader moved while locked does not cause a catch-up jump.
The leaders are not kinematic twins of R1 Pro: this first implementation maps
corresponding calibrated joint deltas, with engagement alignment, joint limits
and rate limits. It is not Cartesian retargeting or IK.

In the two-arm profile, both seven-joint leaders, both Joy-Cons and robot feedback must be fresh.
Loss of any input for 250 ms latches a pause; reconnecting while X is held
does not enable. Release and press X again. Input timestamps reject queued
old commands, so synchronize clocks when using two machines. A leader jump
over 0.75 rad also requires re-engagement. Initial arm targets and jaw openings
come from measured feedback, with no automatic home movement.

The base defaults to 0.1 m/s (0.2 with boost), yaw to 0.25 rad/s, arms to
0.25 rad/s and torso joints to 0.15 rad/s. A/B/Y, capture and rail buttons
do not start/save/discard episodes in this profile. Recording workflow and
physical-robot deployment remain follow-up work.

## Run the simulation check on this Mac

From this branch's checkout:

```bash
uv sync --extra base --extra manipulation --extra cpu --extra sim --all-groups
source .venv/bin/activate
python -m dimos.robot.galaxea.r1pro.sim.model
python -m dimos.robot.galaxea.r1pro.sim.demo_joylo
```

The model command fetches the pinned vendor asset source on first use. The
demo starts real module workers and MuJoCo on an isolated Zenoh domain, sends
synthetic input, asserts measured arm/jaw/torso/base movement and timeout
stopping, prints the result, and stops its workers. It never opens input
hardware. Duplicate action rejections may appear in sim status because the
servo retransmits the same command when sim2 repeats a barrier observation;
the simulator rejects that duplicate without applying it twice.

For an interactive receiver, use `dimos run r1pro-joylo-sim2`. It starts paused
and waits for the physical operator stream. `r1pro-joylo-sim2-mock` additionally
exposes `JoyloSimInput.send(...)` through `dimos shell`; each sample is sent
once and callers must refresh it. The demo shows that API.

## Connect the existing leaders

Joy-Cons work over Bluetooth on macOS and Linux. The reader automatically uses
HIDAPI on macOS and Krishna's `hid-nintendo` event-device backend on Linux.
The HID backend reads factory/user stick calibration and the original
Nintendo Joy-Con's full reports, preserving upright button labels and keeping
the two sides distinct. Install `--extra joycon-teleop` on macOS. It configures
the report mode and reads calibration; it does not change pairing or flash.
`--joyconteleopmodule.backend hid` selects HID explicitly; `evdev` selects the
Linux reader. Original Joy-Con L/R are supported; Joy-Con 2 is not supported.

Use `r1pro-openarm-joylo-sim2` for both operator and simulation on one machine,
or split the operator and receiver across machines. Pair both Joy-Cons and
check their actual labels/axes with `python -m dimos.teleop.joycon.probe 60`.
Run the probe while the teleop blueprint is stopped so only one reader owns the devices.
See [the existing input setup guide](/docs/capabilities/manipulation/openarm_mini_teleop.md#joy-cons-instead-of-trigger-servos)
for pairing and driver details.

Install the `leader-teleop` extra on the operator. Use Krishna's calibration
CLI for each arm and verify signs with `joint-tui` before following. This
profile requires exactly `joint_1` through `joint_7`; it takes jaw input from
the Joy-Cons, so an old calibration with an eighth `gripper` motor needs a
separate arm-only calibration. Copy the existing files to a separate calibration
root and remove only their `gripper` entry, preserving all seven arm offsets
and flips; select that root with `--leaderteleopmodule.calibration-root`.
Joint flips depend on the actual assembled
leaders and should be verified against one R1 sim joint at a time.

### Left arm only on this Mac

Pair the left Joy-Con in macOS Bluetooth settings and connect the seven-motor
Feetech leader over USB. Enable power to the entire motor chain. With a separate
seven-joint calibration under `<calibration-root>/left/calibration.json`:

```bash
uv sync --extra base --extra manipulation --extra cpu --extra sim \
  --extra leader-teleop --extra joycon-teleop --all-groups
source .venv/bin/activate
dimos run r1pro-openarm-joylo-left-sim2 --daemon \
  --leaderteleopmodule.port-left /dev/cu.usbmodem<your-port> \
  --leaderteleopmodule.calibration-root <calibration-root>
```

Open Controller in DimOS Desktop with fixed frame `world`. This blueprint
requires only the left leader and left Joy-Con. It starts paused, holds the
right arm, right jaw, torso and base, and ignores sticks and D-pad movement.

| Left Joy-Con | Effect |
|---|---|
| Release, then press Minus | Align and enable; press again to pause |
| Capture (square button) | Pause |
| ZL | Toggle the left gripper open/closed |
| Hold L | Hold the three left wrist joints; release reanchors them |

Move one leader joint at a time to verify direction before larger movements.
The initial Mac trial uses encoder offsets saved in a separate local calibration
root; engagement aligns the current leader pose to the current simulated arm.
Those trial offsets do not establish mechanical joint zero or verify joint signs.
Missing/stale input pauses following and requires release and a fresh Minus
press after reconnection. HID silence is detected after 200 ms, in addition to
the profile's 250 ms stream timeout. `dimos shell` exposes `R1ProJoylo.status()`
with input ages, readiness and the pause reason; `dimos stop` ends the run.

### Two hosts

For two hosts, start a reachable DimOS Host router on the simulation machine.
If one is already running, reuse it. Otherwise, in a separate terminal:

```bash
# Replace en0 with the desired network interface.
HOST__LISTEN=iface/en0 dimos host start --foreground
```

Then use the same domain on both sides:

```bash
# Simulation host
dimos --robot-ip 127.0.0.1 --zenoh-mode client --zenoh-domain r1pro-joylo \
  run r1pro-joylo-sim2

# Operator host: add --extra joycon-teleop on macOS; replace placeholders.
uv sync --extra base --extra manipulation --extra leader-teleop --all-groups
dimos --robot-ip <simulation-host-ip> --zenoh-domain r1pro-joylo \
  run teleop-openarm-r1pro-joylo \
  --leaderteleopmodule.port-left <left-feetech-port> \
  --leaderteleopmodule.port-right <right-feetech-port>
```

Keep hands stationary for engagement, release X, then press X. Verify each
arm's joint direction and comfortable range, then wrist/arm locks, jaws,
torso and base. Home pauses; `dimos stop` terminates a daemonized run. The
full two-arm controller pass has not yet been performed on this branch.

## Validation of this integration

The local MuJoCo run measured 0.174/0.145 rad arm movement, approximately
104 mm aperture on each jaw, 7.8 mm of torso lowering, 100 mm of base travel
and 0.39 mm of base drift during the final paused half-second. Input loss
latched the pause and fresh neutral input did not re-enable following.

The focused input/profile/servo tests passed (126), as did the merged CC
contract/connection/hold regressions (202), scene tests (6), actual sim2 module
tests with `-m native_e2e` (16), and blueprint registry checks (5). Mypy passed
for the twelve integration source files. This validates the software path;
physical calibration and two-host timing still need testing with the actual
leaders. The Mac bring-up read the paired left Joy-Con over HID and completed
100/100 full seven-motor leader reads over USB, then ran the left-only sim2
blueprint with fresh physical input streams. Direction/range checks and the
first operator-driven following trial remain interactive checks.
