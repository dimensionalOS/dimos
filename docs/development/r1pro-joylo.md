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
`~/.cache/dimos/sim2/r1pro-joylo-2025/` directory.

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

For JoyLo, the leader module publishes complete, unsmoothed readings and delegates
jump rejection to `R1ProJoylo`. The generic reader's default jump guard compares
against its last accepted sample: after a serial dropout and a large operator
movement it can otherwise reject every later sample, preventing Minus/X from
recovering. JoyLo retains the current reading while latching a pause, so a fresh
enable press can resume tracking without a restart. Input errors still drop samples;
timeouts, discontinuity stopping, joint limits and rate limits remain enforced.
Generic OpenArm blueprints retain their reader-side jump guard.

The Feetech reader reports calibrated angles in [-pi, pi]. The JoyLo blueprint
sets `leader_angle_period=2*pi` so successive samples are unwrapped before
absolute targeting and jump detection. A captured joint-3 crossing from -3.13392275 to
+pi was only -0.0076699 rad of physical movement, but previously triggered the
0.75 rad jump guard as a +6.2755 rad discontinuity. Input gaps over 250 ms still
pause and require re-enabling; recovery preserves the nearest continuous angle
branch instead of resetting it. A single-turn encoder cannot resolve extra full
turns made while input is missing.
`R1ProJoylo.status()` retains `pause_count` and `last_pause_reason` after recovery,
and every active-to-paused transition is logged with input ages. Real jumps
include the offending joint and delta in the reason.

Separate observed input faults remain hardware/transport issues: the Feetech
SDK decodes motor status bit 1 as an input-voltage error and communication -6
as no reply. Those samples are rejected, not silently accepted. HID report loss
also correctly pauses following; wrap handling does not disable either guard.


The coordinator owns one trajectory task covering both arms and torso, two
gripper tasks and one base velocity task. The sim connection uses CC's same
joint/interface declarations. It converts canonical 0–0.1 m jaw opening into
the simulated fingers' 0–0.100 m aperture, and converts measured world-frame
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
| Release, then press X | Enable following the fixed calibrated leader joint positions |
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
leaders or reproduce Dynamixel torque locks/haptics. Unlocking resumes movement
toward the same absolute leader targets, at the configured rate. Enabling,
pausing, input recovery and unlocking never change the calibration or create
a new joint offset. The follower starts its catch-up motion from measured
feedback; it does not treat that feedback as the leader's new zero.

The leaders are not kinematic twins of R1 Pro: corresponding calibrated joint
angles map directly to follower joint targets. Saved encoder zeros and signs
determine that mapping. Joint limits, coupled wrist limits and finite motion
rates still apply; this is not Cartesian retargeting or IK.
`R1ProJoylo.status()` exposes calibrated leader angles, outgoing arm targets,
measured arm feedback, locked joints and URDF limit clipping. Coupled wrist
clipping happens separately in the downstream servo.

In the two-arm profile, both seven-joint leaders, both Joy-Cons and robot feedback must be fresh.
Loss of any input for 250 ms latches a pause; reconnecting while X is held
does not enable. Release and press X again. Input timestamps reject queued
old commands, so synchronize clocks when using two machines. A leader jump
over 0.75 rad also requires re-engagement. Initial arm targets and jaw openings
come from measured feedback, with no automatic home movement.

The base defaults to 0.1 m/s (0.2 with boost), yaw to 0.25 rad/s, arms to
1.5 rad/s and torso joints to 0.5 rad/s in the simulation blueprints. The
generic profile retains conservative 0.25/0.15 rad/s hardware defaults. A/B/Y, capture and rail buttons
do not start/save/discard episodes in this profile. Recording workflow and
physical-robot deployment remain follow-up work.


## WIM model and controller alignment (2026-11-12)

The recovered WIM source snapshot is **d2594a1**; its distributed controller
binary was built from **235e224**. This integration is a source-guided simulation
change, not deployment or a benchmark of that binary.

Our previous arm path stacked 0.25 rad/s (JoyLo), 0.3 rad/s (coordinator), and
0.5 rad/s (servo) limits. A 90-degree leader move therefore required at least
6.28 seconds. The sim now uses 1.5 rad/s throughout; torso is 0.5 rad/s.
These are simulation tracking settings, not measured hardware maxima. WIM's
`robots/r1pro/r1pro.yaml` limits arm speed to 3/3/3/5/5/5/5 rad/s, and its
`demo-motion-big.yaml` proposes 1.5 rad/s. That demo file is not proof of a
successful physical run.

The public vendor `R1Pro/urdf_r1pro_2025.urdf` at `2e5d31e` matches WIM's
on-robot URDF joint frames, axes, limits and link inertias. We use that model
instead of the 2026 G1Z variant: 0.40/0.30 m torso links, vendor masses/COMs/
inertias, and +/-50 mm finger travel. JoyLo's torso solver and the coordinator's
joint limits use the same selected model. This establishes agreement with their
URDF; physical dimensions still need confirming against our particular robot.

The 2025 mesh package is missing from the vendor URDF repository. Available
arm/torso/chassis/jaw meshes come from the official
[Galaxea simulator at abe7f516](https://github.com/OpenGalaxea/GalaxeaManipSim/tree/abe7f5161eeaa150e6eaffdf443af5df7f23f356/galaxea_sim/assets/r1_pro).
Those visuals/collisions are approximate for chassis revisions; unavailable
head/sensor geometry is explicitly omitted while retaining its inertia and
frames. Contacts use convex mesh hulls and the existing wheel cylinders.
Do not treat camera images or head/sensor collision clearance as validated.

The servo follows WIM's outer-loop structure:

- Three independent Ruckig groups (torso, left, right), updated every 4 ms.
  Acceleration is 1.5 times the speed setting; jerk is 10,000 rad/s^3. Positions
  and velocities go to the simulated PD drive, initialized from measured pose.
- Pinocchio gravity uses measured torso angles and commanded arm angles, with
  the vendor URDF inertias. Sim torso gravity compensation approximates the
  hidden firmware position loop; WIM itself sends no torso feedforward torque.
- The vendor's coupled J6/J7 polynomial bounds are applied, as well as URDF
  limits. Clipping a moving bound resets the trajectory at the bounded state.
- Tiny, already-stationary retargets are normalized to avoid Ruckig's observed
  zero-duration synchronization failure. Other solver failures surface as errors.

WIM's `kp=2`, `kd=25` are raw firmware fields, not MuJoCo SI torque gains. They
are deliberately not copied. Servo gains, reflected inertia, friction/contact
behavior and the gripper/base drive remain approximations. WIM's board emulator
copies targets to feedback; it is not a measured dynamic plant. There is no
identified friction, backlash, actuator latency or closed-loop motor response
in the reviewed runtime model. Real recordings are needed to calibrate those.

A local headless 0.6 rad shoulder step reached 90% in 0.77 s versus 1.22 s with
the old servo, and final static error fell from 0.0473 rad to about 0.000003 rad.
This compares simulated servos, excludes the old 0.25 rad/s JoyLo cap, and is
not a real-robot accuracy measurement. `demo_joylo` exercises the full module
path, including torso height, jaws, chassis, input-loss pause and absolute
pose recovery after moving the leader during two pause/enable cycles.

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
synthetic input, asserts measured arm/jaw/torso/base movement, fixed pose
tracking after re-enabling and timeout stopping, prints the result, and stops
its workers. It never opens input hardware. Duplicate action rejections may appear in sim status because the
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

Capturing encoder zero offsets does **not** determine rotation directions.
The calibration's per-motor `flip` sets the direction before the R1 mapping;
enabling preserves both that sign and the saved encoder zero.
A calibration with incorrect signs moves individual follower joints backwards;
incorrect zeros cause a repeatable pose offset. Verify both against known
reference joint poses. Do not use enable as a calibration operation.

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

Open Controller in DimOS Desktop with fixed frame `world`. A direct native
view is also available if Controller's browser connection is unavailable:

```bash
mjpython -m dimos.robot.galaxea.r1pro.sim.demo_live_view
```

The native view subscribes to the same simulation state; it sends no robot
commands and runs no separate dynamics. Closing it leaves the stack running.
On Linux use `python` instead of `mjpython`.

This blueprint requires only the left leader and left Joy-Con. It starts paused, holds the
right arm, right jaw, torso and base, and ignores sticks and D-pad movement.

| Left Joy-Con | Effect |
|---|---|
| Release, then press Minus | Enable fixed calibrated tracking; press again to pause |
| Capture (square button) | Pause |
| ZL | Toggle the left gripper open/closed |
| Hold L | Hold the three left wrist joints; release resumes absolute tracking |

Move one leader joint at a time to verify direction before larger movements.
The initial Mac trial uses encoder offsets saved in a separate local calibration
root. Those trial offsets were captured from the operator's then-current pose,
so they do not establish mechanical joint zero. They now remain the fixed
reference across every enable/pause cycle; a different zero requires an explicit
calibration change. Joint signs were checked separately.
The initial zero-only calibration left every `flip` false. The operator's
direction check required the following correction for the attached left leader
(numbered from shoulder to wrist):

| Leader motor / R1 arm joint | Motion | Calibration `flip` |
|---|---|---|
| 1 | Shoulder forward/backward | `true` |
| 2 | Shoulder outward/inward | `true` |
| 3 | Upper arm below shoulder | `false` |
| 4 | Elbow | `false` |
| 5 | Wrist nearest elbow | `false` |
| 6 | Middle wrist | `true` |
| 7 | Wrist nearest hand | `true` |

These signs belong to this assembled leader's separate calibration, not a
universal OpenArm default or the right arm. When calibrating this same setup
again, use `--left-flips joint_1,joint_2,joint_6,joint_7` with the calibration
CLI. To correct an existing calibration, preserve its IDs and homing offsets,
change only the relevant `flip` fields while following is paused, and restart
the operator to reload it. Do not also negate the mapping's `scale`: that
would apply the same direction correction twice. Re-engage after the reload
and check each joint individually.

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
100/100 full seven-motor leader reads over USB. A physical Minus press enabled
the left-only sim2 profile and operator movement produced up to 0.68 rad of
measured simulated left-arm movement during the observed trial. The native
viewer received all 28 simulated joints; Controller's browser view remained
unresolved on that installation. Exhaustive direction/range checks, gripper
and wrist-lock checks remain interactive validation.

## SF555 office: two pickable bottles and three robot cameras

`r1pro-openarm-joylo-office` uses the same two-leader calibration, absolute
joint mapping and Joy-Con controls as `r1pro-openarm-joylo-sim2`. It starts at
`(18.0, 8.41, 0.005)`, facing west at the east end of the downstairs engineering
tables. The head and both wrist RGB-D views are published as sim camera streams.
The original plain scene remains a separate blueprint.

The existing `Hall_A_middle_water_bottle_*` and
`Hall_A_outer_water_bottle_*` assemblies become two free rigid bodies. All 21
parts of each bottle move together. Their original visual triangles, dimensions
and placement are preserved. The bottles are removed from both the static
collision scene and the saved Unreal background, including caps/labels merged
into furniture chunks. The runtime renderer binds each native bottle mesh to
its sim2 body pose; Unreal does not run a second physics simulation.

Assumptions: each bottle weighs 0.30 kg, inertia is a solid-cylinder approximation,
collision is a convex envelope, and MuJoCo's default friction is uncalibrated.
The translucent plastic is rendered as opaque pale blue so ideal depth sees the
same graspable surface as color. There is no fluid, deformation or stereo noise
model. Other office objects stay static, with the existing approximate furniture
collisions. The office-specific robot model decomposes the four finger meshes
into convex pieces; single whole-finger hulls bridge their concave openings and
push bottles out. All robot joint frames, limits, inertias and visuals are unchanged.

A single Unreal process renders head-left RGB-D and both wrist RGB-D views.
Switching views resets temporal history to prevent cross-camera ghosting;
render targets are cached by resolution. Source pose timestamps and camera calibration accompany every pair.
The 2025 robot's head and uppercase `left_D405_link`/`right_D405_link` frames
are used directly. The previous camera profile supplies approximate fields of
view (118×92° head, 87×58° wrists), not measured 2025 factory intrinsics.
Head preview is 640×512; wrists are 320×180, with ideal wrist depth restricted
to 0.07–0.50 m. These previews are not the hardware cameras' full resolution.
Camera-rig code was adapted from committed `47713be18a`, preserving its earlier
implementation history rather than copying another worktree's uncommitted work.

### Prepare the private assets

Requires the SF555 V24 source package, its previously prepared collision export,
Blender, Unreal 5.7, and the R1 2025 meshes used by the plain blueprint. These
private/generated assets are not committed. Run from the repository root;
change the source/cache paths for another machine. Set the asset-root environment
variable before importing the blueprint or generating descriptions.

```bash
export DIMOS_SF555_JOYLO_ASSETS="$HOME/.cache/dimos/sf555-joylo"
OFFICE_SOURCE="$HOME/Desktop/sf-555-office"
OFFICE_ENGINE="/Users/Shared/Epic Games/UE_5.7"
OFFICE_EDITOR="$OFFICE_ENGINE/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor"
URDF_GENERATOR="$HOME/.codex/skills/urdf/scripts/urdf"

.venv/bin/python -m dimos.sim2.scenes.sf555.pickables \
  --source "$HOME/.cache/dimos/sf555" --output "$DIMOS_SF555_JOYLO_ASSETS"
for target in office bottle_a bottle_b; do
  SF555_URDF_TARGET="$target" .venv/bin/python "$URDF_GENERATOR" \
    dimos/sim2/scenes/sf555/pickables.py \
    -o "$DIMOS_SF555_JOYLO_ASSETS/$target.urdf"
done
.venv/bin/python -m dimos.robot.galaxea.r1pro.sim.grasp_contacts
.venv/bin/python "$URDF_GENERATOR" \
  dimos/robot/galaxea/r1pro/sim/grasp_contacts.py \
  -o "$DIMOS_SF555_JOYLO_ASSETS/r1pro.urdf"

.venv/bin/python -m dimos.sim2.scenes.sf555.prepare_unreal \
  --source "$OFFICE_SOURCE" --output "$DIMOS_SF555_JOYLO_ASSETS" \
  --engine "$OFFICE_ENGINE"
/Applications/Blender.app/Contents/MacOS/Blender --background \
  --python dimos/sim2/scenes/sf555/export_pickables.py -- \
  "$OFFICE_SOURCE" "$DIMOS_SF555_JOYLO_ASSETS"
"$OFFICE_EDITOR" "$DIMOS_SF555_JOYLO_ASSETS/DimosRenderer/DimosRenderer.uproject" \
  -run=pythonscript \
  -script="$PWD/dimos/sim2/scenes/sf555/prepare_pickables_unreal.py" \
  -unattended -NullRHI -nosound
```

The editor script saves `/Game/SF555/Maps/OfficeJoylo` only inside the derived
project and writes `native-visuals.json`. Source hashes guard against mixing
Blender versions; `pickables.json`, `visual-split.json`, `finger-collisions.json`
and `unreal-pickables-report.json` record the extraction and placement.
`DIMOS_UNREAL_EXECUTABLE` and `DIMOS_UNREAL_PROJECT` override the local runtime
paths without changing code.

Run `dimos run r1pro-openarm-joylo-office --daemon` with the same explicit
left/right serial ports and calibration-root arguments used for the plain
blueprint. Use `--r1projoylo.arm-only false` for base/torso controls. X enables
following; Home pauses. Camera startup is asynchronous: inspect
`R1ProSimCameras.statistics()` for all three counters and an empty `error`.
RGB-D and CameraInfo are available on
`color_image`/`depth_image`/`camera_info`, `left_wrist`/`left_wrist_depth`/
`left_wrist_info`, and the corresponding `right_wrist` topics.

### Contact and camera checks

`python -m dimos.robot.galaxea.r1pro.sim.demo_office_grasp` exercises each real
finger model against each prepared bottle. A driven vertical fixture holds the
wrist orientation; the finger joints and bottle obey dynamics. All four cases
lifted 0.179–0.184 m, held, then released. There is no weld or artificial object
attachment. This validates contact feasibility, not a full arm pick trajectory.
The operator still needs to line up the grasp using the wrist view.

The focused extraction, renderer protocol, camera calibration and rig tests pass.
Native rendering was built and all three live RGB streams were inspected in the
full office. Rendering load can reduce both camera rates and simulation speed
on this Mac; use simulation-time/wall-time deltas and camera counters to measure
that separately from leader/Joy-Con input ages. A zero camera error does not
claim a particular frame rate or real-time physics performance.
