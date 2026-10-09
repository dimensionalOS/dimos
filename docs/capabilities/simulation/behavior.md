# BEHAVIOR / OmniGibson

BEHAVIOR runs an R1 Pro mobile manipulator in an isolated Python process. dimOS owns
blueprint composition, visualization, navigation, and agent tools. OmniGibson owns
physics, cameras, task evaluation, and physical/symbolic action primitives.

```text
dimOS host (NumPy 2, Rerun)
  BehaviorConnection ── streams / RPC ── BehaviorRuntime (Python 3.11, NumPy 1)
                                          └─ main thread: OmniGibson + Isaac Sim
```

## One-time setup

The runtime targets Linux x86-64 with an NVIDIA GPU supported by Isaac Sim 5.1.
Install `pixi` and `uv`; Pixi supplies the pinned CUDA 12.8 compiler, C++ toolchain,
and native libraries including `libxml2.so.2`. The isolated module automatically
runs preparation and launch commands through Pixi when its project has `pixi.toml`.
Download staging uses the runtime’s `.downloads/` directory rather than `/tmp`.
The runtime uses BEHAVIOR v3.9.2 at commit
`b1979916ec1549b10a4e65e630bc6504a9af1b00`, Isaac Sim 5.1, and PyTorch 2.7.
Setup includes the 2026 task-instance bundle, which supplies an R1 Pro start pose
for the reference task. Allow substantial disk space for simulator wheels, CUDA,
and datasets.

Read the [NVIDIA license](https://www.nvidia.com/en-us/agreements/enterprise-software/nvidia-software-license-agreement/)
and [BEHAVIOR dataset terms in the pinned upstream installer](https://github.com/StanfordVL/BEHAVIOR-1K/blob/b1979916ec1549b10a4e65e630bc6504a9af1b00/setup.sh).
The dataset terms restrict use to non-commercial academic research. Setup requires
explicit acceptance of both agreements:

```bash
python -m dimos.simulation.behavior.setup \
  --accept-nvidia-eula --accept-dataset-license
```

Use `--data-path /absolute/path` to select asset storage. Setup records the location
and completion in an ignored local marker. Subsequent launches use that location.
A failed setup leaves the marker incomplete; rerun setup to finish it.

The runtime owns all its dependencies. Setup syncs its lockfile, then installs the
shared dimOS checkout selected by `get_project_root()` with `--no-deps`.
It never installs host Rerun dependencies into the simulator environment. The
lockfile uses NumPy 1.26.4 for bidirectional NumPy 2 array serialization and
explicitly reproduces upstream's Pillow, websockets, packaging, and cffi
installation overrides. Setup also installs the pinned upstream startup icon
omitted from the OmniGibson wheel. Assets and accepted-license markers are never shipped.

## Blueprints

| Command | Layer | Purpose |
|---|---|---|
| `dimos run behavior-teleop` | R1 Pro + visualization | Camera streams and keyboard base control in `Rs_int` |
| `dimos run behavior-nav` | + voxel map, MLS planner, path follower | Click a reachable floor point in Rerun |
| `dimos run behavior-task` | R1 Pro + pre-sampled task | `picking_up_trash`, `house_double_floor_lower`, definition/instance 0 |
| `dimos run behavior-r1pro` | KronkNav + R1 Pro manipulation | Native Isaac viewer and Viser planning controls |
| `dimos run behavior-agentic` | Task + MCP server/client + skills | Inspect ground truth and execute explicitly selected primitives |

For desktop keyboard control, run:

```bash
dimos --rerun-open native run behavior-teleop keyboard-teleop
```

Focus the separate Keyboard Teleop window: W/S drives forward/backward, A/D turns,
Q/E strafes, Ctrl slows movement, and Space stops. Release movement keys to stop;
Ctrl+C in the terminal stops the whole stack. These controls move R1 Pro's base.

The task fixture is a candidate acceptance scene until the live GPU checks below
pass on the installed dataset. `list_tasks()` reports the actual installed task
instances. Task scenes load task-relevant objects, as in the upstream example.
`max_episode_steps` defaults to 30,000 so continuous physical primitives can finish;
this differs from the upstream task’s 500-step benchmark default and is reported
by `describe()`. The navigation blueprint rejects task load/reset because its map belongs
to its current scene; restart that blueprint to reset both simulator and mapping.

## Combined navigation and manipulation

Build the native navigation modules from this checkout (also refreshes existing binaries):

```bash
cargo build --release -p dimos-voxel-ray-tracing -p dimos-mls-planner
```

Rust and Python use Zenoh 1.10.1 or newer for loopback peer discovery. Version
1.10.0 has an [upstream gossip-discovery regression](https://github.com/eclipse-zenoh/zenoh/pull/2755)
that can leave the native planner disconnected from the mapper.

```bash
env -u WAYLAND_DISPLAY dimos --rerun-open native run behavior-r1pro
# Repeatable verification without a desktop window:
python -m dimos.simulation.behavior.demo_r1pro --headless --report /tmp/r1pro.json
```

The blueprint opens the native Isaac Sim viewport, the dimOS Rerun viewer, and
the existing manipulation Viser server (its URL appears in the startup log).
Unsetting `WAYLAND_DISPLAY` avoids a native viewer hang on the tested Linux desktop.
Rerun shows the fused voxel map, traversable surface, robot heading, current goal
and path beside the three cameras. Duplicate clouds and unused map layers are
not logged; map and transform displays update at 2 Hz, camera views at 5 Hz,
and viewer memory is capped at 4 GB. These display limits do not change mapping
or control rates.
Long recordings can still trigger Rerun memory-pressure warnings and display lag;
the viewer's memory cap bounds retained history rather than guaranteeing latency.

For navigation and base teleop in the **native dimOS viewer**:

1. Wait for the map and camera images to appear, with the timeline following live data.
2. Click a mapped floor point in the **Navigation** view to send a goal. Pick the
   traversable floor, rather than a wall, tabletop, camera image, or empty background.
3. Watch the goal marker and path, and the robot moving in Isaac.
4. Click the **Keyboard Teleop** overlay to engage it. W/S drive forward/backward,
   A/D turn, and Q/E strafe. Manual input cancels navigation; releasing keys stops
   manual motion. Space or the overlay's **STOP** control cancels and stops movement.
5. Click the overlay again to disengage keyboard capture, then click a new floor
   goal to resume navigation. The old goal does not resume automatically.

Use `--viewer none` to omit Rerun. The verification demo's `--headless` flag
suppresses both native windows.

Viser exposes the torso and both arm planning groups. Preview a target before
executing it. Base navigation uses
KronkNav: head and wrist RGB-D clouds → robot self-filter → ray-traced voxel map → MLS planner → local planner →
holonomic controller → simulator velocity commands. Each cloud retains its own
optical frame and ray origin; the wrist cameras cover floor hidden from the head
camera. The planner consumes the fused global map so each camera’s local bounds
do not discard the other views. In `dimos shell`, use
`BehaviorR1ProBridge.set_goal(x, y, z)` to send a world-frame floor goal.
Restart the stack to change the task or scene; its navigation map belongs to one episode.

The robot and controllers come from the pinned upstream `eval/r1pro.yaml`. Task
selection loads the corresponding **training instance** and its prescribed robot
pose. Scene-only teleop samples a traversable floor position, lets the robot
settle, and requires floor support without furniture contact before accepting it;
`spawn_position` and `spawn_yaw` can select an explicit pose.

The manipulation layer reuses dimOS's R1 Pro planning groups, joint names,
ControlCoordinator, and trajectory task. Its geometry and joint limits come from
the installed simulator assets, whose dimensions differ from the hardware URDF.
The bundled `r1pro_original.urdf` has the same joints as the processed URDF and
complete mesh references. Measured base pose and gripper positions update the
planning model; they are not executable manipulation groups.
Planning uses dimOS's selected-joint RRT-Connect with RoboPlan collision queries.
This keeps unbounded measured base translation outside the arm search space;
the native RoboPlan RRT interprets omitted prismatic limits as zero bounds.

The verification script checks end-effector FK against simulation (1 cm / 2°),
navigation of at least 0.5 m to within 0.2 m with a stationary finish, planned
motion on each arm with at most 0.05 rad joint error, and cancellation followed by
a measured hold. It writes a JSON report on success or failure. `--goal X Y Z`
selects a reachable goal instead of the default 0.8 m forward target.

This is a development integration, not a benchmark score. Navigation currently
uses simulator ground-truth localization. Official evaluation will need estimated
localization from the permitted observations; global pose and object state must
not become policy inputs. The initial arm proof checks robot self-collision; the
navigation voxel map is not yet connected to the manipulation collision world.
It does not establish collision-safe pick-and-place in arbitrary clutter.

### Browse scenes and tasks

- [Scene gallery](https://behavior.stanford.edu/behavior_components/scenes.html)
- [Scene inventory](https://behavior.stanford.edu/knowledgebase/scenes/index.html)
- [Challenge task videos](https://behavior.stanford.edu/challenge/tasks/index.html)
- [Official evaluation rules](https://behavior.stanford.edu/challenge/evaluation.html)

## Public interface

`dimos shell` exposes the running module's RPCs. `describe()` reports joint names,
limits, controller action slices, cameras, primitive capabilities, and grasping mode.

| Interface | Meaning |
|---|---|
| `cmd_vel: Twist` | Body-frame x/y velocity (m/s), yaw velocity (rad/s) |
| `joint_command: JointState` | Named absolute joint positions; radians for revolute joints, meters for prismatic joints |
| `native_action: list[float]` | Complete action vector with layout and limits from `describe()` |
| `color_image`, `depth_image`, `camera_info` | Head RGB, plane depth in meters, calibration |
| `left_wrist_*`, `right_wrist_*` | Wrist RGB/depth/calibration |
| `joint_state`, `odometry`, `tf` | Measured joints, world pose, world→base→optical transforms |
| `registered_scan`, `left_wrist_scan`, `right_wrist_scan` | Optional RGB-D clouds in each camera’s optical frame; navigation maps all three on one input channel |
| `semantic_image` | Optional semantic IDs, 16-bit image |
| `status` | Runtime state, control owner, episode evaluator result, operation, achieved loop frequency |

Optical coordinates are +X right, +Y down, +Z forward. Ground-truth
RPC responses carry episode and step tags. Ground truth is privileged simulator
information, and is explicitly labeled as such in agent tools.

### Control ownership

Call `take_control("dimos" | "native" | "primitive")` and wait for its operation to
succeed before sending commands. Ownership covers the whole robot. Inactive
streams are discarded. Takeover cancels the active primitive and clears pending
commands. Direct joint targets hold their positions; base velocity and native action
commands expire after `command_timeout` (default 0.2 seconds).

`start_primitive(kind, primitive, target)` requires primitive ownership and returns
an operation ID. `kind` is explicitly `physical` or `symbolic`. Physical mode exposes
GRASP, PLACE_ON_TOP, PLACE_INSIDE, NAVIGATE_TO, and RELEASE. OPEN/CLOSE/TOGGLE are
unsupported physically in the pinned upstream implementation. Symbolic operations
may change object poses and states directly. There is no automatic fallback.
Upstream primitives use R1 Pro's default arm; direct named-joint control covers both arms.
The pinned evaluator configuration uses assisted grasping, recorded in each episode.

Primitives retain ownership when finished, cancelled, or failed. `stop_motion()`
cancels motion and holds in primitive mode. Resume direct control with an explicit
takeover. A second ordinary operation while one is running fails with `busy`.
Pause, reset, load, and takeover may interrupt an operation. Cancellation takes effect
at an engine boundary; a long synchronous upstream planner call cannot be preempted.

`load_task(TaskSelection(...))`, `reset_task()`, `pause()`, and `resume()` return
operation IDs too. Poll `get_operation(id)` for succeeded/failed/cancelled results.
`get_status()` retains the terminal BEHAVIOR result until reset. Primitive completion
and evaluator success are separate fields. Reset creates a new episode and clears
commands. Execution kinds record whether physical and/or symbolic actions ran.

## Verification demos

These scripts use public streams and RPCs and exit nonzero on failure. Run them in
sequence after setup. `--report` saves the observed results as JSON.
Motion checks use simulation time, with a 120-second wall-clock timeout, so
first-use Torch compilation does not prematurely end a movement check.

```bash
python -m dimos.simulation.behavior.demo_integration sensors --report /tmp/behavior-sensors.json
python -m dimos.simulation.behavior.demo_integration handoff --report /tmp/behavior-handoff.json
python -m dimos.simulation.behavior.demo_integration task --kind physical --report /tmp/behavior-physical.json
python -m dimos.simulation.behavior.demo_integration task --kind symbolic --report /tmp/behavior-symbolic.json
# Combined navigation and manipulation; optionally supply --goal X Y Z.
python -m dimos.simulation.behavior.demo_r1pro --headless --report /tmp/r1pro.json
```

| Demo | Required evidence |
|---|---|
| sensors | All three cameras with depth/calibration, joints, odometry and TF; base moves ≥2 cm; both arms/grippers reach named targets; native actions move and expire |
| handoff | Conflicting stream cannot steal primitive control; explicit takeover cancels; held robot does not replay old velocity; fresh command moves |
| task | All three cans placed inside the bin; evaluator success; reset and repeat with a distinct episode ID |
| R1 Pro | Navigation ≥0.5 m to within 0.2 m, stationary finish, FK agreement, both arms, and cancellation hold |
| agentic | `dimos agent-send` can inspect and attempt the task through MCP; LLM completion is exploratory, not an acceptance gate |

Agent example: `dimos agent-send "Inspect the task and ground truth, take primitive control, then use physical actions to put all soda cans into the trash bin. Poll each operation and report the evaluator result."`

CPU tests exercise admission, cancellation, terminal-state retention, command expiry,
and failure reporting without importing Isaac Sim. They do not establish camera
calibration, physical grasp success, or task-fixture compatibility. Those claims
require the live demos and their reports.

### Development-only stationary radio interaction

The experimental `demo_radio` composition connects the Python manipulation SDK
to arm trajectories and one selected coupled gripper. It has no locomotion task.
Run the measured motion check before trying contact:

```bash
mkdir -p radio-development/evidence
python -m dimos.simulation.behavior.demo_radio --stage motion --report radio-development/evidence/radio-motion.json
python -m dimos.simulation.behavior.demo_radio --stage serve --report radio-development/evidence/radio-ready.json
```

`serve` leaves the coordinator running so a separate Python policy can use
`Dimos.connect()` and `Arm.from_app(app, group="left_arm")`. Dimcode's ordinary
write/edit/bash tools can write that policy and execute it with the selected
dimOS environment's Python; this path needs no MCP wrapper. An SDK error is
feedback for the policy, not proof that the remote motion has stopped. Confirm
cancellation before dispatching a recovery motion.

The contact helper accepts a world-frame **gripper-link** target and approach
direction, performs pose/linear SDK motion, holds for at least five simulator
steps, then retracts. It never sets `ToggledOn` symbolically. Press mode requires
a JSON target containing `position`, `approach`, optional XYZW `orientation`, and
explicit `provenance`, for example `development oracle-assisted geometry`.

```bash
python -m dimos.simulation.behavior.demo_radio --stage press --target /tmp/radio-target.json --report radio-development/evidence/radio-press.json
```

These commands are development experiments, not validated task-performance
claims. Finger contact geometry and reachability still need live verification.
An optional `--spawn-position X Y Z --spawn-yaw RADIANS` changes only the starting
robot pose after restoring the official task instance; the report marks this
custom setup and retains the official goal. SDK motion completion and the BDDL
evaluator result remain separate. Truth-derived targets must be reported as
oracle-assisted. The development environment does not enforce a truth boundary.

The fixed script establishes actuation and contact plumbing. A first useful
agent comparison keeps the same task but changes a reachable target pose or
returns a recoverable planning failure, then checks whether the policy adapts
from sensor observations and SDK feedback. Repeating the fixed script proves
code execution only.

Current development validation loaded official `turning_on_radio` instance 0.
RRT-Connect rejects Cartesian paths, so this composition selects RoboPlan.
A native URDF parser treated omitted planar-base position bounds as zero and
clamped the frozen base during Cartesian planning. Explicit parser-compatible
bounds fix that contract; equivalent translated robot, target, and obstacle
regressions pass with the base frozen and only the arm group selected.

At the official robot spawn, the SDK measured gripper closing and opening, then
executed a 10 mm Cartesian excursion and return: 9.806 mm displacement and
0.194 mm return error. This verifies development plumbing, not radio task
completion. The BDDL evaluator remained false. A custom near-field physical press attempt failed precontact IK before
contact; its evaluator stayed false. Agent adaptation remains untested. Scene obstacles are not yet
registered automatically in the planner world; the simulator still enforces
physical scene contact.

The radio development composition disables PyTorch compilation before importing
the simulator, using its supported `TORCH_COMPILE_DISABLE=1` environment option.
First-use compilation can block the simulator's single stepping/feedback thread.
In the torso-assisted smoke, the trajectory clock completed while feedback was
stale; the subsequent press could not plan, and shutdown then logged a C++
compilation error. Replaying the same compiler command on CPU succeeded in
about one second, so a compiler defect has not been established. Pose-level
steps now require fresh measured FK at the target before continuing. The latest
physical attempt still reported BDDL false and zero consecutive toggle-contact
steps. These development results do not establish fair benchmark performance.

With compilation disabled and fresh measured FK required, the SDK completed
precontact, a 30 mm press, eight hold steps, and retraction. All three measured
poses were within about 1 mm of their targets, but BDDL remained false. The radio
moved about 17 cm and tipped over. The final toggle-contact counter was zero;
before/after snapshots alone cannot establish the contact history or which
motion displaced it. Stage evidence now records precontact, press, hold, and
retraction separately, with episode/step tags and optional privileged geometry.

The accepted `wxnicr` radio marker matches the measured object's local button
coordinates. It lies near the collision box's +X face, whose world outward normal
is approximately `(-0.050, 0.997, 0.051)`. The earlier +Y approach points outward,
so it can traverse the body before reaching that face. This is a face-normal
inference from bounding geometry; the spherical marker supplies no intrinsic
press normal. A revised candidate approaches from outside that face, after a
collision-checked clearance motion, with shallow contact travel. Friction, mass,
object constraints, and official task goals remain unchanged. Whether a separate
physical stabilization action is needed has not been demonstrated.

A temporary official Dimcode `0.1.0-next.7` installation completed a localhost
mock-provider tool loop: write a Python policy file, execute it with the selected
Python interpreter, import `dimos.manipulation.sdk.Arm`, and return API signatures
as tool feedback. This verifies Dimcode's code execution path without MCP,
provider credentials, external model calls, or simulator execution. It does not
verify genuine model planning or autonomous task performance.

### Tested environment and results

The integration was validated on an RTX 3090 with NVIDIA driver **590.48.01**,
Isaac Sim 5.1, and OmniGibson 3.9.2. The combined demo used training instance 0
of `picking_up_trash` in `house_double_floor_lower`.

| Check | Observed result |
|---|---|
| Three RGB-D cameras, calibration, joints, odometry, TF | Sensor demo passed in `Rs_int` |
| Direct and native base commands | 0.090 m and 0.152 m motion; stable height and expired-command hold |
| Combined navigation | 0.655 m displacement; 0.145 m goal error; stationary for 2 s |
| End-effector FK | About 5.3 mm position error before and after navigation |
| Planned left / right arms | 0.00025 / 0.00198 rad maximum joint error |
| Cancellation | Confirmed `ABORTED`; 0.000008 rad maximum hold drift |
| Native viewer navigation and teleop | Click goal, manual takeover, stationary hold, and fresh goal passed |

These checks establish the navigation/manipulation connection, not full task
completion or benchmark performance. The reference physical and symbolic task
runs did not reach evaluator success: physical grasp planning failed to find an
accessible path, and symbolic placement exhausted its samples.

### Renderer startup troubleshooting

Driver 610.57.04 caused renderer crashes in `librtx.scenedb.plugin.so`, including
with bare Isaac Sim outside dimOS. Driver 590.48.01 passed locally; this is a tested
configuration, not an upstream certification. See
[Isaac Sim issue #651](https://github.com/isaac-sim/IsaacSim/issues/651) for upstream
driver compatibility guidance. Pixi supplies userspace libraries and CUDA tools;
it does not replace the host NVIDIA kernel driver.

## Runtime development

The host contract stays in `dimos/simulation/behavior`; the isolated project lives
in `native/python/behavior`. Setup and startup share a cached Python environment.
The setup marker, Pixi toolchain, downloads, and default assets belong to the project.

Run mocked runtime tests without installing Isaac Sim from the repository root:

```bash
PYTHONPATH="$PWD:$PWD/native/python/behavior" .venv/bin/pytest --confcutdir=native/python/behavior native/python/behavior/dimos_behavior/test_runtime.py
```

The development radio press now requires an explicit `collision_scene` in its
oracle-assisted target JSON. It installs matching radio/support boxes in the
runtime planner and checks the actual materialized trajectory in a second
planning world. The previous inside-body target is rejected. Intended contact
permits only the selected finger/radio pairs; the static radio/support pair is
permitted because their conservative boxes intersect at the support surface.
These planning exclusions do not alter simulator physics.

Every action retains the returned SDK plan ID. The coordinator's previous
command can anchor the first waypoint, so the checker validates that effective
trajectory too. Dispatch consumes a one-use authorization of the original and
effective trajectory digests; a changed path, anchor, or expired authorization
is rejected. Robot state and fresh, episode-tagged object pose snapshots are
checked again before authorization. The radio/support snapshot is held fixed
in the planning worlds, not physically frozen in the simulator. If the object
moves, the next action rejects the old snapshot instead of silently retargeting.

This is development validation with privileged geometry. It checks linearly
interpolated command edges at a 0.01 configuration-space step against robot/self
and radio/support boxes. Other scene objects, continuous collision clearance,
tracking deviations, and object motion during execution remain outside that
claim. Candidate boxes have a recorded 1 mm margin per face for asset scale
rounding; this margin changes only the planning geometry. Per-stage physical
pose/contact logs and independent BDDL results remain the actual trial evidence.

The first live checked trial exposed a guard-contract error before any arm
motion: the checker substituted fresh measured feedback for every first
waypoint, while JTT only substitutes cached commanded positions. Saved encoder
snapshots drifted by at most 0.000000928 rad, sufficient to change the erroneous
hash although the task would retain its stored first waypoint. The guard now
mirrors only cached substitutions, keeps an independent validation snapshot,
and separately rejects measured start deviations above 0.002 in the selected arm/torso joint coordinates. Actual path or cached-command changes still reject
by digest. Future reports capture both prepared and dispatched command payloads,
their numeric diff, and start errors. The failed trial did not record the full
pending trajectory; its CPU reconstruction uses saved feedback and an explicitly
synthetic endpoint, rather than claiming recovery of that missing payload.

One bounded development trial with the corrected guard completed the official
`turning_on_radio` instance-0 goal in `house_double_floor_lower`. This used the
explicit custom robot base `[3.6, 4.15, 0.005]`, yaw pi/2, an oracle-assisted
finger-tip target, and arm/torso motion through the Python SDK. Four generated
and dispatched trajectory payloads matched exactly. The radio stayed in place
through clearance and precontact, then displaced approximately 6.9 mm during
contact. The runtime recorded five consecutive finger-contact steps, changed
`ToggledOn` from false to true, and independently terminated with BDDL success
at step 402. No object freezing, physics parameter change, or symbolic toggle
was used. The episode ended during the hold, so retraction and the complete
motion sequence were not reported as completed. This is a single development
baseline, not autonomous agent or fair-evaluation performance. The recorded
head-camera stream is genuine, but its framing does not directly show the
button interaction; contact/pose logs provide that trial's diagnostic evidence.
