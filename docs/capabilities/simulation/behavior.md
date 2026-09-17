# BEHAVIOR / OmniGibson

BEHAVIOR runs an R1 Pro mobile manipulator in an isolated Python process. DimOS owns
blueprint composition, visualization, navigation, and agent tools. OmniGibson owns
physics, cameras, task evaluation, and physical/symbolic action primitives.

```text
DimOS host (NumPy 2, Rerun)
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
current DimOS checkout (or exactly the installed host version) with `--no-deps`.
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
dimos run behavior-r1pro
# Repeatable verification without a desktop window:
python -m dimos.simulation.behavior.demo_r1pro --headless --report /tmp/r1pro.json
```

The blueprint opens the native Isaac Sim viewport and the existing manipulation
Viser server (its URL appears in the startup log). Viser exposes the torso and both
arm planning groups. Preview a target before executing it. Base navigation uses
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

The manipulation layer reuses DimOS's R1 Pro planning groups, joint names,
ControlCoordinator, and trajectory task. Its geometry and joint limits come from
the installed simulator assets, whose dimensions differ from the hardware URDF.
The bundled `r1pro_original.urdf` has the same joints as the processed URDF and
complete mesh references. Measured base pose and gripper positions update the
planning model; they are not executable manipulation groups.
Planning uses DimOS's selected-joint RRT-Connect with RoboPlan collision queries.
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

Optical coordinates are +X right, +Y down, +Z forward. Observations and ground-truth
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
# Choose a reachable floor point from the scene; example coordinates are not assumed.
python -m dimos.simulation.behavior.demo_integration navigation --goal X Y Z --report /tmp/behavior-nav.json
```

| Demo | Required evidence |
|---|---|
| sensors | All three cameras with depth/calibration, joints, odometry and TF; base moves ≥2 cm; both arms/grippers reach named targets; native actions move and expire |
| handoff | Conflicting stream cannot steal primitive control; explicit takeover cancels; held robot does not replay old velocity; fresh command moves |
| task | All three cans placed inside the bin; evaluator success; reset and repeat with a distinct episode ID |
| navigation | Existing DimOS planner/follower reaches selected goal within 0.3 m in 120 s |
| agentic | `dimos agent-send` can inspect and attempt the task through MCP; LLM completion is exploratory, not an acceptance gate |

Agent example: `dimos agent-send "Inspect the task and ground truth, take primitive control, then use physical actions to put all soda cans into the trash bin. Poll each operation and report the evaluator result."`

CPU tests exercise admission, cancellation, terminal-state retention, command expiry,
and failure reporting without importing Isaac Sim. They do not establish camera
calibration, physical grasp success, or task-fixture compatibility. Those claims
require the live demos and their reports.

### GPU driver validation

The installed environment and assets reached Isaac Sim startup on an RTX 3090
with NVIDIA driver 610.57.04. Both the sensor demo and a standalone OmniGibson
empty-scene launch crashed before scene initialization. A native debugger located
the segmentation fault in `librtx.scenedb.plugin.so` on a renderer worker thread.

Further local checks reproduced the renderer crash with bare Isaac Sim 5.1,
without DimOS or OmniGibson imports; with a clean environment and only `libxml2`
preloaded; and with fresh user settings and NVIDIA-only Vulkan device selection.
An existing native Isaac Sim 4.5 installation also crashed. Basic host CUDA
initialization succeeded. Historical logs confirm that Isaac Sim 5.0 ran on the
same RTX 3090 with driver 580.76.05, before subsequent driver upgrades.

After changing the host driver to **590.48.01**, the same bare Isaac Sim 5.1
startup test completed, ran ten updates, and exited successfully. This removes
the renderer startup blocker. On the preceding R1 integration, the `sensors` demo also passed: three RGB/depth
cameras with calibration, joints, odometry and TF; about 9 cm of Twist-controlled
base motion; both arms and grippers reaching named targets; about 10 cm of native
action motion followed by a stable expired-command hold; and clean shutdown.
That R1 `handoff` demo passed in the task scene: explicit takeover cancelled the
primitive, discarded the inactive command, held position, and accepted a fresh
direct command. Task-camera initialization requires a simulation update after
reset before reading calibration; render-only updates returned empty metadata.
The reference task has **not** passed: symbolic GRASP succeeded, but PLACE_INSIDE
exhausted its placement samples for `trash_can_116`; physical GRASP exhausted five
attempts because the upstream planner found no accessible path to the first can.
Neither run reached evaluator success or the second episode. The physical run
exited cleanly with the simulator's ten-second shutdown allowance.
That earlier R1 navigation stack built and ran, but the planner reported no full path to the
nearby `(0.6, 0, 0)` goal and the 120-second acceptance check timed out. Navigation
also exited cleanly. The R1 Pro results below supersede that navigation check.
The earlier crash matches
[Isaac Sim issue #651](https://github.com/isaac-sim/IsaacSim/issues/651), where
NVIDIA identifies a driver compatibility gap and recommends the R580 branch for
Isaac Sim 5.1. The local 590.48.01 result is a tested alternative on this machine,
not a claim of upstream certification. Installing CUDA or native libraries with
Pixi does not replace the machine's NVIDIA kernel driver.

### R1 Pro navigation and manipulation validation

On 2026-09-16, the combined headless demo passed on the RTX 3090 / 590.48.01
with Isaac Sim 5.1 and OmniGibson 3.9.2. It loaded training instance 0 of
`picking_up_trash` in `house_double_floor_lower` and completed without external
map injection or manual control:

| Check | Measured result |
|---|---|
| Navigation | 0.661 m displacement; 0.140 m goal error; stationary for 2 s |
| End-effector FK | About 5.3 mm position error, before and after navigation |
| Planned left arm | 0.00195 rad maximum joint error |
| Planned right arm | 0.00027 rad maximum joint error |
| Cancellation | Confirmed `ABORTED`; 0.000013 rad maximum hold drift |

A fresh native-viewer repeat also passed: 0.655 m displacement, 0.146 m goal
error, both arms within 0.001 rad, and 0.0000024 rad cancellation hold drift.
The Isaac viewport showed the robot clear of scene geometry, and the Viser
server returned HTTP 200. Both runs used the same combined blueprint.

The R1 Pro scene-only sensor demo also passed in `Rs_int`: all three RGB-D
cameras and calibration, joints, odometry and TF, both arms and grippers,
0.090 m of base-command motion, and 0.157 m of native-action motion. Base height
remained stable, and the expired native command held position. Spawn validation
requires floor support after settling; the demo checks vertical motion as well
as horizontal displacement.

This verifies the navigation/manipulation connection and its control lifecycle.
It does not establish full BEHAVIOR task completion or benchmark performance.
