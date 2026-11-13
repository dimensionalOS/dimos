# R1 Pro teleoperation and collection backend

The backend uses one coordinator motion policy for JoyLo, browser joint commands,
Cartesian/IK tasks, and policy tasks. Clients select targets and authority; the
coordinator arbitrates, bounds motion, and dispatches commands. Galaxea's hardware
controllers still execute them. The hardware blueprint includes CC's WebXR IK
and joystick tasks alongside leader input, using one selected teleop source.
This does not install WIM.

## Frontend integration contract

Discover modules and invoke their existing RPC interface. Do not put a second
physical speed limiter in the frontend.

| Module | RPC | Meaning |
| --- | --- | --- |
| `ControlCoordinator` | `get_motion_limits()` | Read named groups, joint names, live `rate`, deployment `ceiling`, `unit`, and `label`. |
| `ControlCoordinator` | `set_motion_limits(rates)` | Atomically update any subset, e.g. `{"arm_rate": 0.75, "base_speed": 0.3}`. Returns all groups. Unknown names, nonfinite values, and values outside `0..ceiling` fail without applying any change. |
| `ControlCoordinator` | `set_estop(estopped)` | Existing emergency-stop latch. Clearing requires fresh connection feedback and does not resume cancelled trajectories. |
| `ControlCoordinator` | `get_teleop_source()` | Read `source` (`leader` or `webxr`) and `webxr_ready` (headset clutches have been released since selection). |
| `ControlCoordinator` | `select_teleop_source(source)` | Select `leader` or `webxr`, finish the current tick, clear old task targets and holds, pause JoyLo, and preserve any existing estop. Selection is idempotent. |
| `R1ProJoylo` | `status()`, `pause()` | Read leader/Joy-Con health and authority; pause requires a fresh physical enable press to rearm JoyLo. |
| `episodes` | `command(event)` | `start`, `save`, `discard`, or `toggle`. Returns `EpisodeStatus`. Included in the normal hardware blueprint. |
| `episodes` | `get_status()` | Read recording state, saved/discarded counts, last event, timestamp, and task label. |

`motion_settings: String` publishes the full group dictionary as JSON at startup
and on accepted updates. Read the RPC on app attachment, then subscribe for changes.
`status: String` from `episodes` carries versioned `EpisodeStatus` JSON. Saving marks
an episode boundary; it does not export a dataset or certify demonstration quality.
An app should show backend failures instead of optimistically claiming recording
has started. Repeated `start` saves the preceding take, as in the existing collector;
use `get_status()` for reconnects, not a new `start`.

The existing `R1ProJoylo.get_motion_speeds()` and
`set_motion_speeds(arm_rate=..., torso_rate=..., base_speed=..., yaw_speed=...)`
remain compatibility wrappers around the coordinator. Existing Float32 speed input
ports remain available. New apps should use the coordinator RPCs directly.

The hardware profile starts at 0.75 rad/s for arms, 0.3 rad/s for torso,
0.3 m/s for base translation, 0.6 rad/s for yaw, and 0.3 m/s for total gripper
opening. Deployment ceilings are respectively 1, 0.5, 0.5, 2, and 0.3 in those units.
Sim2 uses its own declared arm/torso ceilings. Speed changes never enable motion.
A zero rate holds the position command or caps velocity at zero; use estop for a
stop latch. Joy-Con stick-click cannot exceed the selected drive limit.

## Command processing

Position targets slew from the last dispatched position, with fresh measured state
as the initial anchor. Limits apply after arbitration and therefore survive task
handoffs. Torso joints scale their step together to preserve the requested joint
ratios. Velocity commands are capped directly. This is velocity limiting, not a
new acceleration/jerk planner. IK's input filtering, trajectory timing, Sim2's servo,
and the vendor's low-level behavior remain separate concerns.

The trajectory task delegates slew to the coordinator for configured joints and
stays active until its endpoint has actually been dispatched. It retains its old
behavior for robots without central motion groups. Missed ticks cannot accumulate
an arbitrarily large step. Feedback outages and estop invalidate the limiter's
anchor; guarded leader commands still expire. Existing enable, pause, calibration,
and discontinuity guards remain in the input adapter. Arm, torso, and gripper slew
are no longer duplicated in JoyLo.

`coordinator_joint_command: JointState` contains dispatched position commands,
including connection holds and clamped values. `coordinator_joint_velocity:
JointState` contains dispatched velocity commands for connection-backed robots.
They are not hardware acknowledgements. `coordinator_joint_state` contains measured
state and stops emitting fresh aggregates while required connection feedback is
stale. Recorder actions use command streams; observations use measured state.

## Collection deployment

Recording is included in the distributed `r1pro-openarm-joylo-real` blueprint.
Keep the existing robot host selection and operator device settings, and add a new,
absolute **robot-side** recording directory and a task description at launch:

```bash
dimos run r1pro-openarm-joylo-real \
  --recorder.recording /home/nvidia/recordings/pick-bottle-001 \
  --episodes.task "pick up the bottle"
```

Both options are required; no directory is overwritten or task label invented.
The old `r1pro-openarm-joylo-collect` name remains an equivalent entrypoint.
No separate recorder blueprint or `--record` process is needed.

The recorder observes the coordinator's dispatched actions, measured robot state,
and camera outputs, independent of the selected device. OpenArm/Joy-Con and WebXR
are connected to the same coordinator; switching devices does not restart recording.

## WebXR and leader selection

`r1pro-openarm-joylo-real` starts with `leader` selected. To start with WebXR,
add `--ControlCoordinator.teleop-source webxr`. The built-in HTTPS headset page is
served by `R1ProWebXR` on the **operator laptop**, at
`https://<operator-address>:8443/teleop` (configurable with
`--r1prowebxr.server-port`). Add `--g.listen-host 0.0.0.0` when the headset must
reach that server from another machine. The existing `webxr_pose`/`webxr_joy`
topic inputs also accept packets from a custom spatial frontend.

For a headset session without USB leaders or Joy-Cons, additionally pass
`--disable r1-pro-leader-connection --disable joy-con-teleop-module`; keep `R1ProJoylo`
in the blueprint so its pause and existing speed compatibility APIs remain available.

An app can switch sources with `ControlCoordinator.select_teleop_source("webxr")`
or `"leader"`. The switch clears pending browser/task targets as well as device
intent, and recaptures holds from measured feedback. It does not clear estop.
Returning to the leader requires a fresh Joy-Con enable press. Selecting WebXR
requires releasing both grips and the right-stick click before engaging again;
the unselected device's streams cannot overwrite targets. Releasing or losing
WebXR input leaves WebXR selected, without automatically returning to the leader.

Headset controls use CC's mapping:

- Hold both middle-finger grips to steer the two arms; engagement captures the
  current robot pose, then tracks controller displacement.
- Index triggers close the corresponding gripper while its grip is held.
- Hold right-stick click for body motion: left stick translates, right X turns,
  right Y raises/lowers the torso. Release the click to stop requesting body motion.

Head RGB is forwarded to the headset using the original compressed JPEG bytes.
The existing Desktop head/wrist RGB-D and three-view RGB recording remain connected.
Arm/base speeds use the same coordinator settings as leader/browser control;
the R1 joystick mapper updates physical stick scaling when those settings change.
CC's IK filtering, tracking-error guard, and slow torso-pitch preference remain.
The combined hardware stack uses the pinned 2025 kinematics and 0.40/0.30 m lift
links, matching the existing robot model, rather than CC's standalone 2026 defaults.
His standalone VR entrypoints remain available separately; do not launch one
alongside this combined blueprint against the same robot.

## Recording contents

The recorder and episode monitor live beside the robot control/cameras. The recorder
precedes producers so it starts first and stops last. Operator network loss does not
relocate recording to the laptop. Native capture uses the existing Rust recorder;
the robot needs its executable/build dependencies. Its standard build is
`nix build -L path:.` in `dimos/experimental/memory/rust`; deployments with a prebuilt
binary can configure `recorder.executable` and disable `recorder.build_command`.
This change does not start or provision that process on the physical robot.

The collection directory contains `schema.json` and `recording.mcap`. Compressed
camera messages retain their existing encoded bytes; they are decoded to RGB during
export. No live resolution reduction or extra live JPEG encoding is introduced.
Auxiliary live motion settings are also recorded. `schema.json` records actual
transport names in `metadata.recorded_streams`, including run namespaces.

Default camera requirements:

| Feature | Source | Shape (H, W, C) |
| --- | --- | --- |
| `observation.images.head` | Spatial aligned head color | 384 × 480 × 3 |
| `observation.images.left_wrist` | Left wrist compressed color | 720 × 1280 × 3 |
| `observation.images.right_wrist` | Right wrist compressed color | 720 × 1280 × 3 |

The default profile requires all three cameras. A missing wrist must not silently
produce a nominally complete dataset. Python compositions can explicitly select a
subset with `r1pro_collection(cameras=("head", "right_wrist"))`; the selected views
are persisted in the schema. The default spatial streams run at 5 Hz, and the dataset
uses 5 Hz. Independent camera phases permit 110 ms alignment error; gaps over 450 ms
or insufficient source rate fail validation. Commands older than 200 ms also fail.
These settings are recorded explicitly rather than inventing 30 Hz observations.

`episodes.command("start")` begins a take; `save` retains it; `discard` excludes it.
Stopping during a take leaves it incomplete and excluded. Raw capture continues
throughout the session, including idle intervals. Dataset preparation includes only
saved episodes that pass quality validation.

## Dataset and policy contract

```bash
dimos imitation inspect /path/to/pick-bottle-001 --json
dimos imitation prepare /path/to/pick-bottle-001 --output /path/to/pick-bottle-dataset
```

These commands consume the portable recording directory; no running robot or UI is
required. Preparation uses DimOS's isolated LeRobot runtime. HDF5 is also available
through `RecordingSchema.dataprep_config(..., OutputConfig(format="hdf5", ...))`
and `run_dataprep()`. `dimos imitation train` forwards training arguments to the
existing LeRobot training entry point; choose a policy and train on collected data.

The policy observation is 20 measured positions plus the selected RGB views. The
23-element `action` vector is ordered exactly as saved in the feature schema:

| Indices | Values | Units |
| --- | --- | --- |
| 0–3 | Torso joints 1–4 | rad |
| 4–10 | Left arm joints 1–7 | rad |
| 11–17 | Right arm joints 1–7 | rad |
| 18–19 | Left/right total gripper openings | m |
| 20–22 | Body forward, lateral, yaw velocities | m/s, m/s, rad/s |

Position and velocity messages remain separate raw streams with honest message
semantics. Offline preparation takes the most recent action at or before each
observation timestamp and packs them in the schema's explicit `action_vector`
order. Future commands cannot leak into earlier samples. `dimos_meta.json` retains
the source projections, packing order, quality results, and episode metadata.

For a future rollout adapter, send the first 18 positions through the coordinator
joint target input, normalize gripper openings by the R1's 0.1 m opening range for
its existing left/right Float32 gripper ports, and map the last three entries to
Twist linear x/y and angular z. Do not publish directly
to Galaxea or bypass the existing command watchdogs. This change supplies the data
and shared motion backend, not a trained R1 policy or automatic policy execution.

## Validation

Tests cover shared limits, task completion, feedback loss, estop, accepted-command
recording, JoyLo mapping/authority, compressed-camera decoding, causal base commands,
missing/stale source rejection, and exclusion of discarded/interrupted takes.
Synthetic R1 MCAP is exported to HDF5 and to real LeRobot data; LeRobot's training
loader verifies the 23-element action, joint ordering, image shapes, and 5 Hz rate.
Physical capture performance and driving behavior still need the next supervised
hardware session. No physical robot or simulator is started by these tests.

The broader control suite also has two pre-existing navigation benchmark failures
(`test_end_to_end_controller_benchmark_scoring` and
`test_end_to_end_fullpose_benchmark_scoring`); both reproduce with the committed
pre-change task/state definitions. The new control, recording, registry, and type
checks pass. Native recorder process integration has an existing macOS skip;
physical robot deployment/capture was not exercised in this backend change.
