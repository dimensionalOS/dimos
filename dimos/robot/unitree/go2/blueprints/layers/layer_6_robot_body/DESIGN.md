# Layer 6 Robot Body / Local Policy Design

This document explains the first Go2 Layer 6 implementation. Layer 6 is the
robot-body and local-policy boundary: hardware connection, replay/simulation
connection, odometry, camera, lidar, mapping, navigation, movement management,
and local safety behavior.

## Current Scope

Layer 6 V1 keeps the existing Go2 robot stack in place. It does not move or
rewrite `GO2Connection`, `VoxelGridMapper`, `CostMapper`,
`ReplanningAStarPlanner`, `WavefrontFrontierExplorer`, `PatrollingModule`, or
`MovementManager`.

The new module is `_Go2RobotBodyState`. It observes existing body streams and
exposes an RPC-only state snapshot for upper layers.

The Layer 6 package entrypoint uses lazy blueprint construction. Importing
`robot_body_spec.py` or `robot_body_state.py` should stay light; the full Go2
robot stack is imported only when a blueprint asks for `_go2_robot_body`.

## Runtime Flow

```text
GO2Connection + mapping/navigation/local policy modules
  -> odom / color_image / lidar streams
  -> _Go2RobotBodyState.get_robot_body_snapshot(...)
  -> Layer 4 _Go2StructuredWorldState.get_robot_state(...)
  -> Layer 3 ContextProvider.get_context(...)
```

`_Go2RobotBodyState` is observational. It does not publish movement commands,
call sport commands, or override local robot safety.

## Module Responsibilities

`_Go2RobotBodyState`

- Tracks whether odom, color image, and lidar streams have produced messages.
- Returns connection mode and runtime connection configuration.
- Returns local-policy settings visible from `GlobalConfig`.
- Returns conservative safety metadata for upper-layer context.
- Does not execute motion, stop the robot, or own durable state.

Existing Go2 robot-body modules

- Continue to own hardware/replay/simulation IO.
- Continue to publish sensor streams and consume command streams.
- Continue to own local navigation, mapping, patrolling, and movement behavior.

## Function Implementation Notes

### `_Go2RobotBodyState.get_robot_body_snapshot(...)`

- File: `layer_6_robot_body/robot_body_state.py`
- Entry point: RPC-only helper.
- Purpose: return one compact Layer 6 body/local-policy state payload.
- Inputs:
  - Cached stream state from `odom`, `color_image`, and `lidar`.
  - Optional injected `GO2ConnectionSpec`.
  - Runtime config from `global_config`.
- Storage:
  - No database.
  - No persistent files.
  - Keeps latest odom and stream counters in memory while the process runs.
- Data shape:
  - `available`
  - `version`
  - `connection`
  - `sensors`
  - `local_policy`
  - `safety`
- Algorithm:
  - Read connection mode from runtime config.
  - Convert latest odom to a JSON-shaped pose when available.
  - Report stream availability through per-stream counters and last-seen age.
  - Report local-policy and safety metadata as conservative context only.
- Current limits:
  - Connection health is inferred from config and injected spec presence, not
    active hardware heartbeat.
  - Safety status is descriptive; physical safety remains in local control.

### `_Go2RobotBodyState.get_sensor_state(...)`

- File: `layer_6_robot_body/robot_body_state.py`
- Entry point: RPC-only helper.
- Purpose: expose stream liveness for odom, image, and lidar.
- Inputs:
  - Stream callbacks `_on_odom`, `_on_color_image`, and `_on_lidar`.
- Storage:
  - In-memory counters and timestamps only.
- Return shape:
  - `odom`, `color_image`, and `lidar` sections, each with `available`,
    `count`, and `last_seen_age_sec`.
  - Odom also includes latest pose data when available.

## Version Boundaries

V1 implemented:

- Add an observational Layer 6 robot-body state facade.
- Add a `RobotBodyStateSpec` so Layer 4 can read body/local-policy state.
- Connect the facade into `_go2_robot_body` without changing public Go2
  blueprint names.

V2 planned:

- Add real connection heartbeat/health if `GO2Connection` exposes it.
- Add local command-stream tracking after confirming it will not interfere with
  movement-manager fan-out.
- Add explicit safety-stop state if a local control module exposes it.

V3 planned:

- Use Layer 6 body state in end-to-end Go2 blueprint tests to verify the full
  Layer 3 -> Layer 4 -> Layer 6 context path.

## Design Rules

- Keep physical actions in existing low-level modules and Layer 5 skills.
- Layer 6 state helpers should be RPC-only, not MCP skills.
- Avoid importing the full Go2 robot stack from lightweight spec modules.
- Do not treat Layer 6 state summaries as physical safety guarantees.
