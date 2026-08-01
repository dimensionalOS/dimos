## Why

The OpenArm Mini leader can now drive OpenArm follower joint commands and a left-side visualization path exists, but there is no right-arm bring-up blueprint that exercises the real OpenArm follower through the normal coordinator/manipulation stack. Operators need a safe right-arm workflow that defaults to mock follower hardware, requires explicit real-hardware opt-in, and uses the standard ManipulationModule Viser visualization instead of a custom renderer.

## What Changes

- Add a right-side OpenArm Mini teleop blueprint that connects a real OpenArm Mini right leader to `ControlCoordinator` and `ManipulationModule`.
- Default the right leader port for this blueprint to `/dev/ttyACM0` while preserving CLI override support.
- Use `--can-port` as the real-follower opt-in: absent means mock right OpenArm follower; present means real right OpenArm follower on that CAN interface.
- Render follower-observed coordinator state through ManipulationModule's Viser backend.
- Use ManipulationModule-compatible global joint names for this blueprint so `coordinator_joint_state` flows directly into the manipulation world monitor.
- Remove the custom OpenArm Mini Viser renderer path from the right-arm bring-up plan and rely on standard manipulation visualization.

## Capabilities

### New Capabilities
- `openarm-mini-right-real-teleop`: Right-side OpenArm Mini leader teleoperation into a mock-or-real right OpenArm follower with ManipulationModule Viser visualization.

### Modified Capabilities
- `openarm-mini-teleop`: The right-arm bring-up blueprint reuses the OpenArm Mini adapter/runtime and may require configurable follower joint-name namespacing for right-side coordinator/manipulation compatibility.

## Impact

- Affected code: OpenArm Mini teleop configuration/module, OpenArm manipulator teleop blueprints, OpenArm hardware/model config usage, blueprint registry generation, manipulation blueprint tests, and OpenArm Mini docs.
- Systems: Feetech leader serial connection, OpenArm right follower CAN connection, ControlCoordinator servo task routing, and ManipulationModule Viser visualization.
- Safety: real follower hardware remains mocked unless `--can-port` is provided; missing right calibration still fails startup.
