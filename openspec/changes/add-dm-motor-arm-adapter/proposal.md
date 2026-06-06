## Why

DimOS already has OpenArm support, but the current adapter owns a custom Python CAN driver because there was no stable Python SDK surface for Damiao/OpenArm motors. The `can-motor-control` Python package provides a Rust-backed control library with a Python API for building robots, opening SocketCAN buses, ticking control loops, reading motor state, and commanding arm groups. Using the Python binding keeps DimOS integration in the existing Python manipulator adapter layer while avoiding a direct Rust integration in this change.

This change also needs built-in gravity compensation for DMMotor-based arms. The existing OpenArm adapter uses model-based gravity compensation inside MIT commands; the new adapter path should follow that pattern in-place through an adapter flag, applying feed-forward gravity torque without introducing a separate gravity-compensation module.

## What Changes

- Add a new DMMotor manipulator adapter behavior that uses the `can_motor_control` Python binding API rather than calling Rust crates directly.
- Add support for configuring DMMotor arm hardware through the existing manipulator adapter registry and ControlCoordinator hardware configuration path.
- Add an adapter-level gravity-compensation operating path for DMMotor arms that sends gravity feed-forward torque while keeping the behavior opt-in/configurable through adapter kwargs.
- Preserve existing `openarm` adapter behavior and blueprints unless explicitly migrated later; this change does not silently replace the current OpenArm custom CAN adapter.
- Add the published `can-motor-control` package to the manipulation extra so environments selecting the adapter can install it through `dimos[manipulation]`.
- Mark hardware safety behavior explicitly: the adapter must stop or disable safely on shutdown and must avoid unintended stiff position-hold behavior in gravity-compensation-only commands.

## Affected DimOS Surfaces

- Modules/streams: Manipulator adapter behavior behind `ManipulatorAdapter`; ControlCoordinator read/write behavior through existing `joint_state` and command routing surfaces.
- Blueprints/CLI: New DMMotor/OpenArm-style hardware blueprint entry point for coordinator use through `dimos run` once registered.
- Skills/MCP: No direct skill or MCP tool changes planned for this proposal.
- Hardware/simulation/replay: Real SocketCAN DMMotor/OpenArm hardware bring-up; mock/vcan validation through the `can_motor_control` Python binding where available; no replay changes planned.
- Docs/generated registries: Manipulation/OpenArm documentation, blueprint registry generation if new runnable blueprints are added, and adapter registry discovery behavior.

## Capabilities

### New Capabilities

- `dm-motor-manipulator-adapters`: Covers DMMotor arm adapter behavior through the Python binding, including lifecycle, state reads, command writes, binding availability assumptions, and safe shutdown.
- `gravity-compensation-control`: Covers gravity-compensation-only behavior for manipulators, including operator-visible expectations that joints remain free to move while gravity torque is compensated.
- `manipulation-stack`: Covers manipulation-stack integration behavior for DMMotor hardware through DimOS blueprints and coordinator-compatible surfaces.

### Modified Capabilities

- None.

## Impact

Users gain a new path for DMMotor/OpenArm-style hardware that relies on the `can-motor-control` Python package instead of maintaining another in-tree low-level CAN implementation. Developers can keep the integration inside the established Python adapter registry and ControlCoordinator flow, while future design work can decide when or whether existing OpenArm blueprints should migrate.

Compatibility risk is primarily around hardware safety, binding availability, joint ordering, tick timing, and gravity compensation semantics. The dependency is exposed through `dimos[manipulation]`, and selecting the adapter still fails explicitly if `can_motor_control` is not importable. Documentation and QA must cover mock/vcan validation, one-motor bring-up, full-arm state monitoring, adapter gravity-compensation behavior, and shutdown/disable behavior on interruption.
