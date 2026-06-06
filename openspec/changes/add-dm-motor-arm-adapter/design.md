## Context

DimOS currently supports manipulators through the `ManipulatorAdapter` protocol, the manipulator adapter registry, `HardwareComponent.adapter_type`, and the `ControlCoordinator` read/compute/write loop. OpenArm hardware already has an `openarm` adapter that owns a custom Python SocketCAN driver, sets MIT mode, computes Pinocchio-based gravity feed-forward, and preserves existing OpenArm blueprints.

The `can-motor-control` package exposes a Python binding for DMMotor/Damiao hardware. Its Python API owns robot lifecycle and bus ticks through `Robot.connect()`, `Robot.enable()`, repeated `Robot.tick(...)`, group state reads, and arm commands. This change must use that Python binding from DimOS and expose the published package through the manipulation extra.

Gravity compensation should follow the existing OpenArm adapter pattern: it is computed in-place inside adapter command writes and controlled with an adapter flag. The gravity-compensation-only helper method must not act like a trajectory or stiff position-hold controller.

## Goals / Non-Goals

**Goals:**

- Add a DMMotor manipulator adapter path named `DMMotorArm` that uses the `can_motor_control` Python binding.
- Register the adapter under a stable DimOS adapter key, expected to be `dm_motor_arm`.
- Preserve existing `openarm` adapter and blueprint behavior unless a later change explicitly migrates it.
- Provide adapter-level gravity compensation that sends model-based feed-forward torque and can be enabled or disabled with an adapter flag.
- Add the published `can-motor-control` dependency to the manipulation extra; environments selecting the adapter should install `dimos[manipulation]`.
- Document and validate safe hardware bring-up, especially enable/disable, state freshness, and shutdown behavior.

**Non-Goals:**

- Do not call the Rust crates directly from DimOS.
- Do not replace the existing `openarm` adapter registration in this change.
- Do not build or vendor the Rust binding in DimOS; consume the published `can-motor-control` package.
- Do not expose new skills or MCP tools.
- Do not introduce a separate gravity-compensation module or blueprint.

## DimOS Architecture

The adapter should live in the existing manipulator hardware layer:

```text
ControlCoordinator
  -> HardwareComponent(adapter_type="dm_motor_arm")
    -> manipulator adapter registry
      -> DMMotorArm
        -> can_motor_control Python binding
          -> SocketCAN / MockCanBus / vcan-backed DMMotor hardware
```

The `DMMotorArm` class should satisfy `ManipulatorAdapter` for coordinator-compatible use. It should be discovered through the existing manipulator registry hook and created from `HardwareComponent` fields such as `address`, `dof`, `hardware_id`, and `adapter_kwargs`. The adapter module should not import `can_motor_control` at module import time; it should import lazily when the adapter is constructed or connected so unrelated DimOS users do not lose registry discovery when the binding is absent.

The coordinator path should use existing `joint_state` output and command routing. The adapter must reconcile DimOS' separate read/write calls with `can_motor_control`'s queued-command plus `Robot.tick(...)` model by owning tick/cache semantics internally. Reads should return coherent cached positions, velocities, and efforts for one cycle rather than independently ticking three times. Writes should queue commands and flush/update at a controlled point so the CAN bus is not double-loaded.

Gravity compensation lives in `DMMotorArm` rather than a separate module. With `gravity_comp=True`, adapter position writes compute `tau_g(q)` from the current measured joint state and send MIT commands with gravity feed-forward. The adapter intentionally supports position and effort semantics only; velocity control is rejected because nonzero gains would hold the supplied `q` anchor. The adapter also exposes a gravity-compensation-only helper that sends MIT commands with `kp=0`, configurable low/no damping, and gravity torque for direct bring-up tests:

```text
ControlCoordinator / caller
  -> DMMotorArm adapter
    -> read current q/dq through can_motor_control tick/cache
    -> compute tau_g(q)
    -> send MIT position command with adapter gains or effort/gravity-only kp=0
```

This follows the existing OpenArm adapter style and avoids a duplicate lifecycle thread or standalone gravity-compensation blueprint.

No new DimOS `Spec` Protocol is required for this change unless implementation needs RPC injection between modules. No skill/MCP exposure is planned. If new runnable blueprints are added, `dimos/robot/all_blueprints.py` must be regenerated with `pytest dimos/robot/test_all_blueprints_generation.py`.

## Decisions

- **Use the Python binding, not Rust crates directly.** This keeps the integration inside DimOS' Python module/adapter system and matches the user's requested surface.
- **Create a new `dm_motor_arm` adapter instead of replacing `openarm`.** The current OpenArm adapter includes hardware-specific limits, MIT-mode setup, thermal reporting, and gravity compensation behavior. Replacing it would be a silent compatibility and safety change.
- **Expose the binding through `dimos[manipulation]`.** The adapter should clearly fail when selected and the Python package is unavailable, while the manipulation extra should install the published `can-motor-control` package on supported platforms.
- **Use lazy binding import.** Registry discovery should remain healthy even when `can_motor_control` is not installed.
- **Treat `Robot.tick(...)` as adapter-owned.** The adapter should ensure one coherent state snapshot per cycle and avoid ticking once per position/velocity/effort read.
- **Provide gravity compensation in-place through the adapter.** This matches the existing OpenArm implementation and keeps lifecycle/tick ownership inside one hardware adapter.
- **Use model-based gravity compensation with zero position stiffness for effort/gravity-only commands.** Gravity-only helper calls should send feed-forward torque and optional damping, not position targets with nonzero stiffness.

## Safety / Simulation / Replay

Hardware assumptions:

- DMMotor/Damiao hardware is connected through Linux SocketCAN with CAN-FD enabled by default, or a compatible mock/vcan backend supported by the Python binding.
- The Python binding is installed in the active runtime environment, typically through `dimos[manipulation]`.
- Joint ordering in DimOS configuration matches the arm group ordering in the binding/config.
- Gravity compensation model and hardware joint signs/offsets are valid before enabling gravity-only mode on real hardware.

Safety constraints:

- Disable motors on shutdown, interruption, and disconnect.
- Do not auto-install dependencies or silently select a different adapter.
- Do not use position-hold gains in gravity-compensation-only helper commands.
- Start hardware QA at low rates with one motor or mock/vcan before full-arm bring-up.
- Make binding-unavailable failures explicit before attempting hardware access.

Simulation/replay:

- Replay is out of scope.
- Mock/vcan validation through `can_motor_control.MockCanBus` or SocketCAN virtual interfaces should be supported for development.
- Existing `openarm` mock/planner blueprints remain available and unchanged.

Manual QA surface:

- Registry discovery includes `dm_motor_arm` when the adapter module is present.
- Missing binding produces a clear selected-adapter error.
- Mock/vcan adapter cycles read state and accepts commands without hardware.
- Adapter gravity compensation can be enabled/disabled through `gravity_comp`, computes feed-forward torque in command writes, and the gravity-only helper keeps joints manually movable with `kp=0`.

## Risks / Trade-offs

- **Binding availability and API stability:** `can-motor-control` is a new binding. Mitigation: document expected availability through `dimos[manipulation]`, verify import behavior in QA, and keep adapter usage explicit.
- **Tick timing:** The binding flushes queued commands and receives feedback through `Robot.tick(...)`. Mitigation: centralize tick/cache behavior in the adapter and avoid multiple ticks per coordinator cycle.
- **Gravity compensation correctness:** Incorrect model, joint signs, or offsets can push hardware unexpectedly. Mitigation: require mock/vcan and low-rate bring-up, document validation, and avoid position stiffness in gravity-only helper mode.

## Migration / Rollout

1. Add the new adapter and in-place gravity-compensation behavior without changing existing `openarm` registrations.
2. Add opt-in DMMotor/OpenArm-style blueprints using `adapter_type="dm_motor_arm"`.
3. Regenerate blueprint registry if runnable blueprints are added.
4. Update OpenArm/manipulation docs to distinguish legacy `openarm` and new DMMotor binding paths.
5. Validate mock/vcan, then one-motor hardware, then full-arm adapter gravity compensation.
6. Only consider migrating existing OpenArm blueprints after hardware parity and safety behavior are proven.

## Open Questions

- Should `dm_motor_arm` expose only `DMMotorArm`, or should it also provide an alias such as `dmmotorarm` for CLI convenience?
- Should the adapter build robots from a binding TOML config path, from DimOS `RobotConfig` fields, or support both?
- What model source should gravity compensation use for non-OpenArm DMMotor arms?
- What default damping, if any, is safe for gravity-compensation-only mode while preserving free joint motion?
- Should torque routing be generalized through `ControlCoordinator` in a later change?
