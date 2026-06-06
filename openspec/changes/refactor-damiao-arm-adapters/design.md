## Context

DimOS manipulator hardware is selected through `HardwareComponent.adapter_type`, constructed by the manipulator adapter registry, and consumed by `ControlCoordinator` through the existing `ManipulatorAdapter` protocol. Current OpenArm support includes an `openarm` adapter with an in-tree Damiao/CAN implementation and a newer `dm_motor_arm` binding-backed adapter path.

The current code already has enough configuration hooks for one OpenArm-style DMMotor arm, but the reusable Damiao behavior and OpenArm-specific assumptions are not separated clearly. A fully dynamic URDF-plus-sidecar config loader was considered, but the desired scope is smaller: keep OpenArm explicit, preserve existing OpenArm adapter behavior, and make future Damiao arms straightforward to add by subclassing a shared base.

Relevant surfaces include `dimos/hardware/manipulators/openarm/adapter.py`, `dimos/hardware/manipulators/dm_motor_arm/adapter.py`, `dimos/hardware/manipulators/registry.py`, `dimos/robot/catalog/openarm.py`, `dimos/robot/manipulators/openarm/blueprints.py`, and `docs/capabilities/manipulation/openarm_integration.md`.

## Goals / Non-Goals

**Goals:**

- Keep `OpenArmAdapter` as the explicit OpenArm adapter and keep `adapter_type="openarm"` stable.
- Extract shared Damiao arm behavior into one reusable base used by OpenArm and any future Damiao-arm subclasses.
- Represent per-arm constants with typed class-level specs rather than broad user-editable sidecar config.
- Preserve existing ControlCoordinator, stream, blueprint, and manipulation task contracts.
- Preserve the explicit `dm_motor_arm` adapter path where it remains useful for binding-backed bring-up.
- Keep hardware safety behavior at least as conservative as the current OpenArm and DMMotor adapters.

**Non-Goals:**

- Do not introduce a new runtime-configurable YAML/TOML sidecar schema for arbitrary users in this change.
- Do not remove the existing `openarm` adapter registration.
- Do not silently migrate all OpenArm blueprints to `dm_motor_arm`.
- Do not build or vendor the Rust binding in DimOS; consume the published `can-motor-control` package through the manipulation extra.
- Do not change the public ControlCoordinator stream contract or add new MCP/skill tools.
- Do not implement ros2_control or transmission parsing.

## DimOS Architecture

The implementation should keep the same external coordinator shape:

```text
ControlCoordinator
  -> HardwareComponent(adapter_type="openarm" | "dm_motor_arm")
    -> manipulator adapter registry
      -> OpenArmAdapter or DMMotorArm
        -> shared Damiao base behavior
          -> can_motor_control binding or existing OpenArm CAN bus path, depending on adapter path
```

The shared base should be an internal hardware-layer abstraction, not a new DimOS `Spec` Protocol. The public adapter contract remains `ManipulatorAdapter` from `dimos/hardware/manipulators/spec.py`. No new module streams, transports, or RPC injection contracts are required.

Recommended code shape:

```text
dimos/hardware/manipulators/damiao/
  specs.py          # typed DamiaoJointSpec / DamiaoArmSpec-style data
  base_adapter.py   # shared lifecycle/state/command/gravity behavior

dimos/hardware/manipulators/openarm/adapter.py
  OpenArmAdapter(DamiaoArmAdapterBase)
  OpenArm v10 left/right specs and OpenArm-specific behavior

dimos/hardware/manipulators/dm_motor_arm/adapter.py
  DMMotorArm(DamiaoArmAdapterBase) or thin compatibility wrapper around the base
```

Exact module names can change during implementation, but the boundary should not: common Damiao behavior belongs in one shared hardware-layer implementation, while OpenArm-specific values stay in the OpenArm adapter/catalog layer.

Blueprints should continue to compose `ControlCoordinator.blueprint(...)` with hardware components generated from existing `RobotConfig` catalog helpers. If runnable blueprint names change, regenerate `dimos/robot/all_blueprints.py` with `pytest dimos/robot/test_all_blueprints_generation.py`. If no runnable names change, registry generation is not required.

## Decisions

- **Use shallow inheritance, not a dynamic sidecar loader.** A single shared base plus per-arm subclasses matches the current need and keeps new-arm support code-defined and type-checkable.
- **Keep OpenArm explicit.** Existing users select `adapter_type="openarm"`; preserving that surface avoids a silent hardware behavior change.
- **Use typed arm specs for constants.** Motor names, motor types, send/recv IDs, gains, limits, URDF/gravity-model paths, and side-specific metadata should live in immutable data structures owned by subclasses.
- **Keep the ControlCoordinator contract unchanged.** The refactor should be invisible to tasks and stream consumers: joint state and command routing continue through the same surfaces.
- **Do not make URDF the only source of hardware truth.** URDF remains useful for model/gravity/planning, but Damiao motor IDs, CAN details, gains, and signs are hardware facts supplied by subclass specs.
- **Retain lazy optional imports.** Adapters that need optional bindings must not break registry discovery when those packages are unavailable.
- **Prefer composition inside the base over deep inheritance.** One base class is enough; avoid multi-level adapter hierarchies or mixin webs.

## Safety / Simulation / Replay

Hardware assumptions:

- OpenArm hardware continues to run through the existing `openarm` adapter unless a blueprint explicitly selects a different adapter.
- DMMotor/Damiao hardware may require Linux SocketCAN, CAN-FD or classical CAN depending on adapter path, and staged bring-up before full-arm control.
- Gravity compensation uses a configured model path and current measured joint state; incorrect model/sign/offset assumptions can push hardware unexpectedly.

Safety constraints:

- Preserve disable-on-disconnect and stop behavior.
- Preserve or improve state freshness/coherence checks before command writes and gravity compensation.
- Do not introduce background command loops that keep running after adapter disconnect.
- Keep gravity-compensation-only behavior free-moving by using zero position stiffness where applicable.
- Validate mock/vcan behavior before real hardware, then one-motor or single-arm staged bring-up before full operation.

Simulation/replay:

- Replay is out of scope.
- Mock/vcan adapter tests should cover the refactored shared base.
- Existing OpenArm mock/planner blueprints should remain available.

Manual QA surface:

- Registry discovery includes `openarm` and any retained `dm_motor_arm` key.
- Existing OpenArm mock blueprint still builds and runs through the coordinator surface.
- Binding-unavailable behavior remains explicit when selecting binding-backed adapters.
- Mock/vcan DMMotor adapter can connect, enable, read one coherent state snapshot, write position/effort/MIT commands, and disconnect safely.

## Risks / Trade-offs

- **Accidental behavior drift:** Moving logic under `OpenArmAdapter` can subtly change current OpenArm behavior. Mitigation: focused before/after adapter tests for limits, gains, motor specs, lifecycle, and command shapes.
- **Over-abstracting too early:** A generic base can become too broad. Mitigation: support only the behavior already shared by OpenArm/DMMotor adapters and keep subclass specs explicit.
- **Optional binding confusion:** `openarm` and `dm_motor_arm` may use different low-level paths. Mitigation: document the distinction and keep missing-binding errors tied only to selected binding-backed adapters.
- **Gravity model mismatch:** Shared gravity code can hide per-arm assumptions. Mitigation: subclass specs must provide model path and torque limits explicitly where gravity compensation is enabled.

## Migration / Rollout

1. Add the shared Damiao spec/base layer without changing blueprint names.
2. Move `OpenArmAdapter` onto the shared base while preserving its constructor arguments, registration key, limits, gains, and side behavior.
3. Refactor `DMMotorArm` to reuse the same shared behavior or become a thin compatibility wrapper.
4. Keep existing OpenArm catalog and blueprint selection stable.
5. Add focused tests for shared base behavior, OpenArm compatibility, and retained `dm_motor_arm` behavior.
6. Update OpenArm/manipulation docs to explain that OpenArm is a subclass/preset over shared Damiao behavior.
7. Regenerate blueprint registry only if runnable blueprint names or exported blueprint variables change.

Rollback is straightforward if the refactor is isolated: keep the previous adapter classes intact until parity tests pass, and avoid changing external blueprint names during the first implementation pass.

## Open Questions

- Should the shared base live under `dimos/hardware/manipulators/damiao/` or inside the existing `dm_motor_arm` package?
- Should `DMMotorArm` remain a public generic adapter key, or become an internal compatibility alias after OpenArm uses the shared base?
- Should the initial shared base target only the binding-backed path, or also absorb the in-tree OpenArm CAN driver path?
- How much of OpenArm's current velocity-mode behavior should be retained if the binding-backed path rejects velocity commands?
