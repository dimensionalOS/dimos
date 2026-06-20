## Context

DimOS blueprints describe hardware through `HardwareComponent` entries and control behavior through `TaskConfig` entries. `ControlCoordinator` resolves those string names through internal registries: manipulators use the manipulator adapter registry, mobile bases use the twist/base adapter registry, whole-body robots use the whole-body adapter registry, and control tasks use the task registry.

Those registries currently discover built-in adapters and tasks from DimOS package-local files. That works for in-repo robots, but it blocks external robot packages from being complete on their own: an external blueprint can refer to `adapter_type="mydog"` or `TaskConfig(type="mydog_gait")`, but DimOS has no public supported API for that package to register those names before `ControlCoordinator` resolves them.

This design adds a small public facade for explicit registration. External blueprint discovery and package auto-discovery are intentionally out of scope; this change assumes an external package's blueprint module is imported by the user or by a separate discovery mechanism.

## Goals / Non-Goals

**Goals:**

- Provide a public `dimos.control.extensions` module for external ControlCoordinator registration.
- Support external hardware adapter registration for all current `HardwareType` values: `MANIPULATOR`, `BASE`, and `WHOLE_BODY`.
- Support external control task registration using the existing lazy factory-path style.
- Preserve unchanged blueprint usage through `HardwareComponent.adapter_type` and `TaskConfig.type`.
- Make duplicate registrations safe and debuggable by failing on conflicting mappings while allowing idempotent same-object or same-path re-registration.
- Document an external robot package pattern centered on `register_extensions()` in the package's blueprint module.

**Non-Goals:**

- Add Python entry point discovery.
- Add automatic package scanning.
- Change CLI or generated blueprint registry discovery.
- Add `available_*` diagnostics helpers.
- Replace the three hardware registries with a new unified internal registry.
- Add a robot template generator.
- Change stream, transport, skill, or MCP contracts.

## DimOS Architecture

The public API should live in a new facade module:

```python
from dimos.control.extensions import register_hardware_adapter, register_control_task
```

`register_hardware_adapter(hardware_type, adapter_type, factory)` is the logical public hardware catalog. Internally it dispatches to the existing registries by `HardwareType`:

- `MANIPULATOR` delegates to `dimos.hardware.manipulators.registry.adapter_registry`.
- `BASE` delegates to `dimos.hardware.drive_trains.registry.twist_base_adapter_registry`.
- `WHOLE_BODY` delegates to `dimos.hardware.whole_body.registry.whole_body_adapter_registry`.

The hardware factory is any callable accepted by the target registry. The public contract does not guarantee semantic equivalence for wrappers such as fresh `partial()` objects or lambdas; idempotency is object identity only.

`register_control_task(task_type, factory_path)` delegates to the existing `control_task_registry.register_path(...)` lazy path mechanism. The extension facade should validate the name and path shape but should not import the task module during registration. The factory is imported later when `ControlCoordinator` creates the configured task.

Blueprints remain unchanged:

```python
HardwareComponent(hardware_type=HardwareType.BASE, adapter_type="mydog")
TaskConfig(type="mydog_gait")
```

External packages should group registration in a helper and call it from their blueprint module at import time:

```python
def register_extensions() -> None:
    register_hardware_adapter(HardwareType.BASE, "mydog", MyDogAdapter)
    register_control_task("mydog_gait", "dimos_mydog.tasks.gait:create_task")

register_extensions()
```

No module stream names or transport schemas change. No DimOS Python `Spec` Protocol is required for this API. Adapter Protocol expectations remain those already consumed by `ConnectedHardware`, `ConnectedTwistBase`, and `ConnectedWholeBody`. Skills and MCP exposure remain dynamic through launched modules and are not part of this registration surface.

## Decisions

1. **Use a public facade instead of documenting internal registries.**
   External users should import `dimos.control.extensions`, not low-level hardware or task registry singletons. This keeps the stable public API small while allowing internal registry implementation to evolve.

2. **Use one hardware registration function keyed by `HardwareType`.**
   The user-facing model is a logical hardware adapter catalog keyed by `(hardware_type, adapter_type)`. The implementation can still delegate to separate internal registries to avoid a larger refactor.

3. **Keep hardware factories direct callables.**
   Hardware registries already create adapter instances from callable objects. Direct callables keep external adapter registration simple and allow classes or factory functions.

4. **Use lazy factory paths for control tasks.**
   Control tasks may depend on heavier planning, simulation, policy, or IK code. A lazy path matches the existing built-in task registry and avoids importing task modules until the task is actually selected.

5. **Validate task paths only by shape at registration time.**
   The registration API should reject empty or malformed paths, but it should preserve laziness by not importing the target module.

6. **Enforce duplicate protection in underlying registries.**
   Duplicate behavior should be consistent whether registration happens through the facade or through existing internal discovery. Same key with the same object/path is idempotent; same key with a different object/path raises an error.

7. **Do not support semantic equivalence for wrappers.**
   If a caller creates a new `partial()`, lambda, or wrapper each time, it is a different factory object. The public pattern should use stable top-level classes or functions.

## Safety / Simulation / Replay

This change does not alter command semantics, robot motion limits, stream contents, or transport behavior. It changes which registered adapter and task names `ControlCoordinator` can resolve.

The safety-sensitive risk is accidental name collision. Silent overwrite could cause a blueprint to resolve to the wrong hardware adapter or task factory. The new duplicate policy mitigates this by failing on conflicting mappings.

Simulation adapters are covered when they register through one of the existing hardware registries. Replay behavior is unchanged because no recorded stream format changes.

Manual QA should use fake adapters and tasks rather than physical robot motion. Hardware-facing manual tests, if any, should be limited to confirming registration and coordinator construction before any movement command is issued.

## Risks / Trade-offs

- **Underlying duplicate enforcement may expose existing internal collisions.** Run focused tests and fix any collision explicitly instead of preserving silent overwrite.
- **Import order remains explicit.** Without entry points, registration only happens when the external package module that calls `register_extensions()` is imported. Documentation must make this pattern clear.
- **The facade is intentionally narrow.** External packages still need a way to make their blueprints importable or discoverable. That is handled elsewhere or later.
- **Callable identity is strict.** This avoids fuzzy equality rules, but callers must use stable factory objects for idempotent registration.

## Migration / Rollout

Existing built-in DimOS registrations should continue to use their current registry hooks unless maintainers choose to migrate them later. External docs should bless only `dimos.control.extensions`.

No generated blueprint registry update is required because this change does not add or rename DimOS blueprints. No new dependency is required.

Rollout steps:

1. Add the public facade module.
2. Tighten duplicate registration behavior in hardware and task registries.
3. Add tests for direct facade behavior, duplicate policy, invalid input, and real `ControlCoordinator` resolution.
4. Add the external robot package guide.
5. Run focused control/registry tests and documentation validation.

Rollback is straightforward: remove the facade and restore prior duplicate behavior if needed, with no data migration.

## Open Questions

None for phase 1. Future work may add entry point discovery, diagnostics helpers, automatic package scanning, or a robot package template.
