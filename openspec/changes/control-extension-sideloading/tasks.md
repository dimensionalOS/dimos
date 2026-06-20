## 1. Implementation

- [x] 1.1 Add `dimos/control/extensions.py` with public `register_hardware_adapter(...)` and `register_control_task(...)` functions.
- [x] 1.2 Implement `register_hardware_adapter(...)` dispatch for `HardwareType.MANIPULATOR`, `HardwareType.BASE`, and `HardwareType.WHOLE_BODY` through the existing hardware adapter registries.
- [x] 1.3 Implement `register_control_task(...)` as a lazy factory-path wrapper around the existing control task registry without importing the target factory module at registration time.
- [x] 1.4 Add input validation for extension registration: non-empty hardware adapter type, non-empty control task type, callable hardware factory, and format-valid control task factory path.
- [x] 1.5 Tighten duplicate registration behavior in the manipulator adapter registry so same-name/same-factory registration is idempotent and same-name/different-factory registration raises an error.
- [x] 1.6 Tighten duplicate registration behavior in the twist/base adapter registry so same-name/same-factory registration is idempotent and same-name/different-factory registration raises an error.
- [x] 1.7 Tighten duplicate registration behavior in the whole-body adapter registry so same-name/same-factory registration is idempotent and same-name/different-factory registration raises an error.
- [x] 1.8 Verify or update control task registry duplicate behavior so same-task/same-path registration is idempotent and same-task/different-path registration raises an error.
- [x] 1.9 Preserve existing built-in discovery hooks for hardware adapters and control tasks while routing external packages through the new facade.

## 2. Tests

- [x] 2.1 Add unit tests for `register_hardware_adapter(...)` covering all current `HardwareType` values.
- [x] 2.2 Add unit tests for hardware duplicate policy: same object is idempotent; different object for the same adapter name and hardware type raises.
- [x] 2.3 Add unit tests for `register_control_task(...)` covering lazy path registration and format-only validation.
- [x] 2.4 Add unit tests for control task duplicate policy: same path is idempotent; different path for the same task type raises.
- [x] 2.5 Add a coordinator-level test proving a registered external `BASE` adapter type resolves through `HardwareComponent(hardware_type=BASE, adapter_type="...")` using the normal `ControlCoordinator` path.
- [x] 2.6 Add a coordinator-level test proving a registered external task type resolves from `TaskConfig(type="...")` through the normal lazy control task registry path.
- [x] 2.7 Add regression coverage showing no DimOS package-local adapter or task manifest file is required for the external registration test case.

## 3. Documentation

- [x] 3.1 Add an external robot package guide under the appropriate user-facing docs location, centered on the `register_extensions()` pattern.
- [x] 3.2 In the guide, show all supported embodiments: `MANIPULATOR`, `BASE`, and `WHOLE_BODY`.
- [x] 3.3 Use a `BASE` / robot-dog example as the full walkthrough, including hardware adapter registration, lazy control task registration, and unchanged `HardwareComponent` / `TaskConfig` usage.
- [x] 3.4 Explicitly document phase boundaries: no entry points, no automatic package scanning, no blueprint discovery changes, and no diagnostics helpers in this change.
- [x] 3.5 Update existing custom arm documentation to point external-package authors toward the new external robot package guide where appropriate.

## 4. Verification

- [x] 4.1 Run `openspec validate control-extension-sideloading`.
- [x] 4.2 Run focused tests for control extensions, hardware registries, control task registry, and ControlCoordinator resolution.
- [x] 4.3 Run docs validation for changed docs, including `doclinks` and `md-babel-py run <changed-doc-path>` when applicable.
- [x] 4.4 Confirm no blueprint or module registry generation is required; if implementation adds or renames blueprint registry inputs, run `pytest dimos/robot/test_all_blueprints_generation.py`.
- [x] 4.5 Manually QA by importing an external-package-style module that calls `register_extensions()`, then constructing a coordinator configuration with the registered adapter and task names without physical robot motion.

## 5. Runnable example

- [x] 5.1 Add a no-hardware sample external package with a custom `BASE` adapter and custom control task.
- [x] 5.2 Add visible logging for extension registration, adapter construction/connection, task creation/ticks, and adapter command writes.
- [x] 5.3 Add pytest coverage that imports the sample as an external package and captures the runtime log proof.
- [x] 5.4 Document the demo command from the external robot package guide.
