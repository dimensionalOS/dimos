## Why

External robot packages need a supported way to provide the ControlCoordinator pieces their blueprints reference without placing files inside the DimOS repository. Today, hardware adapters are discovered by closed-world in-repo registries and control tasks are discovered from in-repo manifests. That makes a complete outside-the-repo robot package impractical even when blueprint discovery is handled elsewhere.

This change introduces a small public extension surface for external packages to sideload hardware adapters and control tasks before a blueprint builds. The goal is to let an external robot package register its adapter and task names, then keep using the existing `HardwareComponent` and `TaskConfig` configuration path unchanged.

## What Changes

- Add a public `dimos.control.extensions` facade for external packages.
- Add `register_hardware_adapter(hardware_type, adapter_type, factory)` for all current `HardwareType` values: manipulator, base, and whole-body.
- Add `register_control_task(task_type, factory_path)` using the existing lazy factory-path model for task factories.
- Tighten duplicate registration behavior in the underlying hardware and control-task registries: same key with the same object/path is idempotent; same key with a different object/path raises an error.
- Document the blessed external package pattern: define `register_extensions()`, call it from the external blueprint module at import time, and continue using `HardwareComponent` and `TaskConfig` normally.
- No entry point discovery, automatic package scanning, diagnostics helpers, or blueprint discovery changes are included in this phase.

## Affected DimOS Surfaces

- Modules/streams: no stream protocol changes; external modules may use the new registration facade before blueprint build.
- Blueprints/CLI: blueprints can reference externally registered `HardwareComponent.adapter_type` and `TaskConfig.type` values; CLI discovery behavior is unchanged.
- Skills/MCP: no direct changes; external skills remain exposed through launched modules as today.
- Hardware/simulation/replay: hardware adapter registration changes affect manipulator, base, and whole-body adapter registries, including simulation adapters that use those registries.
- Docs/generated registries: add external robot package documentation; no generated blueprint registry behavior changes.

## Capabilities

### New Capabilities

- `control-extension-sideloading`: Public behavior for registering external ControlCoordinator hardware adapters and control tasks.

### Modified Capabilities

- None.

## Impact

External robot authors gain a stable public API for registering ControlCoordinator extension points from installed packages. Existing DimOS blueprints and built-in registries keep their current usage pattern, but duplicate name collisions now fail instead of silently overwriting. This may expose accidental internal duplicate registrations during testing, which should be fixed explicitly.

Documentation should present this as one layer of the external robot package story: this change handles ControlCoordinator sideloading, while external blueprint discovery and automatic package discovery remain separate or future work. Tests should cover real ControlCoordinator resolution through externally registered hardware adapter and task names, plus duplicate and validation behavior.
