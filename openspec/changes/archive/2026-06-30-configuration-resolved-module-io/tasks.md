## 1. Core IO Contract API

- [x] 1.1 Add public `StreamDecl` and `ModuleIOContract` types with stream-name uniqueness validation.
- [x] 1.2 Add shared validation helpers so construction-time and resolution-time checks use the same rules.
- [x] 1.3 Add default `Module.io_contract(config)` that returns annotation-derived IO.
- [x] 1.4 Add tests proving annotation-only modules resolve the same streams as before.

## 2. Module Stream Registries

- [x] 2.1 Add explicit internal input/output registries to `Module`.
- [x] 2.2 Register annotated streams in the registries while preserving existing annotated attributes.
- [x] 2.3 Instantiate custom configuration-resolved streams into registries without creating arbitrary attributes.
- [x] 2.4 Update `inputs`, `outputs`, `set_transport()`, `connect_stream()`, and related stream lookup paths to use registries.
- [x] 2.5 Add tests proving configuration-resolved streams are available through `self.inputs` and `self.outputs`.

## 3. Resolved Blueprint Planning

- [x] 3.1 Add an internal resolved module plan that stores final module config, final kwargs, and resolved IO contract.
- [x] 3.2 Resolve final per-module config before stream conflict checks, deployment, and wiring in `build()`.
- [x] 3.3 Resolve final per-module config before stream conflict checks, deployment, and wiring in blueprint load paths.
- [x] 3.4 Pass final per-module kwargs into worker deployment instead of late-merging `blueprint_args` inside the worker manager.
- [x] 3.5 Add tests proving CLI/build overrides affect resolved IO before wiring.

## 4. Remapping and Conflict Validation

- [x] 4.1 Validate remappings against resolved stream names.
- [x] 4.2 Fail blueprint resolution when a remapping references a stream absent from the resolved IO contract.
- [x] 4.3 Preserve existing same-name/same-type autoconnect behavior after remapping.
- [x] 4.4 Preserve existing same-name/different-type conflict behavior after remapping.
- [x] 4.5 Add tests for valid remapping, stale remapping, autoconnect, and type-conflict scenarios.

## 5. Teaching Demo

- [x] 5.1 Add `examples/configuration_resolved_io.py` demonstrating a module whose config selects between two different IO shapes.
- [x] 5.2 Document how to run the demo in the example file or nearby README text.
- [x] 5.3 Add a verification command for the demo and ensure it passes locally.

## 6. Verification Gates

- [x] 6.1 Run the focused configuration-resolved IO unit tests.
- [x] 6.2 Run the existing affected core blueprint/module tests.
- [x] 6.3 Run the teaching demo successfully.
- [x] 6.4 Run the standard project test command if practical for the implementation scope.

Standard test note: `uv run pytest -q` was attempted and timed out after 300s; early failures were unrelated existing codebase checks for `__all__` and `__init__.py` files under hardware/simulation paths.
