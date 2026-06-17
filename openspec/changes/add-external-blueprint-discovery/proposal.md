## Why

DimOS blueprint registration is currently limited to the in-repository, generated `all_blueprints.py` and `all_modules` registries. External robot stacks cannot become runnable through `dimos run` without modifying DimOS source code and regenerating the built-in registry, which makes third-party and downstream package workflows unnecessarily coupled to the DimOS repository.

This change lets installed Python packages expose DimOS-provided runnable blueprint names through packaging metadata, while preserving the existing built-in registry and bare-name behavior. The goal is ROS2-like package-level discoverability for external blueprints without filesystem scanning, eager imports, or extra factory conventions.

## What Changes

- Add dynamic external blueprint discovery from installed Python package entry points in the `dimos.blueprints` entry point group.
- External runnable blueprint names are always namespaced as `<canonical-distribution-namespace>.<external-local-blueprint-name>`.
- Bare blueprint names continue to resolve only through DimOS built-in registries; `dimos run <bare-name>` never searches external packages.
- External entry point targets may be either a `Blueprint` object or a DimOS Module class, which is converted with `.blueprint()`.
- Factory functions are not supported for v1.
- `dimos list` includes external blueprints by default, grouped separately from built-in blueprints, using metadata only and without importing or validating target objects.
- Name resolution errors distinguish unknown external namespaces, missing local blueprint names within a namespace, load failures, and invalid target object types.
- Shared resolver behavior applies consistently to CLI, Python API, and coordinator-side name loading.
- No hardware-safety behavior changes are introduced.

## Affected DimOS Surfaces

- Modules/streams: DimOS Module classes can be exposed through external blueprint entry points and converted to blueprints; no stream semantics change.
- Blueprints/CLI: `dimos run`, `dimos list`, static built-in blueprint/module lookup, blueprint composition command behavior, and resolver error messages.
- Skills/MCP: No direct skill or MCP tool changes.
- Hardware/simulation/replay: No hardware, simulation, or replay semantics change; external packages may provide robot-specific stacks using the same launch path.
- Docs/generated registries: User/developer docs for exposing external blueprints; generated built-in registries remain authoritative for bare names and in-repo blueprints.

## Capabilities

### New Capabilities
- `external-blueprint-discovery`: Discovery, listing, and resolution of externally provided runnable blueprint names from installed package metadata.

### Modified Capabilities
- None.

## Impact

External DimOS users can publish robot stacks as normal Python packages and run them through `dimos run` without editing DimOS source. Existing built-in blueprint names, generated registry tests, and multi-argument blueprint composition remain compatible. The change introduces a runtime dependency on Python entry point metadata APIs and packaging-name normalization, plus new tests for discovery, listing, resolver behavior, error handling, and documentation examples. Remote coordinator loading requires the external package to be installed in the coordinator environment that performs name resolution.
