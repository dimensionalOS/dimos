## Why

DimOS cannot currently run a separately packaged Python module with dependencies isolated from the host environment without introducing deployment abstractions intended for future remote execution. The proposed approach must support the local package layout in PR #2869 while keeping ordinary Blueprint composition and the module coordinator as the public model.

## What Changes

- Add a local external-Python-module capability whose declaration names only its implementation import reference.
- Establish a deterministic runtime-project convention: a declaration's sibling `python/` directory contains a required `pyproject.toml` and an optional `pixi.toml`.
- Prepare and launch the runtime through uv, layered inside Pixi when the optional Pixi project is present.
- Keep external modules compatible with ordinary Blueprints, typed streams, RPC, skills, module references, configuration, and restart behavior.
- Exclude remote targets, deployment plans, target sessions, package transfer, and deployment-specific public APIs.

## Affected DimOS Surfaces

- Modules/streams: Module declarations, local external runtime process lifecycle, typed stream wiring, RPC identity, and module-reference injection.
- Blueprints/CLI: Blueprint composition and coordinator lifecycle; no new user-facing CLI is planned.
- Skills/MCP: Existing skill and RPC exposure must remain available through the external declaration contract.
- Hardware/simulation/replay: No direct hardware, simulation, or replay behavior changes.
- Docs/generated registries: User-facing module authoring documentation and the external Python module example.

## Capabilities

### New Capabilities
- `external-python-modules`: Locally run separately packaged Python module implementations while retaining normal DimOS module composition.

### Modified Capabilities

None.

## Impact

Module authors gain a dependency-isolated local runtime convention without learning a deployment API. The change introduces a new module-declaration base type and requires external runtime projects to provide the documented `python/` layout and package-manager manifests. Testing must cover environment preparation, startup failures, streams, RPC/skills, module references, and restart. Documentation must clearly describe the two-layer Pixi/uv environment behavior and its dependency boundaries.
