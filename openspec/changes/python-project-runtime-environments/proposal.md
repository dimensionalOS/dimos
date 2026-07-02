## Why

DimOS Python modules currently run inside the coordinator's Python environment, so modules with large or conflicting dependencies force those dependencies into `dimos run` even when only one module needs them. This makes robot stacks harder to compose, increases dependency conflicts, and prevents clean extraction of modules that require specialized Python, native, or GPU-backed libraries.

DimOS needs a local runtime isolation mechanism that preserves normal Python module semantics: blueprints, typed streams, RPCs, skills, lifecycle, and Spec injection should continue to behave like regular Python modules while selected modules execute from package-local dependency environments.

## What Changes

- Add blueprint-level runtime environment registration for named Python runtime projects.
- Add runtime placements that bind a dependency-light Module Contract to a named runtime environment and Runtime Implementation.
- Add deployment-time Runtime Reconciliation so active placed runtimes are checked and synchronized automatically before workers launch.
- Add Python Runtime Worker pools launched from registered runtime environments while preserving the existing Python worker control protocol.
- Require Runtime Projects to be lockfile-backed and reconciled without mutating source-controlled project files during deployment.
- Require Runtime Implementations to subclass their Module Contract for this change; descriptor-based structural compatibility is deferred.
- Add docs and a lightweight example under `examples/` demonstrating a dependency-light contract and runtime-project implementation.

## Affected DimOS Surfaces

- Modules/streams: Python `Module` deployment, module contracts, runtime implementations, stream/RPC/skill compatibility for placed modules.
- Blueprints/CLI: blueprint composition APIs for runtime environment registration and runtime placement; `dimos run` deployment behavior through `ModuleCoordinator.build()` and dynamic `load_blueprint()` deployment slices.
- Skills/MCP: placed modules keep normal `@skill` and RPC behavior through Python worker semantics; no MCP protocol change.
- Hardware/simulation/replay: no direct hardware, simulation, or replay behavior change; affected blueprints can opt into isolated runtime environments.
- Docs/generated registries: user-facing runtime environment docs and examples; no generated blueprint registry change expected unless an example is intentionally registered.

## Capabilities

### New Capabilities
- `runtime-environment-registration`: Named runtime environments can be registered with blueprints and validated during composition.
- `runtime-placement`: Blueprints can place Module Contracts into named runtime environments and select Runtime Implementations.
- `runtime-reconciliation`: Deployment reconciles active locked Runtime Projects before launching placed module workers.
- `python-runtime-workers`: Python workers can launch from registered runtime environments while preserving DimOS Python module semantics.
- `runtime-project-packaging`: Runtime Projects package isolated dependencies separately from the coordinator environment.
- `blueprint-composition`: Blueprints can carry runtime environment registration and placement behavior through composition.
- `module-deployment`: Python module deployment routes placed modules to runtime-aware worker pools while preserving existing unplaced module behavior.

### Modified Capabilities
None.

## Impact

Developers can isolate complex module dependencies without converting Python modules into native subprocess wrappers or weakening DimOS module semantics. Existing blueprints without runtime placements continue to use the current Python worker path.

The main compatibility risk is that placed modules now distinguish dependency-light Module Contracts from Runtime Implementations. Runtime Projects must include committed lockfile state and must be able to import the dependency-light contract module. Deployment may take longer because it runs package-manager reconciliation for active placed runtimes, but reconciliation should be idempotent and restricted to environment/cache state.

Testing must cover registration errors, duplicate runtime project rejection, locked reconciliation behavior, worker-launch routing, contract/implementation subclass validation, failure-before-worker-launch barriers, dynamic blueprint loading, and the example runtime project. Documentation should explain Runtime Projects, Runtime Reconciliation, Module Contracts, Runtime Implementations, Runtime Placement, and future boundaries for remote runtimes and descriptor-based structural compatibility.
