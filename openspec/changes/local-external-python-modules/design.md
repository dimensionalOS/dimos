## Context

DimOS normally instantiates Python `Module` classes in `PythonWorker` processes selected by `WorkerManagerPython`. External declarations are selected by the dedicated `WorkerManagerExternalPython`. `BlueprintAtom` derives streams and module references from annotations, and `ModuleCoordinator` connects transports and RPC proxies before `build()` and `start()`. `NativeModule` is the existing precedent for an external runtime that remains a normal Blueprint participant.

PR #2869 demonstrates the desired local package layout but adds a generic deployment planner, target session, external worker manager, and public deployment API even though only a local Python target is supported. The example keeps its contract in `examples/external_python_module/contract.py` and puts the isolated runtime in its sibling `python/` directory.

## Goals / Non-Goals

**Goals:**
- Run a separately packaged local Python implementation with its own dependency environment.
- Preserve ordinary Blueprint composition, typed streams, RPC, skills, configuration, module references, and restart behavior.
- Preserve the PR example's contract-plus-sibling-`python/` project structure.
- Provide one concise public declaration field: an implementation import reference.

**Non-Goals:**
- Remote execution, SSH, artifact transfer, tunnels, target sessions, or cross-machine deployment.
- A generic deployment planner, public deployment CLI, package-layout autodiscovery beyond the sibling `python/` convention, or alternate runtime-selection API.
- Supporting non-Python runtimes; `NativeModule` remains the established path for executables.

## DimOS Architecture

An external Python declaration remains a normal `Module`-compatible Blueprint atom. It declares its configuration, `In[T]`/`Out[T]` streams, RPC methods, skills, and DimOS `Spec` Protocol references exactly as an in-repository module does. The declaration exposes only an implementation import reference.

The runtime project is resolved from the declaration source directory:

```text
<declaration-dir>/
├── deployment.py
└── python/
    ├── pyproject.toml     # required
    ├── pixi.toml          # optional
    └── <runtime package>/
```

`WorkerManagerExternalPython` selects a private local external-Python worker for these declarations. It prepares the sibling runtime project, launches its Python bootstrap, and returns the same coordinator-facing RPC proxy shape as a normal worker. `ModuleCoordinator` remains responsible for the existing order: deploy, connect streams, inject module references, build, and start.

The bootstrap imports the implementation from the declared reference, verifies that it fulfills the declaration contract, and serves RPC under the declaration identity so module references and RPC clients use the stable contract name. The implementation environment must declare all dependencies needed by the runtime, including its compatible DimOS dependency. The contract is a normal import from the existing DimOS distribution; `PYTHONPATH` injection is not used.

No generated blueprint registry, CLI entry point, transport type, or DimOS `Spec` Protocol is introduced or changed.

## Decisions

1. **`ExternalPythonModule` has one runtime-specific public field.** A declaration identifies its implementation with an import reference. Interpreter paths, target/session models, deployment plans, and package-discovery APIs are excluded. This preserves a small authoring API and matches ordinary Blueprint composition.

2. **Use the PR's sibling `python/` runtime-project boundary.** `python/pyproject.toml` is mandatory; `python/pixi.toml` is optional. The fixed convention is preferred over walking parent directories or guessing a project root, which would make host-project selection ambiguous.

3. **Manage the environment through the project manifests.** Without Pixi, preparation and execution use uv for the required Python project. With Pixi, Pixi runs uv so the Pixi tool environment and uv project environment are layered. The lifecycle has an explicit prepare phase before launch; lockfile and offline behavior must be made deterministic in the worker implementation and tests.

4. **Hide process specialization behind the dedicated external worker manager.** The local external worker is an internal `WorkerManagerExternalPython` concern, analogous to the external-process ownership in `NativeModule`. A separate deployment plan and coordinator plan injection would duplicate lifecycle ownership and leak implementation details into `ModuleCoordinator`.

5. **Treat the declaration as the stable RPC contract.** The runtime implementation must fulfill the declaration, and the bootstrap must serve the declaration identity. This avoids proxy calls targeting the implementation name while the coordinator and consumers use the declaration name.

## Safety / Simulation / Replay

The change does not command hardware or alter robot safety policies. External modules may be used in hardware, simulation, or replay Blueprints, so failure to prepare or start a runtime must fail deployment before module startup and include actionable diagnostics. Existing transport selection and replay/simulation configuration continue to flow through the coordinator; manual validation must cover a Blueprint that mixes normal and external modules.

## Risks / Trade-offs

- **Environment preparation can mutate local state or require network access.** Keep preparation explicit, respect lockfiles, expose command output in failures, and test the prepared/offline path.
- **Pixi and uv layer rather than merge Python dependencies.** Require runtime Python imports in `pyproject.toml`; document that Pixi provides tools and environment context.
- **A sidecar can die after startup.** Detect termination, preserve bounded stdout/stderr diagnostics, and terminate the process group during stop and failed startup cleanup.
- **The fixed layout is less flexible than arbitrary paths.** It is intentional: the convention eliminates a runtime-selection API. New layouts can be considered only after concrete requirements emerge.

## Migration / Rollout

This is additive. Add the concise declaration/runtime API and migrate the PR example to remove its deployment spec and planner-facing setup while retaining its `deployment.py`, `contract.py`, and sibling `python/` tree. Add authoring documentation to `docs/usage/modules.md` and tests for preparation, composition, restart, and cleanup. No registry generation is required.

## Open Questions

- The example intentionally does not check in a lockfile. When a runtime project has a local `uv.lock`, preparation and launch preserve the existing locked behavior; otherwise uv resolves from `pyproject.toml` and may create a local lockfile.
