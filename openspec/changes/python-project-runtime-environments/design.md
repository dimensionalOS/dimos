## Context

DimOS deploys Python modules through `ModuleCoordinator`, `PythonWorkerManager`, and `PythonWorker`. Blueprints currently identify modules by module class, inspect stream and module-reference annotations in the coordinator process, and deploy module classes into Python worker processes using the existing worker request/response protocol. This preserves normal DimOS behavior for typed streams, RPC methods, skills, lifecycle, module refs, and `get_instance()` lookup.

The source branch `origin/cc/frontier` contains a working venv-worker path, but it is mixed with broader experimentation and dynamic IO-contract work that should not be blindly transplanted. This change extracts the runtime-worker idea into the current branch while preserving the current annotation-based blueprint model and avoiding unrelated native-module, transport, and dynamic-IO changes.

The resolved domain terms for this change are captured in `CONTEXT.md`: Runtime Reconciliation, Runtime Project, Locked Runtime Project, Runtime Environment Registration, Python Runtime Worker, Deployment Slice, Runtime Placement, Module Contract, Runtime Implementation, and future Contract Descriptor.

## Goals / Non-Goals

**Goals:**
- Allow selected Python modules to run from package-local runtime environments without installing their heavy dependencies into the coordinator environment.
- Preserve normal DimOS Python module semantics for placed modules: streams, RPCs, skills, lifecycle, module refs, and `get_instance()` behavior.
- Use dependency-light Module Contracts in coordinator-visible code and Runtime Implementations inside Runtime Projects.
- Reconcile active Runtime Projects automatically during deployment before any placed module workers launch.
- Keep deployment non-mutating for source-controlled project metadata by using locked package-manager modes.
- Support initial coordinator builds and later blueprint loads through the same runtime deployment boundary.

**Non-Goals:**
- Remote, SSH, container, Kubernetes, or multi-machine runtime deployment.
- Descriptor-based structural compatibility that lets implementations avoid importing/subclassing Module Contracts.
- Native module runtime unification, native RPC, or expanded native transport support.
- Dynamic module IO contracts or per-instance blueprint identity changes.
- A separate mandatory `dimos runtime prepare` command. A future explicit build/update command may own lockfile mutation.
- Supporting two active runtime placements for two instances of the same module class.

## DimOS Architecture

Blueprints gain two runtime-related surfaces:
- Runtime Environment Registration: named runtime environments attached to a blueprint.
- Runtime Placement: class-keyed placement that binds a Module Contract to a runtime environment and Runtime Implementation import path.

The coordinator remains responsible for blueprint graph construction from dependency-light Module Contract classes. Contracts are regular `Module` subclasses that declare the DimOS-facing surface: `In[T]`/`Out[T]` streams, config, RPC methods, skills, and module refs. Runtime Implementations live in Runtime Projects and subclass their Module Contract. The runtime worker imports the implementation path, validates `issubclass(Implementation, Contract)`, then instantiates the implementation instead of the contract.

Runtime Reconciliation runs in `ModuleCoordinator.build()` and `ModuleCoordinator.load_blueprint()` before any worker in the current Deployment Slice launches. It selects active placed Runtime Projects only, rejects unknown placements, rejects duplicate project paths during registration/merge, and reconciles independent runtime projects in parallel behind a deployment barrier. If any reconciliation fails, the whole deployment slice aborts before worker launch and reports grouped errors.

Python Runtime Worker pools reuse the existing Python worker control protocol. Unplaced Python modules continue to use the default Python worker manager. Placed modules use manager keys such as `python:<runtime-name>` and launch through a worker launcher backed by the runtime environment. Direct venv runtimes launch `<venv-python> -m dimos.core.coordination.venv_worker_entrypoint`; project runtimes launch through the runtime project's toolchain command while preserving the same worker entrypoint and request/response protocol.

Runtime Projects are package-local Python projects with committed lockfile state. A uv-backed project uses `.venv/` plus `pyproject.toml` and `uv.lock`. A Pixi-backed project may use `.pixi/`, `pixi.toml`, and `pixi.lock`, then run uv inside Pixi. Reconciliation may mutate environment/cache directories such as `.venv/`, `.pixi/`, and package-manager caches, but it must not rewrite source-controlled project files during deployment.

DimOS `Spec` Protocols remain separate from Module Contracts. A DimOS Spec is still a structural interface used for module-reference injection. A Module Contract is a deployable, dependency-light `Module` class used as coordinator-visible identity for a placed module.

## Decisions

1. **Use Python Runtime Workers, not NativeModule special-casing.**
   Isolated Python dependencies should preserve normal Python module behavior. NativeModule is for external executables that perform their own data-plane pub/sub and currently lacks true native RPC and broad transport support.

2. **Run Runtime Reconciliation during deployment.**
   Runtime environments are checked/updated automatically during deployment rather than requiring a separate prepare step. Reconciliation runs before worker launch and applies to both initial builds and later blueprint loads.

3. **Use locked, non-mutating reconciliation.**
   Deployment may create/update environment state but must not mutate lockfiles or project metadata. Stale or missing lockfiles fail clearly and point users to manual package-manager commands or a future explicit build/update command.

4. **Use class-keyed Runtime Placements for this change.**
   The current blueprint model deduplicates by module class. Placement by module class matches current identity behavior and avoids introducing per-atom or per-instance addressing.

5. **Reject duplicate Runtime Project paths at registration/merge time.**
   Two runtime names pointing to the same canonical project path are invalid globally, even if only one is active. This avoids ambiguous ownership and reconciliation behavior.

6. **Use contract/implementation split with nominal subclass validation now.**
   Module Contracts stay dependency-light and coordinator-visible. Runtime Implementations subclass the contract inside the runtime project. Descriptor-based structural compatibility is deferred as a future remote/deeper isolation feature.

7. **Bind implementation path in RuntimePlacement.**
   Runtime placement carries both the runtime name and implementation import path, e.g. `RuntimePlacement(runtime="detector-runtime", implementation="detector_runtime.detector.Detector")`. This keeps contracts pure and allows different blueprints/tests to map the same contract to different implementations.

## Safety / Simulation / Replay

This change does not directly command hardware and does not alter robot motion semantics. Its safety boundary is deployment-time correctness: placed modules must either reconcile and deploy successfully or fail before any worker in the deployment slice launches.

Simulation and replay blueprints should behave the same as hardware blueprints with respect to runtime placement. Runtime Reconciliation must respect active blueprint selection and disabled modules so inactive runtime projects are not reconciled.

Manual QA should include a small runtime-project example that runs without worker-only dependencies in the coordinator environment, plus a failure case where reconciliation fails before module workers launch.

## Risks / Trade-offs

- **Runtime project lockfiles can be stale.** Mitigation: use locked package-manager modes and fail clearly without rewriting source-controlled files.
- **Contract imports can accidentally pull heavy dependencies.** Mitigation: document that Module Contracts must be dependency-light and test the example by importing/building the blueprint without runtime-only dependencies in the coordinator environment.
- **Runtime implementation must import the contract package.** This is accepted for this extraction. Future Contract Descriptor support can remove that requirement.
- **Deployment gets slower.** Mitigation: reconcile only active placed Runtime Projects, run independent projects in parallel, and rely on package-manager idempotency for fast no-op checks.
- **Source branch sediment.** Mitigation: manually transplant only scoped runtime-worker logic and avoid dynamic IO-contract, native-module, manipulation/planning, and heavy demo changes.

## Migration / Rollout

Existing blueprints without runtime placements continue to deploy through the default Python worker pool. New runtime-aware blueprints opt in by registering runtime environments and adding placements.

Implementation should add focused docs under `docs/usage/runtime_environments.md`, link them from `docs/usage/README.md`, and add a lightweight example under `examples/dimos-demo-worker-module/`. No robot blueprint registry regeneration is expected unless an example blueprint is registered in the main blueprint registry.

Rollback is straightforward for opt-in users: remove runtime placements and runtime environment registrations from the blueprint, or revert to an implementation that can run in the coordinator/default worker environment.

## Open Questions

None for this extraction. Future work may define Contract Descriptors for structural compatibility without runtime contract imports, remote/SSH runtime environments, and an explicit build/update command that is allowed to mutate lockfiles.
