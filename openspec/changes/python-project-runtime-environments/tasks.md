## 1. Runtime model and reconciliation

- [x] 1.1 Add runtime environment model types for current Python, direct venv Python, and package-local Python Runtime Projects.
- [x] 1.2 Add runtime launch material types for direct Python executables and toolchain-backed Runtime Projects.
- [x] 1.3 Implement Runtime Environment Registration validation, including unknown runtime lookup errors and duplicate Runtime Project path rejection during registration/merge.
- [x] 1.4 Implement Runtime Reconciliation plans for active placed Runtime Projects only.
- [x] 1.5 Implement locked, non-mutating uv reconciliation that may update `.venv/` and caches but must not rewrite source-controlled project files.
- [x] 1.6 Implement locked Pixi-backed uv reconciliation support, including locked Pixi environment reconciliation before uv synchronization.
- [x] 1.7 Implement deployment-barrier behavior with parallel reconciliation of independent Runtime Projects and grouped failure reporting.

## 2. Worker launch and deployment protocol

- [x] 2.1 Add worker launcher and worker process handle abstractions for default forkserver workers and runtime-launched workers.
- [x] 2.2 Add `dimos.core.coordination.venv_worker_entrypoint` so external Python runtimes can connect back to the existing Python worker loop.
- [x] 2.3 Extend Python worker deployment requests so a placed Module Contract can deploy a Runtime Implementation import path.
- [x] 2.4 Validate in the runtime worker that the imported Runtime Implementation subclasses the Module Contract before instantiation.
- [x] 2.5 Ensure runtime implementation instances receive normal config kwargs, actor refs, lifecycle calls, RPC calls, and stream wiring.
- [x] 2.6 Ensure runtime worker startup, connection, deployment, and validation failures report runtime name, contract, and implementation path when applicable.

## 3. Blueprint and coordinator integration

- [x] 3.1 Add blueprint APIs for runtime environment registration and class-keyed Runtime Placement with runtime name plus implementation path.
- [x] 3.2 Preserve runtime registrations and placements through `autoconnect()` while keeping existing blueprint behavior for blueprints without runtime metadata.
- [x] 3.3 Route unplaced Python modules through the default Python worker pool and placed modules through runtime-specific Python worker pools.
- [x] 3.4 Run Runtime Reconciliation before worker launch in `ModuleCoordinator.build()` for initial deployment slices.
- [x] 3.5 Run Runtime Reconciliation before worker launch in `ModuleCoordinator.load_blueprint()` for dynamic deployment slices.
- [x] 3.6 Ensure disabled placed modules do not create active runtime reconciliation demand.
- [x] 3.7 Ensure failed runtime-aware deployment slices clean up newly created runtime managers or placement state without disturbing existing workers.

## 4. Example runtime project

- [x] 4.1 Add `examples/dimos-demo-worker-module/` with a dependency-light Module Contract and a Runtime Implementation in a package-local Runtime Project.
- [x] 4.2 Add committed lockfile state for the example Runtime Project.
- [x] 4.3 Ensure the example proves coordinator import/build does not require runtime-only dependencies.
- [x] 4.4 Ensure the example proves runtime worker execution can call implementation behavior through normal DimOS RPC/proxy semantics.

## 5. Documentation

- [x] 5.1 Add `docs/usage/runtime_environments.md` covering Runtime Projects, Locked Runtime Projects, Runtime Reconciliation, Runtime Placement, Module Contracts, Runtime Implementations, and Python Runtime Workers.
- [x] 5.2 Document uv-backed and Pixi-backed runtime project conventions, locked reconciliation behavior, and missing/stale lockfile failure guidance.
- [x] 5.3 Document the distinction between DimOS `Spec` Protocols and Module Contracts.
- [x] 5.4 Document future boundaries for Contract Descriptors, remote/SSH runtimes, and explicit build/update commands.
- [x] 5.5 Update `docs/usage/README.md` to link the runtime environments guide.

## 6. Tests

- [x] 6.1 Add unit tests for runtime environment registration, lookup, merge behavior, and duplicate Runtime Project path rejection.
- [x] 6.2 Add unit tests for Runtime Reconciliation selection of active placed runtimes, disabled module handling, locked command selection, parallel execution, and grouped failures.
- [x] 6.3 Add worker tests for runtime launcher startup, entrypoint connection, implementation import, subclass validation, and error reporting.
- [x] 6.4 Add coordinator tests for runtime-aware routing, deployment barriers, build/load deployment slices, cleanup on failure, and default behavior for unplaced modules.
- [x] 6.5 Add blueprint tests for runtime registration/placement APIs and composition behavior.
- [x] 6.6 Add example tests proving coordinator-import safety and runtime execution through the example Runtime Project.
- [x] 6.7 Add CLI-focused tests only if implementation changes CLI-visible errors or options.

## 7. Verification

- [x] 7.1 Run `openspec validate python-project-runtime-environments --type change --strict --no-interactive`.
- [x] 7.2 Run focused pytest targets for runtime environment, runtime reconciliation, worker launcher, Python worker, module coordinator, blueprint composition, and example runtime project tests.
- [x] 7.3 Run `uv run pytest dimos/core/coordination/test_worker.py dimos/core/coordination/test_module_coordinator.py dimos/core/coordination/test_blueprints.py` or the updated focused equivalents.
- [x] 7.4 Run documentation validation for `docs/usage/runtime_environments.md` and changed usage docs, including link checks if available.
- [x] 7.5 Manually QA through the user-facing deployment surface by running or test-building a blueprint that places the example Module Contract into its Runtime Project.
- [x] 7.6 Audit the final diff against `origin/main` for excluded source-branch sediment: native module runtime unification, dynamic IO contracts, manipulation/planning demos, GPD/VGN demos, TSDF/reconstruction, sidecars, and unrelated lockfile churn.
- [x] 7.7 Run `pytest dimos/robot/test_all_blueprints_generation.py` only if the implementation intentionally changes registered robot blueprint inputs or generated registry output.
