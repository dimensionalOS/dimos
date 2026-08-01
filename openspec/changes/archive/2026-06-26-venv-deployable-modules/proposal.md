## Why

DimOS currently requires the coordinator environment to carry many module-specific runtime dependencies because Python workers are launched from the same environment as `dimos run`. This makes it difficult to deploy first-class DimOS Modules that need incompatible, optional, or hardware-specific Python dependency closures while keeping the main coordinator environment small and reliable.

Native modules already have a declarative environment pattern through module-local `flake.nix` files. Python modules that need separate runtime dependencies need an analogous same-machine deployment path that preserves DimOS blueprint orchestration, worker lifecycle, streams, and RPCs.

## What Changes

- Add a Python-first runtime environment registry that names execution environments and resolves concrete local runtime material such as Python interpreters, native executables, environment variables, and optional preparation/build steps.
- Add blueprint-level placement for selected Python Modules into named runtime environments backed by separate Python virtual environments.
- Add a worker launch/process-handle abstraction so the existing Python worker protocol can run over either the current forkserver/Pipe path or a separately launched venv Python interpreter using `multiprocessing.connection.Listener/Client`.
- Define a packaging convention for venv-deployable Python Modules: each dependency-specialized module package owns its own `pyproject.toml` dependency closure, similar in role to a native module's `flake.nix`.
- Preserve existing DimOS Module semantics: modules remain first-class blueprint participants with normal streams, module refs, RPCs, lifecycle, and Pydantic config validation.
- Add a demo module package proving that the coordinator can build and wire a blueprint without installing a dependency that exists only in the venv worker environment.
- Provide a gradual path to unify current `NativeModuleConfig` executable/build fields with named runtime environments without removing the existing fields.

## Capabilities

### New Capabilities
- `runtime-environment-registry`: Defines named runtime environments and how DimOS-managed processes resolve interpreters, executables, environment variables, and preparation steps.
- `venv-module-placement`: Allows blueprints to place import-safe Python Modules into named Python venv worker pools while preserving normal DimOS worker protocol behavior.
- `venv-module-packaging`: Defines the package and import-safety convention for Python Modules whose runtime dependency closure lives outside the coordinator environment.

### Modified Capabilities
- None.

## Impact

- Affected core areas: `dimos/core/coordination`, `dimos/core/module.py`, `dimos/core/native_module.py`, blueprint configuration APIs, and worker lifecycle tests.
- Adds a new local worker launch path using a separate Python executable and `multiprocessing.connection` for the control channel.
- Adds typed runtime environment configuration surfaced through Python APIs first; file loading may be added later as a convenience, not as the primary model.
- Adds packaging/documentation conventions under `packages/` for venv-deployable Python module packages.
- Keeps phase-1 behavior same-machine only; true remote deployment, cluster scheduling, and transport changes remain out of scope.
