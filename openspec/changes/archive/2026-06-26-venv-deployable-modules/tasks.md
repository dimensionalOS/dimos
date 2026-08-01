## 1. Worker Launch Abstraction

- [x] 1.1 Extract the current forkserver process/Pipe operations into a worker process handle abstraction without changing default worker behavior.
- [x] 1.2 Add a worker launcher abstraction with a forkserver launcher implementation that preserves current `PythonWorker` deployment tests.
- [x] 1.3 Update worker manager code to use launcher/process-handle interfaces while keeping existing scheduling, capacity, dedicated-worker, and deploy_parallel behavior.
- [x] 1.4 Add unit tests proving the default forkserver worker path still deploys, starts, calls RPCs, and shuts down Modules as before.

## 2. Venv Worker Control Channel

- [x] 2.1 Add a worker entrypoint that can be launched with an arbitrary Python executable and connect back to the coordinator using `multiprocessing.connection.Client`.
- [x] 2.2 Add a coordinator-side `multiprocessing.connection.Listener` setup for venv worker launch and connection acceptance.
- [x] 2.3 Implement a venv worker process handle that sends and receives existing worker request/response objects over the multiprocessing connection channel.
- [x] 2.4 Ensure worker stdout/stderr are handled separately from the control channel so logs cannot corrupt worker messages.
- [x] 2.5 Add failure tests for missing Python executable, worker connection timeout, incompatible worker import, and worker startup error propagation.

## 3. Runtime Environment Registry

- [x] 3.1 Define typed runtime environment models for current process, Python venv, and Nix-backed native executable resolution.
- [x] 3.2 Add a Python-first runtime environment registry that resolves named environments and reports clear errors for unknown names or unsupported capabilities.
- [x] 3.3 Wire runtime environment registry into blueprint/global runtime configuration without requiring YAML or TOML files.
- [x] 3.4 Add tests for registering environments, resolving Python interpreter material, resolving native executable material, and missing-name diagnostics.

## 4. Blueprint Venv Placement

- [x] 4.1 Add a blueprint-level placement API for assigning Module classes to named Python runtime environments.
- [x] 4.2 Route placed Modules to named venv worker pools while unplaced Modules continue using the default worker pool.
- [x] 4.3 Preserve same-env worker sharing and prevent cross-env Module mixing within one worker process.
- [x] 4.4 Add integration tests with two Modules in one named venv pool and two Modules in distinct named venv pools.
- [x] 4.5 Verify stream wiring, Module refs, and RPC calls work for Modules placed in venv worker pools.

## 5. Native Module Runtime Environment Opt-In

- [x] 5.1 Extend `NativeModuleConfig` with optional runtime environment reference while preserving existing executable/build_command/cwd/extra_env fields.
- [x] 5.2 Define and test deterministic precedence when a native runtime environment and legacy native config fields are both provided.
- [x] 5.3 Update one Nix-backed native module test fixture or fake native module to resolve executable/build/env through a named runtime environment.
- [x] 5.4 Confirm existing NativeModule tests and existing Nix-backed native module configs continue to work unchanged.

## 6. Venv Module Packaging Convention and Demo

- [x] 6.1 Add a small separately packaged demo venv Module with its own `pyproject.toml` and package-local runtime helper.
- [x] 6.2 Make the demo Module import-safe by avoiding runtime helper imports at module import time.
- [x] 6.3 Add a demo blueprint that places the demo publisher Module into a named Python venv runtime environment and keeps a consumer Module in the default environment.
- [x] 6.4 Add demo verification showing the coordinator can import/build the blueprint.
- [x] 6.5 Add runtime demo verification showing the venv worker runs the package helper and communicates through normal DimOS streams or RPCs.

## 7. Documentation and Validation

- [x] 7.1 Document import-safe module file rules and the separately packaged venv Module convention.
- [x] 7.2 Document runtime environment registry usage for Python venv workers and Nix-backed native modules.
- [x] 7.3 Document phase-1 limitations: same-machine only, compatible DimOS/source versions required, and no remote deployment agent yet.
- [x] 7.4 Run focused worker, blueprint, native module, and demo tests.
- [x] 7.5 Run broader relevant test suite or document any skipped slow/hardware-dependent tests.
