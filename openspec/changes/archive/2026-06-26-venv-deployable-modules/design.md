## Context

`dimos run` currently creates a coordinator process that builds a blueprint, deploys Modules through `WorkerManagerPython`, and launches Python worker processes with `multiprocessing` forkserver. Those workers use a `Pipe` and pickled request/response dataclasses such as `DeployModuleRequest`, `CallMethodRequest`, and `WorkerResponse` to host one or more first-class DimOS Modules.

This architecture gives DimOS a clean coordinator/worker split, but all Python workers currently run from the same Python environment as the coordinator. Any Module class imported during blueprint construction can also force optional hardware, simulation, perception, or native-adjacent dependencies into the coordinator environment. Native modules already avoid some of this pressure by wrapping separately built executables, often declared by module-local `flake.nix` files, but Python Modules do not have an equivalent same-machine isolated runtime path.

The design focuses on same-machine separate Python environments first. Remote deployment, cluster scheduling, and transport changes are intentionally deferred, but the model should not block those later.

## Goals / Non-Goals

**Goals:**

- Run selected first-class DimOS Python Modules inside worker processes launched from named Python virtual environments.
- Preserve current blueprint orchestration, module lifecycle, RPC, refs, stream wiring, and worker message semantics.
- Keep placement as a blueprint/runtime decision rather than a permanent property of a Module class.
- Provide a Python-first runtime environment registry that can also describe Nix-backed native runtime material.
- Use separately packaged Python module distributions with their own `pyproject.toml` dependency closure as the Python analog to native module `flake.nix` files.
- Keep current native module launching coherent by making runtime environments resolve executable/build/env material while `NativeModule` remains responsible for native subprocess lifecycle.
- Include a small demo that proves the coordinator can build and wire a blueprint without installing a dependency that exists only in a venv worker.

**Non-Goals:**

- True remote machine deployment or remote worker agents.
- Zenoh, Kubernetes-style scheduling, or new data-plane transports.
- Replacing the existing worker protocol with an HTTP/gRPC/sidecar protocol.
- Requiring all DimOS packages to be split before phase 1.
- Making YAML/TOML config files the primary user API for runtime environments.

## Decisions

### Preserve the DimOS worker protocol; abstract worker launch and channel handling

The phase-1 implementation should keep the semantic worker protocol:

- `DeployModuleRequest`
- `SetRefRequest`
- `GetAttrRequest`
- `CallMethodRequest`
- `UndeployModuleRequest`
- `SuppressConsoleRequest`
- `ShutdownRequest`
- `WorkerResponse`

The current forkserver `Pipe` implementation should become one launch/channel implementation behind a worker process handle. Venv workers should launch with `subprocess.Popen(<venv-python> -m <worker entrypoint>)` and connect to the coordinator through `multiprocessing.connection.Listener/Client`.

Alternatives considered:

- Stdio framed pickle protocol: rejected for phase 1 because normal process logs can corrupt control messages unless stdout/stderr isolation is perfect.
- New sidecar service protocol: rejected because venv workers are still local DimOS workers, not arbitrary services.
- Replacing pickled request objects immediately: deferred because the same-code-version assumption makes the existing protocol a lower-risk spike path.

### Add worker launch abstractions

Introduce a launch/process boundary with concepts equivalent to:

- `WorkerLauncher`: starts a worker process for a particular launch environment.
- `WorkerProcessHandle`: owns send/recv, process liveness, shutdown, and cleanup.

The default launcher uses the current forkserver process and `Pipe`. The venv launcher uses a configured Python interpreter and a multiprocessing connection channel. `WorkerManagerPython` can keep scheduling and capacity behavior while delegating launch mechanics to the appropriate launcher.

Alternatives considered:

- A separate `WorkerManagerVenv`: possible later, but phase 1 should avoid duplicating scheduling and pool semantics.
- One worker process per venv Module: rejected because current workers can host multiple compatible Modules and that behavior should be preserved.

### Use blueprint-level placement into named runtime environments

Venv selection belongs to blueprint/runtime configuration, not to the Module class. The same import-safe Module may run in the default worker pool on a developer machine, a sensor venv on a robot, or a fake/minimal venv in CI.

Placement should reference a named runtime environment. Named runtime environments backed by Python venvs get dedicated worker pools; a pool may host multiple compatible Modules but MUST NOT mix Modules assigned to different Python environments.

Alternatives considered:

- `ModuleBase.deployment = "venv"`: rejected because it makes environment placement intrinsic to the class and overloads the current deployment backend concept.
- Hardcoded Python executable paths in blueprints: rejected because blueprints should remain portable topology/placement declarations.

### Make runtime environments Python API first

Runtime environments should be configured through typed Python objects first. Optional config-file loading can be added later as a convenience layer.

The registry maps stable names to environment backends. Examples of backends:

- current process environment
- Python venv environment
- Nix environment for native executable resolution
- future remote/container environments

Consumers request only the capability they need. A venv worker launcher asks a Python venv environment for a Python interpreter and environment variables. A `NativeModule` asks a Nix environment for executable/build/cwd/env material.

Alternatives considered:

- A venv-specific config such as `venv_workers`: rejected as too narrow because Nix-backed native modules are part of the same runtime environment management problem.
- Primary YAML/TOML environment files: rejected for now because DimOS blueprints are Python-first and should stay directly composable.

### Use separately packaged venv module distributions as declarative Python environment definitions

Python venv-deployable modules should be packaged as separate Python distributions with their own `pyproject.toml`. That `pyproject.toml` declares the module's Python dependency closure, similar to how a native module's `flake.nix` declares native build/runtime closure.

Phase 1 packages may depend on the current root `dimos` package plus module-specific dependencies. A later `dimos-worker-runtime` or smaller core package can reduce each venv package's base dependency closure when that package split is available.

Alternatives considered:

- Root `pyproject` extras as the only environment definitions: rejected because extras are additive fragments solved together in one environment, not isolated named outputs.
- Many in-code environment definition objects: rejected as too much central config surface compared with package-local `pyproject.toml` ownership.

### Require import-safe module files

A venv-deployable Module class must be importable by the coordinator environment. Heavy worker-only dependencies must not be imported at module import time. They should be imported inside runtime methods or helper paths reached only inside the worker environment.

This lets the coordinator inspect type hints, streams, refs, config, and RPC metadata without installing the worker-only dependency closure.

### Keep native module lifecycle in NativeModule

Runtime environment unification must not move native subprocess lifecycle into the registry. `NativeModule` remains responsible for topic collection, command construction, `subprocess.Popen`, log watching, and shutdown. Runtime environments only resolve launch material such as executable, cwd, env, and optional build/prepare command.

Legacy native config fields such as `executable`, `build_command`, `cwd`, and `extra_env` should remain supported during migration and may override or complement a named runtime environment.

## Risks / Trade-offs

- Pickle protocol compatibility across venvs → Mitigation: require compatible DimOS/source versions in phase 1 and add explicit version/import checks before deploying to a venv worker.
- Coordinator import-safety violations → Mitigation: document the rule and add tests that import venv-capable Module classes without worker-only dependencies installed.
- Root `dimos` package remains larger than ideal → Mitigation: allow phase-1 venv packages to depend on full `dimos`, then move to a smaller worker runtime package later.
- Runtime environment registry becomes too abstract → Mitigation: implement only the capabilities needed by venv worker launch and native executable resolution first.
- Native migration could break existing modules → Mitigation: keep current `NativeModuleConfig` fields and add runtime-env resolution as opt-in.
- Separate venv creation may be slow or brittle → Mitigation: phase 1 may bind to an already-created interpreter; package installation/creation automation can be added after the worker path is proven.

## Migration Plan

1. Add worker launch/process-handle abstractions without changing default forkserver behavior.
2. Implement venv worker launch behind an opt-in placement path.
3. Add typed runtime environment registry and Python venv backend.
4. Add demo package and blueprint showing dependency isolation.
5. Add native runtime environment resolution as opt-in while preserving existing native config fields.
6. Document package and import-safety conventions for venv modules.

Rollback is straightforward for phase 1: remove venv placements and run modules in the default worker pool, or keep using existing native config fields instead of named runtime environments.

## Open Questions

- Should phase 1 keep pickled module class deployment only, or add explicit import descriptors in the initial implementation for clearer diagnostics?
- What exact API name should represent blueprint placement: `placements`, `runtime_placements`, or a more DimOS-specific method?
- Should venv environment creation be part of phase 1, or should phase 1 require pre-created venvs and only verify them?
- What is the first real non-demo module package to migrate after the lightweight demo proves the path?
