## Why

Simulator benchmark integrations currently sit behind HTTP runtime sidecars plus ad hoc SHM, payload-fetch, launch-script, and Rerun bridge paths. The newly added named runtime environment and venv/pixi module worker support makes it possible to move these runtimes to first-class DimOS Modules while preserving dependency isolation for Robosuite, LIBERO-PRO, and future simulator stacks.

## What Changes

- Introduce Simulator Runtime Modules: import-safe DimOS Modules hosted in named Python project runtime environments, with simulator-heavy imports executed only inside the placed worker.
- Add package-local simulator runtime blueprint helpers that register a default `PythonProjectRuntimeEnvironment` and place the simulator runtime module using the standard blueprint-level placement API.
- Move runtime control to DimOS RPC: `describe`, `reset`, synchronous `step`, and `score` are module RPCs rather than HTTP endpoints.
- Move runtime data to DimOS-native typed streams: camera images and depth publish as `dimos.msgs.sensor_msgs.Image`, camera models publish as `CameraInfo`, and motor states/runtime events flow through normal DimOS stream transports rather than raw NumPy RPC payloads, HTTP payload fetches, or script-local Rerun bridges.
- Preserve current runtime motor action semantics: `step()` accepts the runtime-derived ordered motor action frame and returns control/evaluation metadata while large observations publish on streams.
- Make HTTP removal a success gate for the migration: HTTP runtime sidecar servers, `RuntimeSidecarClient`, HTTP payload endpoints, and HTTP-first demo launch paths are removed as part of the change rather than preserved as a parallel runtime path.
- **BREAKING**: HTTP runtime sidecar APIs and launch paths are removed after their behavior is covered by Simulator Runtime Modules.

## Capabilities

### New Capabilities

- `simulator-runtime-modules`: Defines first-class simulator runtime modules hosted in named runtime environments, their control-plane RPCs, data-plane streams, placement helpers, stepping/thread-affinity rules, and migration/deletion gates.

### Modified Capabilities

- `runtime-robosuite-sidecar`: Reframe Robosuite from an HTTP sidecar target to a package that provides a Simulator Runtime Module and remove its HTTP server boundary once module coverage lands.
- `runtime-libero-pro-sidecar`: Reframe LIBERO-PRO from an HTTP sidecar target to a package that provides a Simulator Runtime Module, preserving asset validation/reset behavior through module RPCs and removing its HTTP server boundary.
- `runtime-scripted-demos`: Replace HTTP/SHM/payload-fetch demo acceptance with module-native placement, RPC stepping, DimOS stream observations, and removal of script-local sidecar orchestration.
- `runtime-protocol`: Preserve backend-neutral runtime models as shared schema/control payloads while decoupling the target transport from HTTP JSON and payload-fetch endpoints.

## Impact

- Affected packages: `packages/dimos-fake-runtime-sidecar`, `packages/dimos-robosuite-sidecar`, `packages/dimos-libero-pro-sidecar`, and later simulator runtime packages.
- Affected DimOS runtime code: `dimos/simulation/runtime_client/*`, runtime demo scripts under `scripts/benchmarks/`, typed stream/Rerun integration paths, and whole-body runtime adapter plumbing.
- Affected environment configuration: simulator packages will use `PythonProjectRuntimeEnvironment`, optional Pixi-backed uv project environments, `dimos runtime prepare`, and blueprint-level `.runtime_environments()` / `.runtime_placements()`.
- Affected tests/specs/docs: runtime sidecar import-boundary tests, HTTP endpoint tests, runtime scripted demo specs, runtime environment docs, runtime sidecar docs, and new module-native parity coverage.
