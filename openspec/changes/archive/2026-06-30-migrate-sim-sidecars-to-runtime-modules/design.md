## Context

DimOS currently has two overlapping simulator integration shapes:

- First-class DimOS simulator modules, such as Unity and in-tree MuJoCo modules, expose typed DimOS streams at the blueprint boundary while keeping backend protocols private.
- Benchmark runtime sidecars for fake, Robosuite, and LIBERO-PRO run in separate environments and communicate through HTTP JSON, referenced array payloads, local SHM motor bridges, and script-local Rerun stream publishers.

The runtime sidecar split successfully isolates heavy simulator dependencies from the main DimOS environment, but it leaves benchmark orchestration outside the module system and creates a second communication stack beside DimOS transports. The merged runtime environment work now provides the missing substrate: import-safe Python modules can be placed in named venv/uv/pixi-backed project environments through blueprint-level `runtime_environments()` and `runtime_placements()`.

The target architecture is therefore a Simulator Runtime Module: a first-class DimOS Module hosted in a named runtime environment. The simulator package owns backend reset, stepping, observations, scoring, and heavy imports inside its worker environment. The main DimOS process owns blueprint composition, worker placement, lifecycle, typed streams, and RPC coordination.

## Goals / Non-Goals

**Goals:**

- Migrate fake, Robosuite, and LIBERO-PRO benchmark runtimes from HTTP sidecars to placed DimOS Modules.
- Preserve dependency isolation: main DimOS imports must not require Robosuite, LIBERO-PRO, Torch, or simulator asset stacks.
- Use named `PythonProjectRuntimeEnvironment` entries, with optional Pixi-backed uv environments, for simulator packages that need external native/Python dependencies.
- Use package-local blueprint helpers to hide repeated placement boilerplate while preserving the standard blueprint-level placement API.
- Use DimOS RPC for benchmark control-plane calls: `describe`, `reset`, synchronous `step`, and `score`.
- Use DimOS-native typed streams/transports for large or continuous data: RGB/depth images are `Image` messages, camera models are `CameraInfo` messages, and motor state snapshots/runtime events use normal DimOS stream message types.
- Preserve current runtime motor action semantics: `step()` consumes the runtime-derived ordered motor action frame rather than backend-native opaque action vectors.
- Preserve simulator thread-affinity invariants by marshalling RPC work onto the simulator owner thread before calling backend reset/step/render APIs.
- Treat HTTP server/client removal as a migration success gate, not a preserved runtime mode.

**Non-Goals:**

- Do not create, sync, or mutate pixi/uv environments during `dimos run`; environment preparation remains explicit through `dimos runtime prepare` or documented commands.
- Do not make simulator placement an intrinsic `Module` class flag; placement remains blueprint-level.
- Do not introduce a new bespoke simulator transport.
- Do not require remote/multi-machine runtime deployment in this migration.
- Do not replace backend-neutral runtime protocol models immediately; reuse them as schema/control payloads where useful while moving the transport boundary into DimOS.
- Do not require benchmark task success by an agent; plumbing demos validate runtime behavior, not task-solving quality.

## Decisions

### 1. Target shape: Simulator Runtime Module, not HTTP sidecar

Each migrated runtime package exposes an import-safe DimOS Module class, for example `RobosuiteRuntimeModule` or `LiberoProRuntimeModule`. The coordinator imports the module class without importing simulator SDKs. Worker-side runtime methods lazily construct backend state inside the placed runtime environment.

Alternative considered: keep the HTTP sidecar and only standardize launch through the runtime environment registry. This preserves existing code but keeps a second control/data transport stack and prevents normal DimOS stream composition, so it is not the desired migration outcome.

### 2. Placement is hidden by package-local blueprint helpers

Callers should normally use a simulator package helper rather than manually writing runtime environment registration and placement for every blueprint:

```python
robosuite_runtime_blueprint(...)
```

The helper registers a default `PythonProjectRuntimeEnvironment` and returns a blueprint with `.runtime_environments(environment)` and `.runtime_placements({RobosuiteRuntimeModule: environment.name})`. Advanced callers can pass an override runtime environment.

Alternative considered: module-local deployment flags or a global sidecar registry. This conflicts with the current merged API and makes placement less composable across blueprints.

### 3. Control plane uses synchronous DimOS RPC

Runtime control calls are module RPCs:

- `describe()` reports protocol/capability metadata, motor surfaces, streams, and backend/task metadata.
- `reset(request)` establishes a benchmark episode and returns reset metadata or fails synchronously for setup errors such as missing LIBERO assets.
- `step(request)` advances one coordinated benchmark tick and blocks until the simulator step has completed.
- `score(request)` returns score/artifact metadata after episode completion or timeout.

Synchronous `step()` is intentional. DimOS RPC is already request/response over LCM pubsub, and benchmark control needs step completion before the runner can continue. Avoiding HTTP removes the separate request stack without forcing an asynchronous tick protocol.

Alternative considered: stream-driven tick commands. This is useful for autonomous or high-throughput simulation loops but complicates benchmark control that requires deterministic request/response stepping.

### 4. Data plane uses DimOS-native image streams, not RPC payloads

Large observations do not travel in `step()` responses. `step()` returns control/evaluation metadata such as episode id, tick id, reward, done/success, status, error details, and observation sequence/timestamp references. RGB images and depth images publish as `Out[Image]`; camera models publish as `Out[CameraInfo]`; point clouds, motor states, and runtime events publish through their typed DimOS `Out[...]` streams. Simulator internals may use NumPy arrays, but the module boundary should wrap image arrays in DimOS `Image` messages rather than exposing raw `np.ndarray` payloads.

Transport remains a blueprint choice. `LCMTransport(..., Image)` can carry raw `Image` message bytes for moderate rates; `JpegLcmTransport` or `JpegShmTransport` can be used for RGB camera streams where lossy compression is acceptable; raw `Image` over SHM/pickle-SHM is preferred for same-host high-rate images or depth data where JPEG is inappropriate.

Alternative considered: include arrays or payload references in RPC responses and fetch them through HTTP-like payload endpoints. That preserves current sidecar behavior but keeps the payload-fetch path and duplicates stream publication work.

### 5. Preserve runtime motor action frames during migration

The module `step()` input keeps the current `MotorActionFrame` semantics: robot id, command mode, ordered motor names, q/dq/kp/kd/tau fields, and sequence. This represents the simulator runtime's declared DimOS-facing whole-body motor surface. It is not a backend-native action vector.

Alternative considered: switch immediately to existing `MotorCommandArray` stream messages. That may become desirable for deeper whole-body unification, but forcing it into the first migration risks coupling simulator runtime parity to a separate motor API redesign.

### 6. Simulator ownership thread protects backend APIs

Module RPC handlers must not directly call Robosuite/MuJoCo/LIBERO rendering and stepping APIs from the LCMRPC thread-pool thread. Each runtime module owns a simulator executor/thread or main-thread loop. RPC handlers enqueue/marshal work to that owner and wait for completion; the owner thread performs reset/step/render/publish operations.

If a visual backend requires process main-thread ownership, the worker/runtime support should allow a dedicated main-thread module worker mode rather than hiding backend calls in arbitrary callback threads.

Alternative considered: call backend APIs directly from RPC handlers for simplicity. This violates known Robosuite/MuJoCo render-context constraints and risks corrupted frames or crashes.

### 7. HTTP removal is a success gate

The change is not complete while runtime HTTP servers, `RuntimeSidecarClient`, HTTP payload endpoints, or HTTP-first demo launch paths remain in active use. Deletion is a success gate: fake, Robosuite, and selected LIBERO-PRO module paths must cover import boundaries, runtime placement/preparation, control-plane calls, data-plane streams, and benchmark parity, then the old HTTP server/client surfaces are removed.

Alternative considered: leave the old HTTP surface in place after module migration. That would create two simulator runtime products and undermine the goal of unifying communication around DimOS RPC and streams.

## Risks / Trade-offs

- **Thread affinity bugs** → Use simulator owner-thread marshalling and add tests/smoke demos that exercise reset, step, render, and publish through the module RPC path.
- **Backpressure from camera streams** → Treat camera/depth streams as normal `Image`/`CameraInfo` DimOS streams with transport selection and throttling; avoid large RPC payloads.
- **Environment drift** → Keep `dimos run` non-mutating and fail fast with actionable `dimos runtime prepare` guidance when prepared `.venv/bin/python` or project files are missing.
- **Import boundary regressions** → Keep coordinator import tests that import simulator module/blueprint helpers without heavy simulator dependencies installed.
- **Ambiguous reset failure semantics** → Make `reset()` the synchronous authority for episode setup; asset/config failures return through RPC errors rather than event-topic timeouts.
- **Migration limbo with two APIs** → Make HTTP removal part of the success criteria, not optional cleanup.
- **Motor API churn** → Preserve `MotorActionFrame` initially and document it as the module control payload; defer any `MotorCommandArray` convergence to a separate change.
- **Script/demo churn** → Migrate fake first, then Robosuite Panda Lift, then LIBERO-PRO so each phase has a working demonstration before deleting old HTTP code.

## Migration Plan

1. Add module-native fake runtime package support first.
   - Add an import-safe `FakeRuntimeModule` wrapping existing fake runtime state.
   - Add a package-local blueprint helper using `PythonProjectRuntimeEnvironment` or current/default env where appropriate.
   - Prove RPC describe/reset/step/score and `Image`/`CameraInfo` stream outputs without simulator dependencies.
2. Migrate Robosuite Panda Lift.
   - Evolve `packages/dimos-robosuite-sidecar` in place with `module.py` and `blueprint.py`.
   - Wrap existing `RobosuiteRuntimeState` logic rather than rewriting backend mapping first.
   - Marshal reset/step/render/publish onto the simulator owner thread.
   - Publish `color_image`, optional `depth_image`, `camera_info`, and motor state through DimOS streams.
3. Migrate LIBERO-PRO.
   - Evolve `packages/dimos-libero-pro-sidecar` in place.
   - Preserve explicit asset preparation and make missing assets fail through `reset()`/setup RPC.
   - Keep real LIBERO-PRO coverage optional/manual while contract tests use stubs.
4. Move demos and tests to module-native paths.
   - Replace HTTP launch/client usage in scripted demos with blueprint helpers and `dimos runtime prepare` guidance.
   - Replace SHM/payload-fetch/Rerun bridge expectations with module streams.
5. Remove HTTP runtime surfaces as a success gate.
   - Remove HTTP entrypoints, `RuntimeSidecarClient`, HTTP payload endpoint tests, and HTTP-first docs/spec language.
   - Keep backend-neutral protocol models if they still serve as reusable schema payloads.

Rollback before the removal step means continuing work on the module-native path until it reaches parity. After removal, rollback means restoring the old HTTP surface from version control; removal should happen only when module-native demos, tests, and docs cover the required behavior.

## Open Questions

- Should score outputs become only RPC return values, or should selected score summaries also publish as runtime event streams for observability?
- Which concrete DimOS message types should represent runtime events and observation availability metadata in phase 1?
- Should Robosuite/LIBERO module packages initially depend on full `dimos` in their project runtime, matching venv packaging phase 1, or should they split a smaller runtime module support dependency later?
