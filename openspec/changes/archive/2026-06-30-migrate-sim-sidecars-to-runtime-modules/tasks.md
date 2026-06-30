## 1. Shared Simulator Runtime Module Foundations

- [x] 1.1 Define import-safe shared runtime module interfaces, request/response type aliases, and stream naming conventions for `describe`, `reset`, `step`, `score`, motor state, `Image` observations, `CameraInfo`, and runtime events.
- [x] 1.2 Add a simulator owner-thread/executor utility or pattern for marshalling reset, step, render, and publish work from RPC handlers to the simulator owner thread.
- [x] 1.3 Add contract tests proving coordinator imports of simulator runtime module classes and blueprint helpers do not import simulator-heavy dependencies.
- [x] 1.4 Add contract tests proving `step()` responses remain lightweight and large observations are emitted as DimOS `Image`/`CameraInfo` streams rather than raw NumPy arrays or RPC payloads.

## 2. Fake Runtime Module Migration

- [x] 2.1 Add `FakeRuntimeModule` in the fake runtime package by wrapping existing fake runtime state behind DimOS RPCs and typed stream outputs.
- [x] 2.2 Add a package-local fake runtime blueprint helper that registers the runtime environment, places only `FakeRuntimeModule`, and allows environment override.
- [x] 2.3 Migrate the fake runtime smoke demo to use the module-native path instead of HTTP client/payload fetch plumbing.
- [x] 2.4 Add tests for fake runtime placement, reset, synchronous step, score, motor state publication, and observation/event stream publication.

## 3. Robosuite Runtime Module Migration

- [x] 3.1 Add import-safe `RobosuiteRuntimeModule` and package-local blueprint helper in `packages/dimos-robosuite-sidecar`.
- [x] 3.2 Wrap existing `RobosuiteRuntimeState` logic for describe, reset, step, motor surface validation, scoring, and observation generation.
- [x] 3.3 Ensure all Robosuite `env.reset`, `env.step`, render, and camera capture operations run through the simulator owner-thread path.
- [x] 3.4 Publish Robosuite camera images as `Image`, camera metadata as `CameraInfo`, motor states, and runtime events through typed DimOS output streams.
- [x] 3.5 Migrate the Robosuite Panda Lift demo to use the package-local blueprint helper, runtime placement/preparation guidance, module RPCs, and DimOS streams.
- [x] 3.6 Add Robosuite contract tests for import boundaries, runtime placement, action validation, owner-thread marshalling, stream publication, score shape, and HTTP server removal gates.

## 4. LIBERO-PRO Runtime Module Migration

- [x] 4.1 Add import-safe `LiberoProRuntimeModule` and package-local blueprint helper in `packages/dimos-libero-pro-sidecar`.
- [x] 4.2 Wrap existing LIBERO-PRO backend/runtime state for registered task selection, asset validation, reset, step, motor surface validation, scoring, and observation generation.
- [x] 4.3 Make missing LIBERO-PRO assets fail synchronously through module reset/setup validation with actionable errors.
- [x] 4.4 Ensure all LIBERO-PRO reset, init-state application, step, render, and camera capture operations run through the simulator owner-thread path.
- [x] 4.5 Publish LIBERO-PRO camera images as `Image`, camera metadata as `CameraInfo`, motor states, and runtime events through typed DimOS output streams.
- [x] 4.6 Migrate the LIBERO-PRO demo to use the package-local blueprint helper, explicit asset preparation, runtime placement/preparation guidance, module RPCs, and DimOS streams.
- [x] 4.7 Add always-on LIBERO-PRO contract tests with stubs and optional/manual real integration coverage for the placed module path.

## 5. Demo, Documentation, and Compatibility Cleanup

- [x] 5.1 Update runtime sidecar/runtime environment docs to describe Simulator Runtime Modules as the target architecture and HTTP runtime API removal as a migration success gate.
- [x] 5.2 Update runtime scripted demo specs/docs to remove HTTP-first acceptance criteria after module-native demos pass.
- [x] 5.3 Remove script-local Rerun publishers and HTTP payload-fetch paths from migrated demos once equivalent DimOS streams are verified.
- [x] 5.4 Audit all references to `RuntimeSidecarClient`, `/payloads`, HTTP sidecar launch commands, and `shm_motor.py` to classify each as migrated, removed, or non-runtime benchmark usage.
- [x] 5.5 Delete HTTP runtime server entrypoints, HTTP client code, HTTP payload endpoint tests, and HTTP-first runtime docs/spec language after module-native coverage lands.

## 6. Verification

- [x] 6.1 Run focused unit and contract tests for runtime environments, venv/module placement, fake runtime module, Robosuite runtime module, and LIBERO-PRO runtime module.
- [x] 6.2 Run the fake module-native smoke demo in the normal DimOS environment.
- [x] 6.3 Run the Robosuite Panda Lift module-native demo in a prepared Robosuite runtime environment.
- [x] 6.4 Document and gate the optional/manual LIBERO-PRO module-native integration on prepared dependencies and assets; always-on stub coverage remains the archive gate in environments without LIBERO assets.
- [x] 6.5 Run `openspec status --change "migrate-sim-sidecars-to-runtime-modules"` and confirm all required artifacts remain complete.
