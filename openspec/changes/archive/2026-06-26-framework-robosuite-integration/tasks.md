## 1. Package and dependency boundaries

- [x] 1.1 Create `packages/dimos-runtime-protocol` as a lightweight installable package with its own `pyproject.toml`, `src/`, and tests.
- [x] 1.2 Create first-class sidecar package skeletons for `packages/dimos-robosuite-sidecar` and the fake/demo sidecar support without depending on the main `dimos` package.
- [x] 1.3 Add optional dependency wiring or developer documentation so DimOS can install the runtime protocol package while Robosuite sidecar environments can install only the protocol plus sidecar package.
- [x] 1.4 Add import-boundary tests that prove `dimos_runtime_protocol` imports without importing `dimos`, Robosuite, LIBERO-PRO, or OmniGibson.

## 2. Runtime protocol

- [x] 2.1 Define Pydantic protocol models for handshake, runtime description, episode reset, step request, step response, robot motor surfaces, motor action frames, motor state frames, observation frames, score output, artifact output, and protocol errors.
- [x] 2.2 Add protocol version and capability compatibility checks used during sidecar handshake.
- [x] 2.3 Implement binary-friendly codec helpers for protocol envelopes and small numeric arrays, with a path for image/depth payload references or binary payloads.
- [x] 2.4 Add protocol validation tests for malformed step requests, incompatible versions, robot surface descriptions, and observation frame metadata.

## 3. DimOS-side runtime client and local motor bridge

- [x] 3.1 Implement a DimOS-side runtime client that connects to a sidecar endpoint, performs health/handshake, resets an episode, exchanges step frames, and retrieves score/artifact metadata.
- [x] 3.2 Implement the local SHM motor bridge between the runtime client module and a WholeBodyAdapter-facing local motor data plane.
- [x] 3.3 Implement or register a WholeBodyAdapter that reads `MotorState[]` and writes `MotorCommand[]` through the local SHM bridge for benchmark runtime use.
- [x] 3.4 Add observation publishing hooks that translate protocol observation frames into DimOS streams or demo artifacts without exposing simulator SDK objects.
- [x] 3.5 Add unit tests for local motor command/state round-trips and runtime client error handling.

## 4. Prelaunch orchestration and resolved plans

- [x] 4.1 Define `BenchmarkEpisodeConfig` for backend intent, including backend, task, robot profile, control timing, observation streams, evaluator expectations, and artifact destination.
- [x] 4.2 Define `ResolvedRuntimePlan` containing derived hardware components, runtime client config, observation stream config, evaluator config, artifact routing, and sidecar metadata.
- [x] 4.3 Implement prelaunch orchestration that starts the sidecar, waits for health, retrieves runtime description, validates robot profiles, builds the resolved runtime plan, launches the DimOS blueprint directly, monitors both runtimes, and tears both down.
- [x] 4.4 Add failure handling for sidecar health timeout, protocol incompatibility, robot profile mismatch, early DimOS exit, early sidecar exit, and teardown errors.
- [x] 4.5 Add artifact writers for episode config, runtime description, resolved plan, protocol trace summary, motor trace, score output, and logs.

## 5. Fake sidecar smoke demo

- [x] 5.1 Implement a fake sidecar that speaks the runtime protocol, reports a deterministic whole-body motor surface, accepts motor actions, returns synthetic motor states, and provides score/artifact metadata.
- [x] 5.2 Add a plain demo script for the fake sidecar that loads config, starts the fake sidecar, prelaunches DimOS, runs a scripted motor sequence for a fixed number of ticks, collects artifacts, and tears down both runtimes.
- [x] 5.3 Add fake-sidecar demo config under an appropriate benchmark config directory.
- [x] 5.4 Add automated or documented smoke validation showing the fake demo runs without Robosuite installed and writes expected artifacts.

## 6. Robosuite sidecar integration

- [x] 6.1 Implement the Robosuite sidecar server package entrypoint that owns Robosuite environment construction, reset, step, scoring metadata, and artifact export.
- [x] 6.2 Implement Robosuite task/profile resolution for baked scenes such as `Lift` with `Panda`, controller profile, control frequency, horizon, cameras, renderer options, and seed.
- [x] 6.3 Implement runtime-derived motor surface discovery for the Panda joint-position + gripper profile, including validation against supported command modes.
- [x] 6.4 Implement action mapping from runtime motor position frames to Robosuite action vectors and state mapping from Robosuite observations to runtime motor state frames.
- [x] 6.5 Implement observation export for configured Robosuite camera/state observations, including at least `agentview` metadata or frame output.
- [x] 6.6 Add sidecar tests or simulator-gated checks for profile validation, unsupported controller failure, and Robosuite action/state mapping.

## 7. Robosuite Panda Lift plumbing demo

- [x] 7.1 Add a plain Robosuite Panda Lift demo script that orchestrates the Robosuite sidecar, derives the runtime plan, launches the DimOS blueprint directly, runs a scripted motor command sequence, collects score/artifacts, and tears down both runtimes.
- [x] 7.2 Add Robosuite Panda Lift demo config with backend `robosuite`, env `Lift`, robot `Panda`, joint-position controller profile, 100 Hz requested control, horizon, seed, and camera stream settings.
- [x] 7.3 Verify the Robosuite demo records matching motor order/count, motor state changes from scripted commands, observation frame receipt, score metadata, protocol trace summary, and cleanup status.
- [x] 7.4 Document the sidecar environment setup and exact command to run the Robosuite demo script.
- [x] 7.5 Add and validate an optional Robosuite Rerun demo mode that fetches `.npy` camera payloads from the sidecar and publishes `color_image`/`camera_info` through DimOS streams for Rerun visualization.
- [x] 7.6 Isolate the Robosuite Rerun demo from unrelated LCM/Rerun streams and bound raw-image memory usage with configurable Rerun memory and publish-rate settings.

## 8. Roadmap and validation

- [x] 8.1 Update `openspec/drafts/agentic-skill-benchmark-harness.md` as a roadmap document that references this change as the first concrete framework + Robosuite integration slice.
- [x] 8.2 Add concise developer documentation for the architecture boundaries: remote runtime protocol, local SHM motor bridge, prelaunch orchestration, and sidecar package isolation.
- [x] 8.3 Run relevant unit tests and the fake sidecar smoke demo.
- [x] 8.4 Run or document the Robosuite Panda Lift plumbing demo validation in an environment with Robosuite available.
