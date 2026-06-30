## 1. Runtime Config and Planning

- [x] 1.1 Add `backend_options` support to `BenchmarkEpisodeConfig` without breaking existing fake and Robosuite configs.
- [x] 1.2 Add typed LIBERO-PRO backend option validation for benchmark name, task order index, task index, init-state index, controller, cameras, horizon, and asset roots.
- [x] 1.3 Update runtime plan resolution or demo-side validation so `libero-pro` plans fail before blueprint startup when backend, robot id, motor count, protocol version, or command modes mismatch sidecar metadata.
- [x] 1.4 Add a LIBERO-PRO benchmark config fixture for one registered suite/task using common top-level fields plus `backend_options`.

## 2. LIBERO-PRO Sidecar Package

- [x] 2.1 Create `packages/dimos-libero-pro-sidecar/` with package metadata, import-safe module structure, and console script entrypoints.
- [x] 2.2 Implement lazy LIBERO-PRO imports so importing the sidecar package does not import `dimos`, `libero`, `robosuite`, or `torch`.
- [x] 2.3 Implement sidecar configuration/state models for registered-suite task selection and runtime settings.
- [x] 2.4 Implement `/health`, `/describe`, `/reset`, `/step`, `/score`, and `/payloads/{id}` endpoints using the existing runtime protocol models.
- [x] 2.5 Keep the LIBERO-PRO HTTP server single-threaded for MuJoCo/Robosuite render-context safety.

## 3. LIBERO-PRO Task and Motor Runtime

- [x] 3.1 Resolve registered LIBERO-PRO tasks via `get_benchmark(benchmark_name)(task_order_index)` and `benchmark.get_task(task_index)`.
- [x] 3.2 Create and reset `OffScreenRenderEnv`, apply `set_init_state(init_states[init_state_index])`, and return initial protocol state/metadata.
- [x] 3.3 Validate the selected controller/action space exposes Panda joint-position plus gripper with the expected motor order and action dimension.
- [x] 3.4 Translate `MotorActionFrame` position commands into LIBERO-PRO environment actions and translate returned simulator state into `MotorStateFrame`.
- [x] 3.5 Export configured camera observations as runtime observation frames with fetchable `.npy` payload references.
- [x] 3.6 Normalize sidecar-owned score output with success, reward or score, steps, benchmark name, task name, language, and init-state index.

## 4. Asset Bootstrap and Validation

- [x] 4.1 Add explicit asset validation that detects missing BDDL and init-state assets for the selected registered suite/task before episode reset.
- [x] 4.2 Add an optional asset bootstrap command or demo flag that uses Hugging Face APIs where enabled and stages supported assets into the expected LIBERO-PRO layout.
- [x] 4.3 Ensure sidecar startup and `/health` validate assets without downloading or mutating local files unless explicit bootstrap was requested.
- [x] 4.4 Document manual/prepared asset path usage and bootstrap behavior in the sidecar or runtime-sidecar docs.

## 5. Full-Control Demo

- [x] 5.1 Add `scripts/benchmarks/demo_libero_pro_runtime.py` modeled on the Robosuite demo flow.
- [x] 5.2 Launch the LIBERO-PRO sidecar subprocess, wait for health, fetch runtime description, and derive the resolved runtime plan.
- [x] 5.3 Create the SHM motor owner and benchmark runtime hardware component for `HardwareComponent(adapter_type="benchmark_runtime")`.
- [x] 5.4 Run a scripted `ControlCoordinator` servo loop that sends Panda motor position targets, posts `/step`, writes returned motor state to SHM, and records motor/protocol traces.
- [x] 5.5 Fetch camera payloads and publish or record at least one camera observation through the runtime observation path.
- [x] 5.6 Request `/score`, write score/artifact outputs, and guarantee sidecar, coordinator, and SHM cleanup on success and failure.

## 6. Always-On Tests

- [x] 6.1 Add import-boundary tests ensuring the LIBERO-PRO sidecar package imports without loading `dimos`, `libero`, `robosuite`, or `torch`.
- [x] 6.2 Add backend-options validation tests for valid registered-suite config and missing/invalid LIBERO-PRO option failures.
- [x] 6.3 Add stubbed sidecar endpoint tests for `/describe`, `/reset`, `/step`, `/score`, and camera payload behavior without real LIBERO-PRO dependencies.
- [x] 6.4 Add action-surface validation tests covering compatible Panda joint-position plus gripper metadata and incompatible OSC/action-dimension failures.
- [x] 6.5 Add config/plan tests confirming existing fake and Robosuite configs still resolve after introducing `backend_options`.

## 7. Optional Real Integration

- [x] 7.1 Add optional/manual test marker coverage for launching a real LIBERO-PRO sidecar with prepared assets.
- [x] 7.2 Verify the full ControlCoordinator + SHM demo path against one registered LIBERO-PRO suite/task in a prepared environment.
- [x] 7.3 Record manual verification instructions, required environment assumptions, asset preparation steps, and expected artifacts.
