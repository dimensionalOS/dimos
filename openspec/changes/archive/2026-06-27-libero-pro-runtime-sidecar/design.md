## Context

DimOS already has a runtime-sidecar architecture where simulator-specific dependencies live outside the main DimOS process. The shared runtime protocol package defines backend-neutral models for runtime description, reset, step, observations, motor action/state frames, scores, artifacts, and errors. The Robosuite sidecar and demos prove the pattern for a narrow Panda Lift task, using a single-threaded HTTP server, runtime-derived motor metadata, `.npy` camera payload references, a local SHM motor bridge, and a `ControlCoordinator` loop.

LIBERO-PRO should reuse that boundary but cannot be treated as another in-process DimOS module. It relies on a LIBERO/Robosuite/Torch stack that is older and more constrained than the main DimOS environment, and its task identity comes from LIBERO benchmark suites, BDDL files, init states, language metadata, and prepared assets.

## Goals / Non-Goals

**Goals:**

- Provide a dedicated LIBERO-PRO runtime sidecar package that imports no main DimOS package and imports heavy LIBERO dependencies lazily.
- Support registered LIBERO-PRO suites in v1 with typed backend options for suite/task/init-state selection and runtime settings.
- Deliver the full local DimOS control demo path in v1: sidecar, runtime protocol endpoints, SHM motor bridge, `HardwareComponent(adapter_type="benchmark_runtime")`, `ControlCoordinator`, camera payloads, score output, artifacts, and teardown.
- Preserve the shared runtime protocol schema for v1 and keep it backend-neutral.
- Make asset preparation explicit and opt-in, while validating prepared assets before episode startup.
- Fail fast when the selected LIBERO-PRO task/controller cannot expose the canonical Panda joint-position plus gripper whole-body motor surface.

**Non-Goals:**

- Dynamic LIBERO-PRO perturbation generation through `perturbation.create_env(...)`.
- Silent action adaptation from DimOS joint targets into OSC pose actions.
- Automatic asset download or filesystem mutation during sidecar startup or `/health`.
- Requiring real LIBERO-PRO dependencies/assets in normal CI.
- Adding a new `dimos benchmark` CLI command.
- Extending shared runtime protocol models with LIBERO-PRO-specific task objects in v1.

## Decisions

### Dedicated sidecar package

Create `packages/dimos-libero-pro-sidecar/` with its own `pyproject.toml`, import-safe package, and console scripts. The package depends on `dimos-runtime-protocol` by default and declares LIBERO-PRO/runtime extras where feasible.

Alternatives considered:

- Extend `dimos-robosuite-sidecar`: rejected because LIBERO-PRO adds benchmark-suite/task/asset machinery and an older dependency stack that should not be coupled to the current Robosuite sidecar profile.
- Host LIBERO-PRO as a DimOS Module or venv module worker: rejected for v1 because the sidecar boundary already matches simulator ownership and avoids main DimOS module import/build lifecycle coupling.

### Registered-suite v1 scope

Use `libero.benchmark.get_benchmark(benchmark_name)(task_order_index)` and `benchmark.get_task(task_index)` to resolve BDDL path, language, init states, and task metadata. Reset creates `OffScreenRenderEnv`, calls `reset()`, and applies `set_init_state(init_states[init_state_index])`.

Dynamic perturbation generation remains a follow-up because it materializes temporary assets and introduces additional validation/failure modes before the base runtime boundary is proven.

### Typed backend options outside shared protocol models

Add a `backend_options` field to local benchmark episode config and validate LIBERO-PRO values through a sidecar/config-local `LiberoProBackendOptions` model. Common top-level episode fields remain only the values DimOS reasons over generically: backend, episode id, runtime endpoint, robot id, degree of freedom count, control rate, tick count, and artifact directory.

The runtime protocol remains unchanged. `EpisodeResetRequest.options`, `RuntimeDescription.metadata`, `StepResponse.info`, and `ScoreOutput.metrics` are expressive enough for v1 metadata. Shared protocol extensions are deferred until multiple backends need typed benchmark-task semantics or sidecar-owned artifact endpoints.

### Canonical motor surface: Panda joint-position plus gripper

The LIBERO-PRO full-control demo requires an ordered whole-body motor surface compatible with Panda joint-position commands plus gripper. The sidecar validates the selected controller/action space against the expected motor order and action dimension. If LIBERO-PRO exposes only OSC pose control or an incompatible action surface for the chosen task, setup fails with a clear protocol error.

This preserves DimOS's `Whole-body motor surface` contract and avoids a hidden policy layer that would translate joint commands into task-space pose commands.

### Explicit runtime asset bootstrap

Add an optional asset preparation command or demo flag, such as `--prepare-assets`, that uses Hugging Face APIs where enabled and supports manual/prepared paths. Startup and `/health` only validate assets and report clear failures; they do not download or mutate local asset layout.

The sidecar validates the presence of required BDDL and init-state assets for the selected registered suite/task before starting the episode.

### Sidecar-owned scoring

The LIBERO-PRO sidecar owns success extraction because it has direct access to the backend env/task APIs. `/score` exposes normalized success, reward/score, step count, and task metadata such as benchmark name, task name, language, and init-state index. The DimOS demo records this score instead of inferring success from rewards, observations, or images.

### Verification split

Always-on tests cover import boundaries, typed backend option validation, stubbed sidecar endpoint behavior, motor-surface validation, score shape, and config resolution. Real LIBERO-PRO dependency/data tests are optional/manual because they require a prepared external environment and assets.

## Risks / Trade-offs

- Controller/action mismatch → Fail before episode start unless the chosen LIBERO-PRO env exposes Panda joint-position plus gripper with the expected action dimension and motor order.
- Old dependency stack fragility → Keep LIBERO-PRO dependencies isolated in a dedicated sidecar environment and test import boundaries so main DimOS remains unaffected.
- Asset layout drift or missing data → Provide explicit asset validation and optional bootstrap, and make setup failures visible instead of hiding them behind startup downloads.
- Untyped metadata/options can become brittle → Use local typed `LiberoProBackendOptions` now; revisit shared protocol extensions only when typed benchmark-task semantics become cross-backend.
- Optional real integration coverage may miss environment-specific failures in CI → Keep stub tests strict and provide a documented manual marker/demo for real prepared environments.
- Full-control v1 is larger than a smoke-only milestone → Keep dynamic perturbations and protocol extensions out of scope so the added work stays focused on the selected registered-suite control path.

## Migration Plan

1. Add backend-options support in local benchmark episode config without removing existing top-level Robosuite fields.
2. Add the LIBERO-PRO sidecar package and stub-friendly HTTP endpoint implementation.
3. Add LIBERO-PRO config/demo artifacts and tests while preserving existing fake and Robosuite demos.
4. Keep optional real LIBERO-PRO integration tests/manual demos behind explicit markers or commands.

Rollback is straightforward because the change is additive: remove the new package, config/demo entries, tests, and `backend_options` usage while leaving existing runtime protocol and Robosuite paths intact.

## Open Questions

- The exact LIBERO-PRO controller config name and action-vector layout must be verified in a real prepared LIBERO-PRO environment before marking the full-control demo as manually passing.
- The final asset bootstrap CLI shape can be a sidecar console script, demo flag, or both, but it must remain explicit opt-in.
