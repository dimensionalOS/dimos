## Why

DimOS already has a backend-neutral runtime sidecar pattern and a Robosuite Panda demo, but LIBERO-PRO tasks require a separate old LIBERO/Robosuite/Torch stack, registered benchmark-suite task selection, prepared BDDL/init assets, and sidecar-owned success extraction. Supporting LIBERO-PRO through the same runtime boundary lets DimOS validate whole-body motor control, observations, scoring, and artifacts without importing LIBERO-PRO into the main DimOS environment.

## What Changes

- Add a dedicated LIBERO-PRO runtime sidecar package that depends on the runtime protocol package and isolates LIBERO-PRO dependencies from the main DimOS package.
- Support registered LIBERO-PRO suites in v1 using backend options for `benchmark_name`, `task_order_index`, `task_index`, `init_state_index`, controller, camera, horizon, asset paths, and validation behavior.
- Require the v1 full-control path to expose a Panda joint-position plus gripper whole-body motor surface; fail fast when a selected LIBERO-PRO environment/controller cannot provide that action contract.
- Add explicit opt-in runtime asset bootstrap support for preparing or validating LIBERO-PRO BDDL/init assets, while keeping startup and `/health` validation non-mutating.
- Add a script-hosted LIBERO-PRO ControlCoordinator + SHM demo equivalent to the Robosuite demo: sidecar startup, runtime description, reset, stepping, camera payload fetching, motor bridge loop, score collection, artifact writing, and teardown.
- Keep the shared runtime protocol schema backend-neutral and unchanged for v1; typed LIBERO-PRO validation lives in local config/sidecar/demo code.
- Split verification into always-on contract/import/stub tests and optional manual real LIBERO-PRO integration tests requiring prepared dependencies and assets.

## Capabilities

### New Capabilities
- `libero-pro-runtime-sidecar`: Dedicated LIBERO-PRO runtime sidecar behavior, task selection, asset validation/bootstrap, motor-surface validation, observation export, and sidecar-owned scoring.

### Modified Capabilities
- `benchmark-prelaunch-orchestration`: Add backend-specific options to benchmark episode config and require resolved plans to validate LIBERO-PRO sidecar metadata without adding LIBERO-specific top-level config fields.
- `scripted-runtime-demos`: Add a full LIBERO-PRO script-hosted runtime demo using the same ControlCoordinator, SHM motor bridge, observation payload, score, artifact, and teardown expectations as the Robosuite demo.

## Impact

- Adds `packages/dimos-libero-pro-sidecar/` with its own package metadata, console script, sidecar server, typed backend options, and optional asset bootstrap entrypoint.
- Updates benchmark runtime config parsing to support a backend-neutral `backend_options` field while preserving existing Robosuite/fake configs.
- Adds LIBERO-PRO benchmark config and script under benchmark runtime/demo locations.
- Adds always-on tests for import boundaries, backend option validation, sidecar HTTP contract behavior with stubs, action-surface failure, score shape, and config resolution.
- Adds optional/manual integration coverage for real LIBERO-PRO dependencies/assets and the full ControlCoordinator + SHM demo.
- Does not change `packages/dimos-runtime-protocol` wire models in v1.
