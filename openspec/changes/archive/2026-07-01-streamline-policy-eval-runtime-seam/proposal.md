## Why

The merge to module-native simulator runtimes removed the HTTP sidecar boundary, but policy evaluation still carries an HTTP-shaped `RuntimeClient.payload()` seam and synthetic payload references to bridge DimOS streams into policy observations. This makes the code harder to reason about and preserves transport vocabulary that no longer matches the runtime architecture.

## What Changes

- Replace the policy-evaluation runtime client seam with a module-native runtime session interface that returns synchronous reset/step metadata plus stream snapshots for policy-observation building.
- Move camera and state observation collection for policy rollout out of HTTP-style payload lookup and into explicit DimOS stream snapshot handling.
- Update the LeRobot LIBERO policy rollout demo to depend on the same module-native observation seam, while preserving native runtime actions, success-gate behavior, optional videos, and artifact outputs.
- Remove deprecated HTTP-shaped policy-evaluation fields, helper names, and compatibility shims after the stream snapshot seam is in place.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `benchmark-policy-evaluation`: Runtime observation input changes from sidecar payload references to module-native stream snapshots.
- `runtime-scripted-demos`: The LeRobot LIBERO policy rollout demo must use the native runtime module observation seam without HTTP-style runtime flags or payload fetching.

## Impact

- Affected code: `dimos/robot_learning/policy_rollout/evaluation.py`, `scripts/benchmarks/demo_lerobot_libero_policy_rollout.py`, related policy rollout tests, and possibly small runtime stream capture helpers.
- APIs: internal benchmark policy-evaluation runtime seam changes; public rollout CLI should keep policy/gate flags but may remove or deprecate HTTP-only runtime host/port/startup options.
- Dependencies: no new runtime dependencies expected.
