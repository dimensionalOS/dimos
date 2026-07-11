## Why

The first LeRobot LIBERO rollout implementation proves policy inference and benchmark success, but its core `RobotPolicyModule` is a plain Python service class rather than a first-class DimOS `Module`. That makes the rollout path harder to compose in blueprints and blurs the reusable boundary needed for later simulator and real-world policy rollout.

This follow-up aligns the policy rollout stack with normal DimOS workflow: modules are configured through blueprints, heavy policy pieces are selected through registries, policy inputs use runtime-independent robot-learning samples, and policy outputs are runtime-independent policy actions.

## What Changes

- Convert the existing robot policy inference component into a real DimOS `Module` with RPC lifecycle and inference methods.
- Add lazy registries for policy backends and robot policy contracts, following the control task registry pattern.
- Replace benchmark-shaped policy input with a reusable `RobotLearningSample` boundary.
- Replace runtime protocol action output from the policy module with a reusable `RobotPolicyAction` boundary.
- Move LIBERO-specific conversion from runtime observations to `RobotLearningSample` into benchmark evaluation code as an explicit named sample-building seam.
- Add module-backed benchmark evaluation flow that can be launched by blueprint composition while preserving the existing 50-episode LeRobot LIBERO gate semantics.
- Preserve the script-based policy gate as a convenience entrypoint, but have it construct/run the DimOS module-backed workflow rather than directly wiring plain service objects.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `robot-policy-module`: make the policy module a first-class DimOS module, add registry-based backend/contract selection, define `RobotLearningSample` input and `RobotPolicyAction` output boundaries.
- `benchmark-policy-evaluation`: make benchmark evaluation use DimOS modules/blueprints for policy inference while retaining ownership of episode lifecycle, runtime stepping, scoring, artifacts, and gates.
- `runtime-scripted-demos`: update the LeRobot LIBERO demo requirement so the script exercises the module-backed evaluation path instead of a plain Python service stack.

## Impact

- Affected code: `dimos/robot_learning/policy_rollout/`, LeRobot LIBERO rollout script, benchmark policy evaluation tests, and OpenSpec specs.
- API impact: `RobotPolicyModule` construction becomes config/registry based; public inference consumes `RobotLearningSample` and returns `RobotPolicyAction` rather than benchmark-specific samples or `RuntimeActionFrame`.
- Runtime impact: benchmark execution remains lockstep and sidecar-native for v1; no ControlCoordinator action-surface integration is added in this change.
- Dependency impact: no new third-party dependency is expected; optional LeRobot/LIBERO dependency boundaries remain unchanged.
