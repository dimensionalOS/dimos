## Why

The current module-native live policy stream gate proves the topology works for `lerobot/VLA-JEPA-LIBERO`, but the concrete LeRobot backend and demo configuration still encode VLA-JEPA assumptions. Adding an X-VLA LIBERO gate gives us a second real policy family using the same RobotPolicyModule → ControlCoordinator live stream path, strengthening the claim that the implementation is policy-agnostic rather than VLA-JEPA-specific.

## What Changes

- Add support for loading and running the LeRobot X-VLA LIBERO checkpoint, expected as `lerobot/xvla-libero`, through the policy rollout backend/contract seams.
- Generalize the LeRobot LIBERO backend selection so policy class import, dependency guidance, checkpoint metadata, and inference method are not hardcoded to VLA-JEPA.
- Reuse the LIBERO observation/action contract where X-VLA has the same two-camera, 8D state, language prompt, and 7D normalized action semantics; introduce a separate contract only if X-VLA requires distinct semantic mapping.
- Extend the policy rollout script/configuration to select X-VLA as a real-policy gate candidate without replacing the existing VLA-JEPA gate.
- Add a two-stage X-VLA acceptance gate over the same 10-episode `libero_object` slice: first the synchronous benchmark path, then the module-native live policy stream path, each with `success_rate > 0.50`.
- Keep the synchronous benchmark path and existing VLA-JEPA gates available.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `robot-policy-module`: Generalize LeRobot backend support beyond VLA-JEPA so RobotPolicyModule can run X-VLA LIBERO through the same backend/contract interfaces and chunk output semantics.
- `benchmark-policy-evaluation`: Add an X-VLA synchronous benchmark gate that verifies the fast lockstep policy evaluation path before live stream validation.
- `runtime-scripted-demos`: Add an X-VLA LIBERO live policy stream parity gate that validates a second real LeRobot policy through the existing module-native rollout path.

## Impact

- Affected code: policy rollout backend registry and LeRobot backend, LIBERO policy contract selection if needed, `RobotPolicyModule` configuration tests, and `scripts/benchmarks/demo_lerobot_libero_policy_rollout.py` policy-selection arguments/tests.
- Affected docs/specs: LeRobot LIBERO rollout documentation and OpenSpec requirements for policy module backend support, synchronous benchmark gates, and live parity gates.
- Dependencies: X-VLA may require a LeRobot extra or optional dependency set distinct from `vla_jepa`; setup errors should fail clearly without adding X-VLA dependencies to the main `pyproject.toml`.
- Runtime systems: LIBERO runtime assets remain explicit/prepared; stage 1 uses the existing synchronous benchmark runner, and stage 2 uses the same module-native `LiberoProRuntimeModule`, deployed `RobotPolicyModule`, `ControlCoordinator`, streams, RPC trigger, and policy chunk task.
