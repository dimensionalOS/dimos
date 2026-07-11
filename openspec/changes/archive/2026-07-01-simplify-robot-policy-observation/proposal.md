## Why

PR review identified that the policy rollout module boundary still mixes policy inputs with benchmark bookkeeping and carries unused contract-description plumbing. Cleaning this up now keeps the new module API narrow before downstream code starts depending on the first rollout implementation.

## What Changes

- **BREAKING** Rename the policy input boundary from `RobotLearningSample` to an observation-focused model and remove unused or benchmark-specific fields, including `sample_id`, `episode_id`, `tick_id`, `task`, `task_id`, `task_index`, and `init_state_index`.
- Preserve policy language prompts as contract-specific observation metadata rather than a top-level generic `task` field.
- **BREAKING** Constrain backend output envelopes to flat numeric action tuples instead of arbitrary `object` output.
- Remove the unused robot policy contract description API and stop writing `contract_description.json` artifacts.
- Move backend interfaces/implementations into the backend package layout and simplify the LeRobot VLA-JEPA backend to use official top-level LeRobot imports and processor APIs.
- Keep image transport and large-observation streaming design out of scope for this cleanup; that will be handled separately.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `robot-policy-module`: Narrow the policy input/output contract, backend package layout, and LeRobot backend loading behavior.
- `benchmark-policy-evaluation`: Update benchmark evaluation artifacts and sample-building expectations after policy inputs stop carrying benchmark lifecycle fields.

## Impact

- Affected code: `dimos/robot_learning/policy_rollout/`, `scripts/benchmarks/demo_lerobot_libero_policy_rollout.py`, and rollout documentation/tests.
- API impact: policy input model and backend output envelope types change; contract description APIs and artifact output are removed.
- Dependency impact: no new dependencies; LeRobot optional backend imports follow the official `lerobot.policies` API when the backend module is loaded.
