## 1. Backend layout and LeRobot API cleanup

- [x] 1.1 Move the generic policy backend Protocol into the backend package layout and update imports.
- [x] 1.2 Move the LeRobot backend implementation under `backends/lerobot/` and update registry paths.
- [x] 1.3 Remove newly-added namespace `__init__.py` files that violate repository conventions.
- [x] 1.4 Replace dynamic LeRobot policy/processor imports and local typing Protocols with official LeRobot API imports.
- [x] 1.5 Simplify LeRobot processor setup so processors are always prepared through the official factory path.

## 2. Policy observation and backend output model cleanup

- [x] 2.1 Rename `RobotLearningSample` to `RobotPolicyObservation` and remove benchmark-specific fields plus the top-level `task` field.
- [x] 2.2 Keep language prompts in observation metadata or observation roles and update the VLA-JEPA contract to read them there.
- [x] 2.3 Change `BackendOutputEnvelope.output` to `tuple[float, ...]` and normalize backend tensor/array/list outputs at the backend boundary.
- [x] 2.4 Keep benchmark episode identifiers and tick information in evaluation-layer request/record objects instead of policy observations.

## 3. Remove unused contract description surface

- [x] 3.1 Remove `RobotPolicyContractDescription`, `RobotPolicyContract.describe()`, and concrete/fake contract description methods.
- [x] 3.2 Stop writing `contract_description.json` in benchmark artifacts and update docs/tests accordingly.

## 4. Validation and review follow-up

- [x] 4.1 Update targeted unit tests for renamed models, moved modules, constrained backend outputs, and removed contract descriptions.
- [x] 4.2 Fix the md-babel docs snippet so executable docs no longer fail on missing imports.
- [x] 4.3 Run targeted policy rollout tests, ruff, and OpenSpec validation.
- [x] 4.4 Commit and push the review cleanup.
- [x] 4.5 Reply to addressed PR review comments and explicitly leave the image transport/performance question for separate design follow-up.
