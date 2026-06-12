## 1. Dependencies and backend wiring

- [x] 1.1 Add a separate optional RoboPlan dependency extra with Git-sourced RoboPlan packages for core, RRT, simple IK, TOPPRA, OINK, and example models.
- [x] 1.2 Add lazy RoboPlan import helpers that raise actionable install-hint errors only when RoboPlan functionality is requested.
- [x] 1.3 Add `world_backend` to manipulation module configuration with `drake` as the default.
- [x] 1.4 Pass `world_backend` from the manipulation module into `WorldMonitor` and world factory construction.
- [x] 1.5 Extend world factory behavior to construct Drake or RoboPlan worlds by explicit backend name.
- [x] 1.6 Add startup validation for invalid world/planner/kinematics combinations, including `planner_name="roboplan"` requiring `world_backend="roboplan"`.

## 2. RoboPlan world implementation

- [x] 2.1 Add the RoboPlan-backed world module and DimOS-to-RoboPlan robot/model bookkeeping.
- [x] 2.2 Implement robot registration, robot ID/config queries, finalization, and joint-limit extraction with explicit joint-order mapping.
- [x] 2.3 Implement live and scratch context adapters with per-context collision scratch and geometry-revision invalidation.
- [x] 2.4 Implement joint-state synchronization, setting, and querying for live and scratch contexts.
- [x] 2.5 Implement supported obstacle add, update, remove, clear, and list operations with clear errors for unsupported geometry.
- [x] 2.6 Implement collision-free configuration checks backed by RoboPlan collision queries.
- [x] 2.7 Implement collision-free edge checks backed by RoboPlan path collision queries.
- [x] 2.8 Implement straightforward FK/link-pose/Jacobian mappings if the installed RoboPlan bindings expose compatible APIs.
- [x] 2.9 Leave obscure or unverified non-critical query methods explicitly unsupported with `NotImplementedError`, especially signed minimum-distance behavior if no safe equivalent is available.

## 3. Planner integration

- [x] 3.1 Verify `world_backend="roboplan"` with `planner_name="rrt_connect"` uses the existing generic planner against RoboPlan world collision checks.
- [x] 3.2 Add RoboPlan-native planner selection so `planner_name="roboplan"` uses the RoboPlan world object as the planner when the world backend is RoboPlan.
- [x] 3.3 Convert DimOS start/goal joint states and RoboPlan-native paths into existing planning result and joint path data models.
- [x] 3.4 Add clear errors for RoboPlan-native planner options or robot models unsupported by the installed bindings.

## 4. Tests

- [x] 4.1 Add unit tests for backend factory selection, unknown backend names, lazy import error messages, and invalid backend combinations.
- [x] 4.2 Add RoboPlan world tests for robot registration, finalization, joint-limit mapping, context cloning, obstacle mutation invalidation, and joint-state round trips.
- [x] 4.3 Add tests proving unsupported planning-critical inputs fail before planning and are not silently ignored.
- [x] 4.4 Add an acceptance test where RoboPlan world plus the generic RRT planner plans a collision-checked joint path for a supported test model.
- [x] 4.5 Add tests for RoboPlan-native planner wiring if native planning is implemented in the same iteration.
- [x] 4.6 Confirm existing Drake default planning tests continue to pass without RoboPlan installed.

## 5. Documentation

- [x] 5.1 Update user-facing manipulation planning docs with backend selection, valid combinations, and `dimos run` module option examples.
- [x] 5.2 Document RoboPlan optional Git dependency installation and note that package publication/pinning is separate follow-up work.
- [x] 5.3 Document safety behavior for unsupported robot, joint, obstacle, collision, and query-method features.
- [x] 5.4 Update contributor or coding-agent docs only if implementation introduces backend-extension conventions not already covered by user/developer docs.

## 6. Verification

- [x] 6.1 Run `openspec validate add-roboplan-planning-backend`.
- [x] 6.2 Run focused pytest targets for manipulation planning factories, world monitor, RoboPlan world, and planner integration.
- [x] 6.3 Run existing Drake manipulation planning tests to verify default behavior is unchanged.
- [x] 6.4 Run mypy or targeted type checks for changed manipulation planning modules if the touched files are type-checked.
- [x] 6.5 Run docs validation commands for changed docs, including `md-babel-py run <doc>` for executable snippets if applicable.
- [x] 6.6 Manually QA the user-facing surface with a `dimos run ... -o manipulationmodule.world_backend=roboplan -o manipulationmodule.planner_name=rrt_connect` style command in simulation/replay or a safe dry-run environment.
- [x] 6.7 If implementation adds or renames blueprints, run `pytest dimos/robot/test_all_blueprints_generation.py`; otherwise confirm no registry regeneration is needed.
