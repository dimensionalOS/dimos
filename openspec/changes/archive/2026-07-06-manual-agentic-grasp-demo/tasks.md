## 1. Agent-facing facade contracts

- [x] 1.1 Extend the agentic manipulation spec/contracts needed for pose motion, world-frame relative motion, home motion, set-gripper, scene scanning, grasp generation, and cached candidate execution.
- [x] 1.2 Add `AgenticGraspManipulationModule` in `dimos/manipulation/agentic_manipulation_module.py` while preserving the existing dependency-light `AgenticManipulationModule` behavior.
- [x] 1.3 Implement Round 1 skill metadata/docstrings for `scan_objects`, `generate_grasps`, `execute_grasp`, `move_to_pose`, `move_relative`, `move_along_axis`, `go_home`, `open_gripper`, `close_gripper`, and `set_gripper`.

## 2. Grasp candidate orchestration

- [x] 2.1 Implement `scan_objects(...)` delegation to object-scene/perception registration and return clear registered Grasp target summaries.
- [x] 2.2 Implement `generate_grasps(...)` delegation to the configured grasp generation path and cache generated Grasp candidates for later execution.
- [x] 2.3 Implement `execute_grasp(candidate_index=0)` as execution-only over cached candidates, including no-cache and invalid-index failures before motion commands.
- [x] 2.4 Implement the conservative execution sequence for a selected candidate: open gripper, move to pregrasp, move to grasp pose, close gripper, then lift or retract in world frame.
- [x] 2.5 Implement `move_relative(...)` and `move_along_axis(...)` with `frame="world"` defaults and clear unsupported-frame failures.

## 3. Demo wiring

- [x] 3.1 Add or update MuJoCo/GPD blueprint wiring for a manual execution demo that includes perception, GPD grasp generation, manipulation execution, and the grasp-capable agentic facade without requiring `McpClient`.
- [x] 3.2 Preserve the existing candidate-generation-only `gpd-mujoco-grasp-demo` behavior so it remains non-executing.
- [x] 3.3 Provide a manual driver path using MCP/tool calls or an equivalent deterministic helper sequence for `scan_objects("sphere")`, `generate_grasps("sphere")`, and `execute_grasp(0)`.

## 4. Tests and validation

- [x] 4.1 Add simulator-free unit tests for universal facade preservation and grasp-capable facade skill schema/delegation behavior.
- [x] 4.2 Add unit tests for cached Grasp candidate execution semantics: success path, missing cache, invalid index, and no implicit regenerate behavior.
- [x] 4.3 Add unit tests for world-frame relative motion defaults and unsupported-frame error handling.
- [x] 4.4 Add a gated MuJoCo/self-hosted smoke test that validates registered Grasp target discovery, non-empty cached candidates, and `execute_grasp(0)` pipeline completion without requiring object-lift assertion.
- [x] 4.5 Run focused default tests for agentic manipulation and grasp facade behavior.

## 5. Documentation

- [x] 5.1 Document runtime preparation and startup commands for the manual agent-facing GPD MuJoCo grasp demo.
- [x] 5.2 Document the manual command sequence for scanning the sphere, generating grasps, and executing candidate 0.
- [x] 5.3 Add the Rerun visual checklist: registered Grasp target, GPD Grasp candidates, robot approach, gripper close, and lift/retract motion.
- [x] 5.4 Clearly label autonomous LLM execution, physical robot grasp CI, object-lift pass/fail checks, collision-aware filtering, retries, `grab_object`, place/drop/pick-and-place, and raw plan execution as out of scope for this round.
