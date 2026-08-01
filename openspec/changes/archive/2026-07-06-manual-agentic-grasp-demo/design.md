## Context

DimOS already has three relevant layers:

- `AgenticManipulationModule` exposes a small universal agent-facing manipulation facade for robot state, joint motion, speed, and gripper primitives.
- `GPDGraspGenModule` consumes registered-object pointclouds through `GraspGenSpec` and produces `PoseArray` grasp candidates for the current GPD MuJoCo demo.
- The existing `gpd-mujoco-grasp-demo` intentionally proves perception-to-GPD candidate generation only; it does not command robot motion or gripper execution.

The new round should prove a stronger but still deterministic claim: manual agent-facing tool calls can compose perception, GPD grasp generation, motion planning, and gripper execution in MuJoCo. It should not depend on an autonomous LLM agent, physical hardware, or contact-stable object-lift assertions.

## Goals / Non-Goals

**Goals:**

- Keep a single manipulation-facing tool namespace by adding the grasp-capable facade in `dimos/manipulation/agentic_manipulation_module.py`.
- Preserve the universal `AgenticManipulationModule` for blueprints that only provide basic motion/gripper dependencies.
- Add a separate `AgenticGraspManipulationModule` class for blueprints that also provide perception and grasp-generation dependencies.
- Expose Round 1 manual skills for scanning, grasp generation, cached candidate execution, pose motion, world-frame relative motion, home motion, and gripper control.
- Make `execute_grasp(candidate_index=0)` execution-only over cached candidates created by a previous `generate_grasps(...)` call.
- Add a deterministic MuJoCo manual demo path and gated smoke test that require pipeline completion, not object-lift success.
- Document the manual MCP/tool-call sequence and visual Rerun checklist.

**Non-Goals:**

- Autonomous LLM/McpClient execution.
- Physical robot grasp CI.
- Required object-lift/contact success assertion.
- Collision-aware GPD filtering.
- Multi-candidate retry policy.
- `grab_object(...)` macro.
- Place/drop/pick-and-place workflows.
- Raw plan execution APIs or arbitrary trajectory/code-as-policy interfaces.

## Decisions

### Keep the agent-facing surface in one file, split by dependency level

Add `AgenticGraspManipulationModule` in `agentic_manipulation_module.py` rather than creating a separate top-level demo controller module. This keeps the public agent-facing manipulation surface discoverable in one place while avoiding mandatory perception/GPD dependencies for the existing universal facade.

Alternatives considered:

- Extend `GraspingModule`: rejected because it would mix grasp generation with robot execution and agent-facing motion primitives.
- Extend only `PickAndPlaceModule`: rejected because this round is specifically about executing cached GPD candidates, not the existing heuristic pick pipeline.
- Put all dependencies directly on `AgenticManipulationModule`: rejected because blueprints with only basic motion/gripper providers would fail dependency injection.

### Treat `execute_grasp(...)` as execution-only over cached candidates

`generate_grasps(...)` owns grasp target resolution and candidate generation. `execute_grasp(candidate_index=0)` selects from the latest cached candidates and runs a conservative sequence: open gripper, move to pregrasp, move to grasp pose, close gripper, lift/retract.

This explicit sequence makes the manual demo debuggable and avoids hiding scan/regenerate behavior inside execution.

### Default relative motion to world frame

Round 1 relative motion uses `frame="world"` by default for `move_relative(...)` and `move_along_axis(...)`. This makes `dz > 0` a stable lift command in the sim demo and matches the current RoboPlan relative Cartesian support boundary.

EE-frame relative motion can be added later after axis conventions are documented and tested.

### Use tiered demo success criteria

The required smoke test verifies the pipeline completes: registered grasp target found, non-empty candidates generated, and `execute_grasp(0)` completes motion/gripper commands. Object visibly lifting in MuJoCo/Rerun is a demo-quality checklist item, not a CI gate.

This avoids making contact physics stability the blocker for validating module wiring and tool contracts.

### Keep the existing candidate-generation-only GPD demo intact

The current `gpd-mujoco-grasp-demo` remains valid and non-executing. The manual execution demo should be separate or opt-in so the previous documentation and smoke behavior stay honest.

## Risks / Trade-offs

- **Grasp pose convention mismatch** → Start with conservative pregrasp/lift distances, document the expected pose convention, and keep object-lift as visual quality rather than required CI success.
- **Dependency injection becomes brittle** → Keep the universal and grasp-capable facades as separate classes so basic blueprints do not require perception or GPD providers.
- **GPD may return unstable or empty candidates** → Smoke tests should fail clearly on empty candidates, while docs should describe runtime preparation and visual diagnostics.
- **Manual tool transport could be confused with autonomous agent success** → Name and document the demo as manual MCP/tool-call driven and explicitly out of scope autonomous LLM execution.
- **Execution sequence could duplicate future pick/place logic** → Keep Round 1 narrow; defer `grab_object(...)`, retries, and place/pick-and-place until candidate execution proves stable.

## Migration Plan

1. Add facade/spec contracts and unit tests for the new agent-facing skills without simulator dependencies.
2. Wire a MuJoCo/GPD manual demo blueprint or opt-in path that includes the grasp-capable facade and required perception/grasp/motion providers.
3. Add the gated self-hosted/MuJoCo smoke test and require only pipeline completion.
4. Update docs with runtime preparation, manual command sequence, and Rerun visual checklist.
5. Keep existing GPD candidate-generation demo available for users who only want detection visualization.

Rollback is straightforward: remove the new grasp-capable facade wiring, smoke test, and docs while leaving the existing universal facade and GPD detector unchanged.

## Open Questions

- Exact blueprint name and command shape for the manual execution demo.
- Whether the manual driver should call MCP CLI commands only, or use a small Python helper that performs equivalent tool calls against the running blueprint.
- Exact pregrasp/retract offsets and candidate orientation convention needed for stable visual demo behavior.
