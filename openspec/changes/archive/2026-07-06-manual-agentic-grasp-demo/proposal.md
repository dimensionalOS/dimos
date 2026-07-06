## Why

The existing GPD MuJoCo demo proves pointcloud-to-grasp-candidate generation, but it intentionally stops before robot motion and gripper execution. We need a deterministic, manual, agent-facing demo that proves the same tool surface an agent would use can compose registered object detection, GPD grasp generation, motion planning, and gripper execution in simulation without depending on an autonomous LLM loop.

## What Changes

- Add a grasp-capable agentic manipulation facade in the existing agent-facing manipulation module file, separate from the universal motion/gripper facade.
- Expose a small Round 1 skill surface for manual agent-facing grasp demos:
  - `scan_objects(...)`
  - `generate_grasps(...)`
  - `execute_grasp(candidate_index=0)`
  - `move_to_pose(...)`
  - `move_relative(dx, dy, dz, frame="world")`
  - `move_along_axis(axis, distance, frame="world")`
  - `go_home()`
  - `open_gripper()`
  - `close_gripper()`
  - `set_gripper(...)`
- Make `execute_grasp(candidate_index=0)` execution-only over cached grasp candidates produced by a prior `generate_grasps(...)` call.
- Add a manual MuJoCo demo path that runs a deterministic sequence through the agent-facing tools: `scan_objects("sphere") -> generate_grasps("sphere") -> execute_grasp(0)`.
- Add a gated MuJoCo/self-hosted smoke test that verifies pipeline completion without requiring object-lift success as a pass/fail assertion.
- Document the manual command sequence and Rerun visual checklist for demo-quality validation.
- Keep autonomous LLM/McpClient execution, physical robot grasp CI, object-lift pass/fail assertions, collision-aware GPD filtering, retry policy, `grab_object(...)`, place/drop/pick-and-place, and raw plan execution APIs out of scope for this round.

## Capabilities

### New Capabilities
- `manual-agentic-grasp-demo`: Defines the deterministic manual sim demo, smoke-test success criteria, and documentation/checklist for composing agent-facing grasp tools end-to-end.

### Modified Capabilities
- `manipulation-agentic-primitives`: Extends the agent-facing manipulation primitive surface with a grasp-capable facade class, pose motion, world-frame relative motion, named home motion, and set-gripper support.
- `gpd-grasp-detection`: Extends the GPD demo workflow from candidate-generation-only visualization to an optional manual execution demo that consumes cached candidates through the agent-facing facade, while preserving the existing non-execution demo behavior.

## Impact

- Affected modules/specs:
  - `dimos/manipulation/agentic_manipulation_module.py`
  - `dimos/manipulation/agentic_manipulation_spec.py`
  - grasping/perception specs used by the grasp-capable facade
  - xArm MuJoCo/GPD blueprint wiring
- Affected validation/docs:
  - agentic manipulation unit tests
  - gated MuJoCo/self-hosted smoke tests
  - GPD MuJoCo demo documentation and manual command sequence
- No breaking changes are intended for existing agentic manipulation primitives or the current candidate-generation-only GPD demo.
