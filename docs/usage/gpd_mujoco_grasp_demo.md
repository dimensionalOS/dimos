# GPD MuJoCo Grasp Demo

`gpd-mujoco-grasp-demo` is an opt-in xArm7 MuJoCo demo for pointcloud-based GPD grasp candidate generation. It does not change `xarm-perception-sim` or `vgn-mujoco-grasp-demo`, and it does not execute robot motion, gripper, pick/place, or trajectory APIs.

Prepare the optional GPD project runtime first:

```bash
uv run dimos runtime prepare gpd-mujoco-grasp-demo --runtime dimos-gpd-grasp-demo
```

Run the demo with MuJoCo simulation and Rerun visualization:

```bash
uv run dimos run gpd-mujoco-grasp-demo
```

The demo composes MuJoCo xArm simulation, existing object registration/pointcloud perception, `GraspingModule`, a placed `GPDGraspGenModule` in the `dimos-gpd-grasp-demo` project runtime, a pointcloud-only trigger controller, and `RerunBridgeModule`.

Expected Rerun-visible outputs after the registered target is detected and the controller triggers GPD include:

- `world/grasp_target_bounds`
- `world/gpd_grasp_candidates`

`GraspingModule` also publishes compatible `PoseArray` results on `grasps` for downstream code, but candidate visualization in Rerun comes from `gpd_grasp_candidates`.

If no target object is registered before the configured timeout, the controller logs an empty-result message and publishes empty target bounds. If GPD returns no candidates after a target is registered, the controller/module logs an explicit empty-result message and still avoids robot execution.

## Manual agent-facing execution demo

`manual-agentic-gpd-mujoco-grasp-demo` is a separate opt-in demo that adds motion execution and the `AgenticGraspManipulationModule` tool surface. It keeps `gpd-mujoco-grasp-demo` generation-only, and it still does not use `McpClient`, an autonomous LLM loop, or a model API key.

Prepare the same optional GPD runtime first:

```bash
uv run dimos runtime prepare manual-agentic-gpd-mujoco-grasp-demo --runtime dimos-gpd-grasp-demo
```

Start the manual execution demo:

```bash
uv run dimos run manual-agentic-gpd-mujoco-grasp-demo
```

Then run the deterministic RPC helper from another terminal. It calls the same agent-facing tools a manual caller would use:

```bash
uv run python -m dimos.manipulation.grasping.manual_agentic_gpd_grasp_demo \
  --target-name sphere \
  --candidate-index 0
```

Equivalent tool sequence:

1. `scan_objects("sphere")`
2. `generate_grasps("sphere")`
3. `execute_grasp(0)`

`execute_grasp` only executes candidates cached by the prior `generate_grasps` call. It does not implicitly rescan the scene or regenerate candidates.

### Rerun checklist

During a successful manual run, verify these outputs and motions in Rerun/MuJoCo:

- the sphere is registered as the Grasp target;
- GPD Grasp candidates appear on `world/gpd_grasp_candidates`;
- the robot approaches the selected candidate;
- the gripper closes at the grasp pose;
- the arm lifts/retracts in world frame after closing.

### Out of scope for this round

This demo does not claim autonomous LLM execution, physical robot grasp CI, object-lift pass/fail checks, collision-aware filtering, retries, `grab_object`, place/drop/pick-and-place, or raw plan execution APIs.
