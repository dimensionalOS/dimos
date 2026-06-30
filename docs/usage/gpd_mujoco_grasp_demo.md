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
