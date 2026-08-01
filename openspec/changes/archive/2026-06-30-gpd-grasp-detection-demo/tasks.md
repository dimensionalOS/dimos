## 1. Package and runtime setup

- [x] 1.1 Create the import-safe `packages/dimos-gpd-grasp-demo/` Python project with `pyproject.toml`, `pixi.toml`, package source, and local `dimos` dependency wiring.
- [x] 1.2 Declare the pinned GPD dependency and native/Pixi dependencies needed for grasp detection from pointcloud inputs, reusing the proven GPD commit and direct-reference packaging settings.
- [x] 1.3 Add package tests proving coordinator-safe imports, lazy `gpd.core` import behavior, and project-runtime launch material resolution.

## 2. GPD pointcloud-consuming grasp detector

- [x] 2.1 Implement an import-safe `GPDGraspGenModule` that consumes existing `PointCloud2` inputs, implements `GraspGenSpec.generate_grasps(pointcloud, scene_pointcloud)`, and runs in a project runtime.
- [x] 2.2 Add conversion utilities from DimOS `PointCloud2` to the GPD backend input representation with clear handling for empty or malformed pointclouds.
- [x] 2.3 Convert GPD backend results into a compatible `PoseArray` while preserving frame metadata and publishing optional candidate/debug outputs.
- [x] 2.4 Add deterministic unit tests using synthetic/sample pointclouds and stubbed GPD backend responses for non-empty, empty, and invalid-input cases.

## 3. Grasping workflow integration

- [x] 3.1 Wire the GPD generator into a package-local blueprint factory with `PythonProjectRuntimeEnvironment` registration and placement for the generator Module.
- [x] 3.2 Add tests showing `GraspingModule.generate_grasps(...)` routes registered object pointclouds through the GPD generator via the existing `GraspGenSpec` pointcloud path.
- [x] 3.3 Verify `GraspingModule` publishes `PoseArray` results and returns explicit human-readable messages for empty or failed GPD outputs.

## 4. xArm MuJoCo demo

- [x] 4.1 Add an opt-in xArm MuJoCo GPD grasp demo blueprint modeled after `vgn_mujoco_grasp_demo`, using object registration, `GraspingModule`, the placed GPD generator, and `RerunBridgeModule`.
- [x] 4.2 Add a lightweight demo controller or reuse an existing controller pattern to wait for the configured registered grasp target and trigger `GraspingModule.generate_grasps(...)` without robot execution.
- [x] 4.3 Provide a documented command that runs the end-to-end demo with xArm MuJoCo simulation, existing pointcloud/object perception modules, GPD grasp detection, and Rerun visualization.
- [x] 4.4 Add blueprint-level tests proving the GPD demo is opt-in, uses stable remappings/output topics, places only the GPD generator into the project runtime, and does not modify existing VGN/xArm perception blueprints.
- [x] 4.5 Add an optional prepared-runtime integration test or demo smoke path that runs the full MuJoCo workflow when required runtime dependencies are available, skipping clearly otherwise.

## 5. Documentation and validation

- [x] 5.1 Document how to prepare the GPD grasp demo runtime and run the end-to-end xArm MuJoCo GPD grasp demo command.
- [x] 5.2 Document expected outputs, including grasp poses, optional candidate/debug outputs, empty-result behavior, and the fact that the first demo does not execute robot motion.
- [x] 5.3 Run focused tests for the GPD package, grasp generator, blueprint wiring, existing grasping/VGN tests, and runtime-environment regressions.
- [x] 5.4 Run ruff on touched files and validate the OpenSpec change strictly.
