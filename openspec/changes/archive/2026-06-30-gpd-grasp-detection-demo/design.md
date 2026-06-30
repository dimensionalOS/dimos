## Context

DimOS already has two grasp-generation paths:

- A pointcloud path in `GraspingModule.generate_grasps(...)` that calls `GraspGenSpec.generate_grasps(pointcloud, scene_pointcloud)` and publishes a `PoseArray`.
- A TSDF path where the VGN demo uses `SceneReconstructionModule`, `VGNGraspGenModule`, and `TargetGraspDemoController` to produce target-conditioned grasp candidates.

GPD is pointcloud-native, so it should not be modeled as another TSDF/VGN generator and it should not own pointcloud production. The first GPD grasp demo should prove that a real xArm MuJoCo perception workflow can route a registered grasp target's pointcloud, produced by existing perception modules, through the existing pointcloud grasp path into a project-runtime GPD worker. The user-facing deliverable must include a concrete command that starts the simulation, runs existing pointcloud/object perception, invokes GPD grasp detection from the resulting pointcloud, and makes the result visible in Rerun.

The prior GPD worker demo already proves that the pinned GPD binding can be prepared and imported in a Pixi-backed project runtime. This change builds on that runtime capability by adding a grasp-generation Module and a VGN-like workflow demo.

## Goals / Non-Goals

**Goals:**

- Add an import-safe GPD grasp detector Module in a package-local Python project runtime that consumes existing `PointCloud2` inputs.
- Implement the existing `GraspGenSpec` contract so `GraspingModule.generate_grasps(...)` can call GPD without a new user-facing grasp skill API.
- Preserve richer GPD/debug metadata through optional `GraspCandidateArray` or debug streams while returning `PoseArray` for compatibility.
- Add a deterministic sample/synthetic pointcloud test harness for conversion and result handling.
- Add an xArm MuJoCo demo blueprint, similar in spirit to `vgn_mujoco_grasp_demo`, that exercises object registration, `GraspingModule`, and the placed GPD generator.
- Provide a documented end-to-end command that runs the demo with simulation, existing pointcloud perception, GPD grasp detection, and Rerun visualization active.

**Non-Goals:**

- No robot execution, pick/place motion, or gripper actuation in the first GPD demo.
- No replacement of VGN or the TSDF grasp path.
- No broad migration of `GraspGenSpec` from `PoseArray` to `GraspCandidateArray`.
- No guarantee that GPD returns a non-empty grasp set for every test scene; empty results must be explicit and observable.

## Decisions

### Use the existing pointcloud `GraspGenSpec`

`GPDGraspGenModule` will implement `GraspGenSpec` and return `PoseArray | None` from `generate_grasps(...)`. This keeps the current `GraspingModule.generate_grasps(...)` skill path working and avoids broad API churn.

Alternative considered: introduce a new pointcloud candidate spec returning `GraspCandidateArray`. That is cleaner long-term but would require `GraspingModule` changes and more downstream contract decisions. This change may publish candidates for debug, but the required compatibility surface remains `PoseArray`.

### Keep GPD in a demo package first

The first grasp generator will live under a new package in `packages/`, not directly under `dimos/manipulation/grasping`. The package owns the GPD/Pixi dependency closure and must be import-safe in the coordinator environment.

Alternative considered: add `GPDGraspGenModule` directly to core grasping. That may be appropriate later, but a package-local first slice isolates native dependency risk and follows the project-runtime package pattern.

### Build two validation layers

The change will include both:

- A deterministic sample/synthetic pointcloud harness that tests GPD conversion/output handling without MuJoCo or perception timing.
- A headline xArm MuJoCo demo workflow that mirrors the VGN demo pattern and proves the end-to-end registered-object path reaches GPD.

The sample harness is the debugging floor. The MuJoCo workflow is the user-visible demo, and it must be runnable from a documented command rather than only existing as unit-test wiring.

### Stop at candidate generation

The demo produces and visualizes/logs grasp poses or a clear empty-result message. It does not execute a grasp. Current code has an opt-in VGN candidate demo but no proven pointcloud grasp execution path, and `PickAndPlaceModule.generate_grasps()` is not a suitable acceptance target for this slice.

## Risks / Trade-offs

- **GPD binding API mismatch** → Encapsulate binding calls in the demo package adapter and test conversion with stubbed and prepared-runtime paths.
- **Pointcloud frame or unit mismatch** → Validate `PointCloud2` to GPD conversion, preserve frame ids in returned `PoseArray`, and document expected input frame behavior.
- **GPD returns no grasps for the default scene** → Treat empty output as a valid explicit result, but keep tests that prove the call path and published outputs are observable.
- **Native dependency friction** → Reuse the Pixi-backed project runtime pattern and keep optional Pixi/GPD integration tests skippable when the runtime is not prepared.
- **Contract loses GPD scoring metadata** → Publish optional debug candidates while preserving the existing `PoseArray` return contract.
