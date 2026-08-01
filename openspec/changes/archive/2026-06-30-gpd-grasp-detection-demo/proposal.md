## Why

DimOS now has project-runtime workers that can host native Python bindings, but the GPD demo only proves that `gpd.core` can be imported. The grasping stack still lacks a runnable command that starts the xArm MuJoCo simulation, uses existing perception modules to produce pointcloud data, runs GPD grasp detection from that pointcloud, and shows the result in Rerun like the current VGN demo.

## What Changes

- Add an import-safe GPD grasp demo package under `packages/` with a pointcloud-consuming grasp detector Module that runs inside a Python project runtime.
- Wire the GPD generator into the existing `GraspGenSpec` pointcloud path so `GraspingModule.generate_grasps(...)` can call it without changing the user-facing skill contract.
- Publish compatible `PoseArray` outputs and preserve richer candidate/debug data where available without requiring a new core grasp API in this slice.
- Add a VGN-like xArm MuJoCo demo blueprint that routes a registered grasp target through object registration, `GraspingModule`, and the GPD project-runtime worker.
- Provide a documented end-to-end demo command that runs simulation, pointcloud/object perception, GPD grasp detection, and Rerun visualization.
- Add deterministic sample/synthetic pointcloud tests that validate conversion and output behavior independently from MuJoCo/perception timing.
- Do not execute robot pick/place motions in this change; the demo stops at candidate generation and visualization/logging.

## Capabilities

### New Capabilities
- `gpd-grasp-detection`: GPD grasp detection from pointcloud inputs and demo workflow requirements.

### Modified Capabilities
- `venv-module-packaging`: Add a GPD grasp demo package requirement that goes beyond import probing and exercises grasp detection from pointcloud inputs.
- `venv-module-placement`: Add a project-runtime placement scenario for a real GPD grasp generator Module in an xArm MuJoCo workflow.

## Impact

- New package under `packages/` for the GPD grasp demo Module, blueprint factory, and project-runtime manifests.
- New grasping Module tests for pointcloud conversion, lazy GPD imports, empty-result handling, and published outputs.
- New xArm MuJoCo demo blueprint and blueprint-level tests modeled after the existing VGN demo.
- Documentation for preparing and running the end-to-end GPD grasp detection demo command with Rerun visualization.
- No breaking changes to `GraspingModule`, `GraspGenSpec`, VGN, or existing xArm blueprints.
