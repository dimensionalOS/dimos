## 1. Scene Metadata Contract

- [x] 1.1 Add a neutral `PlanningSceneInfo` dataclass containing `robots: Mapping[WorldRobotID, RobotModelConfig]`.
- [x] 1.2 Add `initialize_scene(scene: PlanningSceneInfo) -> None` to `VisualizationSpec` with neutral documentation.
- [x] 1.3 Update existing `VisualizationSpec` fakes and embedded world visualizers with no-op scene initialization.

## 2. WorldMonitor Startup Sync

- [x] 2.1 Store robot configs in `WorldMonitor` when robots are added.
- [x] 2.2 Remove the dynamic `register_robot` visualization probe from `WorldMonitor.add_robot()`.
- [x] 2.3 Add `WorldMonitor.planning_scene_info()` or equivalent snapshot helper.
- [x] 2.4 Add `WorldMonitor.sync_visualization_scene()` that sends the snapshot to the attached visualization when present.

## 3. ManipulationModule Lifecycle

- [x] 3.1 Call `sync_visualization_scene()` during `_initialize_planning()` after robot/world setup and before starting the visualization thread.
- [x] 3.2 Preserve Meshcat default behavior for `enable_viz=True` with no explicit backend.
- [x] 3.3 Preserve Viser startup behavior for current, target, preview, and panel controls.

## 4. Viser Backend Migration

- [x] 4.1 Replace `ViserManipulationVisualizer.register_robot()` with `initialize_scene()`.
- [x] 4.2 Ensure Viser initializes all robots from the scene snapshot, including multi-robot configurations.
- [x] 4.3 Keep target ghost, preview ghost, current robot, joint mapping, and pose selector behavior unchanged after sync.

## 5. Tests and Docs

- [x] 5.1 Add unit tests that `WorldMonitor.add_robot()` no longer probes visualization-specific hooks.
- [x] 5.2 Add unit tests that startup scene sync delegates `PlanningSceneInfo` to visualization.
- [x] 5.3 Add Viser tests for `initialize_scene()` replacing robot registration.
- [x] 5.4 Add or update docs describing startup scene sync and `visualization_options` usage where relevant.
- [x] 5.5 Run focused manipulation visualization tests.
- [x] 5.6 Run ruff, mypy, doclinks if docs changed, and `openspec validate add-planning-scene-sync`.
