## Context

`WorldMonitor.add_robot()` currently performs two jobs: it adds the robot to the planning world, then probes the attached visualization with `getattr(..., "register_robot", None)` so Viser can learn robot metadata. That hook exists because Viser is external to `DrakeWorld`; unlike Meshcat, it does not automatically inherit the Drake plant/scene graph when `WorldSpec.add_robot()` runs.

The same class of problem can appear for other external collaborators. A stateful external IK, planner, or visualization backend may need a consistent view of the initialized planning scene before it can serve requests. The right abstraction is not Viser-specific robot registration during `add_robot()`, but a startup planning-scene synchronization lifecycle.

## Goals / Non-Goals

**Goals:**

- Introduce a backend-neutral planning-scene metadata object that initially carries robot IDs and `RobotModelConfig` values.
- Add a startup scene sync method for visualization backends.
- Move Viser robot bootstrap from `WorldMonitor.add_robot()` into the startup sync flow.
- Keep `WorldMonitor.add_robot()` focused on world state, not optional visualizer capabilities.
- Preserve Meshcat behavior by allowing embedded world visualizers to no-op startup sync.
- Leave room to reuse the same planning-scene snapshot for future external IK/planner lifecycle methods.

**Non-Goals:**

- Do not implement IK or planner scene synchronization in this change.
- Do not move Meshcat out of `DrakeWorld`.
- Do not expose Viser GUI/session/widget concepts in `VisualizationSpec`.
- Do not add obstacle synchronization beyond what is needed for robot-scene bootstrap; obstacle support can be added later when a real external backend needs it.

## Decisions

### 1. Add a neutral `PlanningSceneInfo` model

Add a small frozen dataclass near the manipulation planning protocols/models layer:

```python
@dataclass(frozen=True)
class PlanningSceneInfo:
    robots: Mapping[WorldRobotID, RobotModelConfig]
```

The snapshot is intentionally minimal. It should describe stable planning-scene metadata, not runtime GUI state, not backend handles, and not mutable Drake contexts.

Alternative considered: pass `WorldMonitor` directly to all backends and let them query whatever they need. That already exists for Viser, but it hides initialization order and encourages backend-specific probing from core code.

### 2. Add `initialize_scene(scene: PlanningSceneInfo)` to `VisualizationSpec`

Visualization backends get one startup sync after robots are added and the world is finalized. Backends may render, cache metadata, or no-op.

For Viser, this replaces `register_robot(robot_id, config)` and loads/prepares:

- current robot URDFs,
- persistent target ghosts,
- transient preview ghosts,
- joint-name mappings,
- transform controls when the panel is enabled.

For Meshcat, this can no-op because `DrakeWorld.add_robot()` already configured the Drake plant/scene graph Meshcat observes.

Alternative considered: keep optional `register_robot()` as an ad-hoc method. That avoids protocol churn but keeps backend-specific lifecycle probing in `WorldMonitor.add_robot()` and does not scale to multi-robot/full-scene sync.

### 3. Store robot configs in `WorldMonitor` and sync explicitly

`WorldMonitor` should maintain a robot config registry alongside `_robot_joints`. The flow becomes:

```text
WorldMonitor.add_robot(config)
  └── world.add_robot(config)
      store robot_id -> config

ManipulationModule._initialize_planning()
  ├── add all robots
  ├── finalize world
  ├── add static startup obstacles such as floor
  ├── attach visualization
  ├── sync_visualization_scene()
  └── start visualization thread
```

`sync_visualization_scene()` builds `PlanningSceneInfo(robots=...)` and calls `visualization.initialize_scene(scene)` if visualization exists.

### 4. Keep collaborator sync extensible but demand-driven

The `PlanningSceneInfo` type should be reusable later for external IK/planner setup, but this change only wires visualization because that is the immediate Viser cleanup.

If a future external IK solver needs persistent metadata, add a separate lifecycle method to `KinematicsSpec` or a companion optional protocol, using the same `PlanningSceneInfo`. Do not make every stateless IK implementation pay complexity now.

## Risks / Trade-offs

- **Risk: Protocol method forces no-op implementations.** → Mitigation: the method has semantic meaning for all visualization backends and can be a no-op for embedded renderers.
- **Risk: Scene snapshot becomes a broad DTO.** → Mitigation: start with robots only; add obstacles/frames later only when a real backend requires them.
- **Risk: Initialization order changes Viser startup behavior.** → Mitigation: sync after robot addition/finalization and before visualization thread start; keep focused tests for multi-robot robot-registration and current-state rendering.
- **Risk: Future IK/planner sync diverges.** → Mitigation: keep `PlanningSceneInfo` in a neutral planning spec/model location and document it as reusable collaborator bootstrap metadata.

## Migration Plan

1. Add `PlanningSceneInfo` and `VisualizationSpec.initialize_scene()`.
2. Update all `VisualizationSpec` implementations/fakes with no-op or real initialization.
3. Add `WorldMonitor.sync_visualization_scene()` and remove `getattr(..., "register_robot")` from `add_robot()`.
4. Call scene sync from `ManipulationModule._initialize_planning()` after setup and before the visualization thread starts.
5. Replace Viser `register_robot()` with `initialize_scene()`.
6. Update tests and documentation.

Rollback is straightforward: the previous Viser `register_robot()` path can be restored if startup sync breaks, but the goal is to remove that ad-hoc hook entirely.

## Open Questions

- Should `PlanningSceneInfo` include startup static obstacles now, or wait until a renderer/planner requires them? Recommendation: wait.
- Should future IK/planner sync be required protocol surface or optional companion protocols? Recommendation: optional/demand-driven.
