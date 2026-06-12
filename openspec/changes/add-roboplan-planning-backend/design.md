## Context

DimOS manipulation planning already separates world, planner, and kinematics behavior through Python Protocol contracts. In practice, world construction is currently Drake-only, planner construction supports the generic RRT planner, and manipulation visualization is controlled by a Drake/Meshcat-oriented `enable_viz` boolean rather than a backend selector. The existing generic RRT planner is mostly backend-agnostic: it needs finalized-world state, robot IDs, joint limits, configuration collision checks, and edge collision checks.

RoboPlan should be introduced as an optional manipulation planning backend without replacing the Drake default. RoboPlan's native planner APIs are scene-centric and need access to RoboPlan world internals, so the native RoboPlan planner path should be allowed to use the RoboPlan world object directly rather than forcing all internals through the public world Protocol.

## Goals / Non-Goals

**Goals:**

- Add explicit manipulation configuration for `world_backend="roboplan"` while preserving `world_backend="drake"` as the default.
- Make `world_backend="roboplan"` work with the existing generic `planner_name="rrt_connect"` path as the first acceptance target.
- Add `planner_name="roboplan"` as a RoboPlan-native planner option that is valid only with the RoboPlan world backend.
- Keep `world_backend` and `planner_name` independent and explicit; choosing one must not silently switch the other.
- Implement straightforward world methods where RoboPlan has direct equivalents, and fail clearly for obscure or safety-critical unsupported features.

**Non-Goals:**

- Do not make RoboPlan part of the default manipulation extra.
- Do not support Drake world plus RoboPlan-native planner in this change.
- Do not silently approximate unsupported collision geometry, joint semantics, or obstacle behavior.
- Do not require full min-distance/FK/Jacobian parity before the generic planner path works.
- Do not add new manipulation blueprints unless implementation finds that an opt-in blueprint is necessary.

## DimOS Architecture

- `ManipulationModuleConfig` should expose `world_backend: str = "drake"` alongside existing planner, kinematics, and visualization options. The module should pass this value into `WorldMonitor`/world construction.
- `WorldMonitor` should accept and forward the selected world backend while continuing to wrap a `WorldSpec` world. Normal monitor operations should stay protocol-driven.
- `create_world()` should support `backend="drake"` and `backend="roboplan"`. RoboPlan imports must be lazy so environments without the optional extra keep working.
- `create_planner()` should continue to return standalone planners for generic algorithms. For `planner_name="roboplan"`, the manipulation startup path should select the RoboPlan world object as the planner when the world backend is RoboPlan.
- Startup validation should reject invalid combinations before any plan request:
  - `planner_name="roboplan"` requires `world_backend="roboplan"`.
  - Drake-specific kinematics requires `world_backend="drake"`.
  - Unknown backend/planner/kinematics names fail fast with actionable errors.
- A new RoboPlan world implementation should implement the `WorldSpec` methods required by runtime wiring and generic RRT planning first: robot registration, finalization, robot ID/config/limit queries, context management, joint state management, obstacle registry, collision-free configuration checks, and collision-free edge checks.
- The RoboPlan world may also implement `PlannerSpec` for native planning. In native mode, `world = RoboPlanWorld(...)` and `planner = world`; in generic mode, `world = RoboPlanWorld(...)` and `planner = RRTConnectPlanner(...)`.
- No new streams, transports, skills, or MCP tools are expected. This change is within manipulation module configuration and planning backend internals.
- No generated blueprint registry update is expected unless new blueprints are added. If implementation adds or renames blueprints, run `pytest dimos/robot/test_all_blueprints_generation.py`.

## Decisions

- **Use a separate optional RoboPlan extra.** RoboPlan is compiled, multi-package, and still evolving. Keeping it optional avoids breaking default manipulation installs.
- **Use Git-sourced dependencies for now, not commit-pinned.** PyPI packaging is treated as a separate job, and RoboPlan is in active development. This favors development velocity over lockfile stability for the first integration.
- **Keep backend selection explicit.** `planner_name="roboplan"` must not imply `world_backend="roboplan"`; invalid combinations fail with clear errors.
- **Make RoboPlan native planning backend-coupled.** RoboPlan RRT is scene-centric and lacks evidence of arbitrary world callback hooks, so it should require a RoboPlan world rather than pretending to support generic `WorldSpec` worlds.
- **Generic planner compatibility is the first acceptance target.** `world_backend="roboplan"` with `planner_name="rrt_connect"` proves the world abstraction works with another backend.
- **Implement easy direct mappings opportunistically.** FK/link pose/Jacobian can be implemented if RoboPlan exposes straightforward APIs, but generic planning must not be blocked by obscure query-method parity.
- **No silent degradation.** Unsupported planning-critical inputs raise `ValueError`; optional query methods without safe semantics raise `NotImplementedError`.

## Safety / Simulation / Replay

The RoboPlan backend affects robot motion planning, so unsupported collision geometry, robot joint types, missing limits, or model features must fail before planning. The implementation must not ignore obstacles or approximate unsupported features without an explicit safe mapping.

Real hardware use should remain opt-in through configuration. Initial validation should happen in tests, replay/simulation where available, and manually with visualization or dry-run planning before using the backend on hardware. Simulation/replay behavior should remain identical for Drake-backed stacks unless `world_backend="roboplan"` is selected.

## Risks / Trade-offs

- RoboPlan Git dependencies may change shape during development. Lazy imports and explicit install hints mitigate non-RoboPlan environments, but reproducibility is weaker until dependencies are pinned or published.
- RoboPlan collision context semantics require care: collision scratch state should be per-context, and geometry updates should invalidate or rebuild collision contexts.
- Joint ordering, mimic/floating/continuous joints, and model-package resolution can diverge between Drake and RoboPlan. The backend must validate and record mappings explicitly.
- Native RoboPlan planning cannot be used with Drake worlds without a translator or additional backend-handle abstraction. This is intentionally out of scope.
- `get_min_distance` semantics are hard because DimOS expects signed minimum distance behavior; it can remain explicitly unimplemented until verified.

## Migration / Rollout

- Existing users keep the Drake default without changing configuration.
- Developers opt in with the RoboPlan extra and explicit manipulation module options such as `world_backend=roboplan` and `planner_name=rrt_connect` or `planner_name=roboplan`.
- Add or update manipulation planning docs to describe backend selection, dependency installation, valid combinations, and unsupported-method policy.
- Add tests for factory/config validation, lazy import failure messages, generic RRT compatibility, unsupported planning-critical inputs, and RoboPlan-native planner wiring if implemented.
- Rollback is disabling the RoboPlan extra/config and returning to `world_backend="drake"`.

## Open Questions

- Which exact RoboPlan Git dependency strings and subdirectory names work in this repository's `uv` environment during implementation?
- Which RoboPlan APIs are available in the installed Python bindings for FK, Jacobian, obstacles, and native RRT options?
- Which real robot model should serve as the first manual QA target for the RoboPlan backend?
