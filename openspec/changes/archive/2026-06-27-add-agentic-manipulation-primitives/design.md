## Context

DimOS already has a `ManipulationModule` that integrates robot model configuration, planning, coordinator dispatch, joint state monitoring, and primitive skills such as robot state queries, joint motion, and gripper commands. The existing `PickAndPlaceModule` extends that lower-level stack with perception/object semantics, but the next agentic manipulation step needs a smaller universal primitive surface that can be exercised before committing to higher-level pick/place APIs.

Robosuite integration currently validates runtime plumbing through a script-hosted sidecar demo. That demo starts Robosuite in a subprocess, resolves a runtime plan, creates a SHM motor bridge, instantiates a `ControlCoordinator`, and drives a servo task directly. It does not build the manipulation planning stack or call a skill-facing module API. The new validation should keep the same script-hosted runtime boundary while adding the full manipulation stack above the hardware adapter.

## Goals / Non-Goals

**Goals:**

- Introduce a universal `AgenticManipulationModule` in `dimos/manipulation/` that exposes a small, agent-appropriate primitive skill surface.
- Implement the module as a facade over existing manipulation RPCs using the DimOS Spec injection pattern.
- Keep the initial primitive surface limited to joint-space and gripper operations: robot state, move to joints, open gripper, and close gripper.
- Unit-test the facade without Robosuite, benchmark runtime clients, sidecars, or heavy simulator dependencies.
- Add a script-hosted Robosuite layer-2 demo that constructs the full stack dynamically and calls the new API through DimOS modules.

**Non-Goals:**

- Running an LLM agent or creating a final MCP tool profile.
- Filtering MCP skill exposure when both `ManipulationModule` and `AgenticManipulationModule` are present.
- Making Robosuite or runtime sidecar APIs part of the universal module.
- Requiring Cartesian planning or relative end-effector motion in this slice.
- Promoting the Robosuite validation script to a product CLI or registered blueprint.

## Decisions

### Agentic facade over subclassing

Create `AgenticManipulationModule` as a `Module` that depends on a `ManipulationControlSpec` Protocol instead of subclassing `ManipulationModule`.

Rationale: the agent-facing module should remain a stable facade over whatever manipulation/control stack is present. Subclassing would couple the new surface to `ManipulationModule` internals and make it harder to later swap or compose other manipulation implementations. Spec injection also matches DimOS blueprint wiring and fails at build time if a compatible manipulation provider is absent.

Alternative considered: subclass `ManipulationModule` and redecorate selected methods. This is simpler initially but exposes internals, duplicates existing skill registration concerns, and makes it harder to keep the universal surface independent of planner implementation details.

### Thin primitive wrapper first

The first module methods should delegate to existing manipulation RPCs and return their `SkillResult` values rather than inventing a new result model.

Rationale: this preserves existing error semantics and avoids premature API design before Robosuite and real robot experiments show which higher-level skills are appropriate. The facade can still improve docstrings and names for agent use.

Alternative considered: define new structured result objects for every primitive. This could be useful later, but it increases integration work before the primitive behavior is validated.

### Unit tests stay simulator-free

Unit tests should inject a fake manipulation provider and verify delegation, signatures, docstrings, and result passthrough.

Rationale: Robosuite is a heavy optional dependency and should not be required by default `pytest`. The universal module has no simulator-specific behavior, so simulator-free tests are the right layer for correctness.

Alternative considered: include a Robosuite smoke test in pytest behind a marker immediately. This adds maintenance and environment cost before the API shape is proven.

### Robosuite validation remains script-hosted

Add `scripts/benchmarks/demo_agentic_manipulation_robosuite.py` rather than a registered blueprint or product CLI.

Rationale: the demo needs a pre-blueprint construction stage: start the sidecar, describe/reset the episode, resolve the motor surface, build a dynamic `RobotConfig`, derive blueprint inputs, then build the DimOS stack. The existing script-hosted runtime demo already establishes this orchestration shape.

Alternative considered: add a static blueprint and a separate RPC client. Static blueprint registration cannot know the runtime motor surface and SHM address before sidecar startup.

### Dynamic RobotConfig bridge

The Robosuite demo should convert resolved runtime information into a `RobotConfig`, then use `RobotConfig` methods to derive `HardwareComponent`, `TaskConfig`, and `RobotModelConfig`.

Rationale: `RobotConfig` is the existing pre-blueprint robot configuration abstraction that spans hardware, coordinator task, and manipulation planning model construction. Generating only `RobotModelConfig` would leave the coordinator and hardware adapter configured through a separate path.

Alternative considered: directly instantiate all three derived objects in the demo. That is acceptable as a fallback for missing fields, but the intended implementation should centralize derivation through `RobotConfig` wherever current APIs allow.

### Background runtime stepping for blocking API calls

The Robosuite demo should run the SHM-to-sidecar stepping loop in a background thread while the foreground calls `AgenticManipulationModule` APIs.

Rationale: motion APIs may block while planning, dispatching, and waiting for coordinator completion. The runtime sidecar still needs regular steps to convert coordinator commands into simulator state and write motor state back into SHM.

Alternative considered: interleave one API call with manual step batches in the foreground. That does not match blocking skill execution and risks deadlock when a motion call waits for state progression.

## Risks / Trade-offs

- [Risk] The current Robosuite Panda Lift profile may not provide all model metadata needed to construct a robust `RobotConfig`. → Mitigation: keep direct object construction as a narrow script-local fallback and document missing model assumptions in the demo artifact summary.
- [Risk] Existing `ManipulationModule.move_to_joints` plans and executes through a trajectory path, while the current Robosuite demo uses a servo task. → Mitigation: configure a trajectory-compatible coordinator task for the layer-2 demo instead of reusing the direct servo-only loop.
- [Risk] Both old and new manipulation skills would be visible if MCP is added to the same stack. → Mitigation: keep MCP/agent execution out of this change; handle skill filtering in a later change.
- [Risk] Robosuite validation may be unavailable in default developer environments. → Mitigation: keep it manual/script-only and fail with a clear message when the sidecar cannot start.
- [Risk] The facade may be too thin to add much value initially. → Mitigation: treat this as a stable primitive seam for experiments; evolve higher-level semantics only after Robosuite and real stack validation.

## Migration Plan

1. Add the universal module and tests without changing existing manipulation module behavior.
2. Add the Robosuite demo as an opt-in script that does not affect default test runs or blueprint registry generation.
3. Document the demo command and scope boundaries.
4. Rollback is removal of the new module, tests, docs, and script; no existing API behavior changes are required.

## Open Questions

- Which higher-level semantic manipulation skills should be exposed after joint/gripper primitives are validated?
- What MCP filtering mechanism should hide lower-level provider skills when an agent-facing facade is present?
- What additional runtime metadata should Robosuite sidecars expose to make `RobotConfig` derivation less demo-specific?
