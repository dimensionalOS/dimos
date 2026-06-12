## Context

The manipulation planning stack already routes Cartesian planning through a backend-neutral kinematics interface. `ManipulationModule.plan_to_pose()` converts a requested pose into a world-frame target, calls the configured kinematics backend, receives an `IKResult`, and then plans a joint path to the solved joint state. The active planning abstraction is `KinematicsSpec`; there is no separate planning-level `IKSpec` symbol today.

The current factory supports `jacobian` and `drake_optimization`. A custom Pinocchio IK solver exists under planning kinematics, but it has a standalone `solve(target_pose, q_init, ...)` shape and is used directly by control/teleop paths rather than by the manipulation planning factory. Pink should be introduced as a new planning backend parallel to the existing backends, not by replacing the custom Pinocchio control solver.

Pink is a Pinocchio-based differential IK library. It solves task-based QPs over a `pink.Configuration`, returns tangent-space velocity, and expects the caller to iterate by integrating that velocity until task error converges or a limit is reached. Its package install name is `pin-pink`, while the Python import is `pink`. It also depends on a usable `qpsolvers` backend.

## Goals / Non-Goals

**Goals:**

- Add a Pink kinematics backend that satisfies the existing manipulation planning `KinematicsSpec` contract.
- Make the backend selectable with `kinematics_name="pink"` through the planning factory and module config.
- Preserve existing Jacobian, Drake optimization, and customized Pinocchio control behavior.
- Provide clear missing-dependency and unavailable-solver errors only when the Pink backend is requested.
- Preserve existing collision semantics by delegating final candidate collision validation to the configured world.
- Provide a simulation-only manual QA command sequence that selects Pink on an existing blueprint, plans to a Cartesian target, and compares against the Jacobian backend.

**Non-Goals:**

- Do not rename `KinematicsSpec` or introduce a broad top-level IK abstraction unless a future change explicitly scopes that refactor.
- Do not migrate cartesian/teleop control tasks from the customized Pinocchio solver to Pink in this change.
- Do not make Pink a required dependency for users who only use existing kinematics backends.
- Do not claim global IK behavior; Pink remains a local differential IK solver.
- Do not execute hardware motions as part of the default QA path.

## DimOS Architecture

- Planning backend: add a new `PinkIK` implementation under `dimos/manipulation/planning/kinematics/` that implements the existing `KinematicsSpec.solve(...)` behavior.
- Factory: extend `dimos/manipulation/planning/factory.py:create_kinematics()` to accept `"pink"` and return the new backend. Unknown names should keep the existing invalid-backend behavior.
- Module config: update the `ManipulationModuleConfig.kinematics_name` comment/docs to include `"pink"`; no stream schema or RPC signature changes are required.
- World integration: use `WorldSpec.get_robot_config()`, `get_joint_limits()`, `get_joint_state()`, `get_ee_pose()`, and `check_config_collision_free()` to preserve backend-neutral behavior. The Pink backend may load/cache a Pinocchio model from `RobotModelConfig.model_path` and map between Pink/Pinocchio joint ordering and DimOS `JointState` ordering.
- Adapter boundary: keep Pink imports localized to the Pink backend module or a small adapter helper. Other backends must import and run without Pink installed.
- Blueprint/manual QA: reuse the existing simulation-safe xArm manipulation blueprint and select Pink with CLI `-o` config overrides rather than adding a dedicated Pink blueprint.
- CLI/manual command surface: the run command should be a normal blueprint invocation with an override, for example `dimos --simulation run xarm-perception-sim -o pickandplacemodule.kinematics_name=pink`, followed by a manipulation RPC client command that plans to a safe Cartesian target. For blueprints built from `ManipulationModule` directly, use `-o manipulationmodule.kinematics_name=pink`. No MCP skill changes are required.

## Decisions

1. **Implement Pink as a `KinematicsSpec` backend, not as a new `IKSpec`.**
   - Rationale: planning call sites already depend on `KinematicsSpec`, and `plan_to_pose()` already has the right solve shape.
   - Alternative considered: introduce a new `IKSpec` and refactor callers. That would add churn without changing the observable behavior needed for this backend.

2. **Leave the customized Pinocchio solver in place.**
   - Rationale: control/teleop tasks currently use it directly. Changing those paths would mix a planning backend addition with a control behavior migration.
   - Alternative considered: wrap or replace the custom Pinocchio solver. That should be a separate change after Pink is validated in planning.

3. **Treat Pink as optional at the adapter/backend boundary.**
   - Rationale: existing users of Jacobian and Drake backends should not fail imports because Pink or a QP solver is absent.
   - Implementation direction: import `pink`, `pinocchio`, and solver-specific dependencies lazily or behind a clear backend initialization path; raise an actionable error when `kinematics_name="pink"` is requested but dependencies are missing.

4. **Preserve collision checking outside Pink.**
   - Rationale: existing world collision behavior is the planning-stack source of truth. Pink can handle kinematic limits, but final collision semantics should remain consistent with other backends.
   - Behavior: if `check_collision=True`, only return success after the candidate joint state passes `world.check_config_collision_free(robot_id, candidate_joint_state)`.

5. **Use an iterative Pink solve loop.**
   - Rationale: Pink returns velocity/tangent displacement, not a one-shot global IK solution.
   - Implementation direction: initialize configuration from the seed/current joint state, set a frame target for the configured end-effector link, optionally include a posture task biased toward the seed, call `pink.solve_ik(...)`, integrate, and check pose error until tolerances or iteration limits are reached.

6. **Use CLI overrides for simulation QA instead of a dedicated Pink blueprint.**
   - Rationale: DimOS already supports per-module `-o` config overrides, so users can prove the new backend can be selected in a real module graph without expanding the public blueprint list.
   - Alternative considered: add a dedicated Pink blueprint variant. That adds registry surface area for a configuration variant that existing CLI overrides already cover.

## Safety / Simulation / Replay

Manual QA must run in simulation by default. The planned QA surface is the existing xArm simulation blueprint with Pink selected through CLI config override. The flow should not execute on physical hardware unless the operator explicitly chooses a hardware blueprint after validating simulation behavior.

The Pink backend must honor existing joint limits and return failure when convergence, limit, dependency, or solver issues prevent a safe IK result. When collision checking is requested, the backend must reject candidates that fail the world collision check. Error messages should state whether failure came from convergence, missing dependency, solver unavailability, frame/model mapping, joint limit, or collision rejection.

Manual QA command blueprint:

```bash
# Terminal 1: start the sim-safe manipulation blueprint with Pink selected
uv sync --extra all --extra pink
uv run dimos --simulation run xarm-perception-sim \
  -o pickandplacemodule.kinematics_name=pink

# Terminal 2: solve IK and plan to a conservative reachable target without executing hardware
uv run python -i -m dimos.manipulation.planning.examples.manipulation_client
>>> robots()
>>> joints()
>>> ik_pose(0.45, 0.0, 0.25)
>>> ik_pose(0.45, 0.0, 0.25, seed_joints=[0.0] * 7)
>>> plan_pose(0.45, 0.0, 0.25)
>>> preview()
```

The `ik_pose(...)` helper is intentionally a client-side convenience wrapper
around the `solve_ik(Pose, ...)` RPC. It accepts xyz/rpy arguments for interactive
use and can pass an optional seed joint configuration so local differential IK
starts near a known configuration instead of always reading current state.

Comparison command blueprint:

```bash
# Baseline with the existing Jacobian backend
uv run dimos --simulation run xarm-perception-sim
# Repeat the same manipulation_client plan_pose(...) and preview() calls.
```

If implementation chooses not to add a `pink` optional extra, replace `uv sync --extra all --extra pink` with the documented dependency install command used by the project.

## Risks / Trade-offs

- **Local minima:** Pink is differential/local. Mitigation: seed from current joints, allow bounded random or named restarts if needed, and report non-convergence clearly.
- **Joint ordering mismatch:** Pinocchio/Pink model order may differ from DimOS `RobotModelConfig.joint_names`. Mitigation: build explicit name-based maps and test them.
- **Frame mismatch:** Pink `FrameTask` uses Pinocchio frame names. Mitigation: validate `end_effector_link` exists during backend context creation and include available-frame hints in errors when practical.
- **World/base transform mismatch:** The planning target is world-framed while the Pink model may be robot-base-framed. Mitigation: define and test how the backend converts the target into the model frame, starting with fixed-base xArm simulation.
- **QP solver availability:** Pink delegates to `qpsolvers`. Mitigation: document the required optional dependency/solver, choose a default solver with a clear fallback policy, and fail actionably if none is installed.
- **Dependency footprint:** Making Pink required would affect all users. Mitigation: keep the dependency optional or guarded until maintainers decide to promote it.

## Migration / Rollout

- Add the backend and tests while leaving the default `kinematics_name` as `"jacobian"`.
- Add optional dependency metadata or documentation for `pin-pink` and the selected QP solver package.
- Update manipulation planning docs to list `"pink"`, dependency requirements, limitations, and the simulation QA command using CLI overrides.
- Rollback is straightforward: remove `"pink"` factory dispatch while leaving existing backends and blueprints untouched.

## Open Questions

- Which QP solver should be the default in DimOS environments: `proxqp`, `quadprog`, or a solver selected from `qpsolvers.available_solvers`?
- Should Pink be added as a new optional dependency extra such as `pink`, included in `all`, or documented as a manual install first?
- Should the first implementation support only fixed-base manipulators, or attempt floating-base models immediately?
- Should the backend include random restart behavior in v1, or keep v1 deterministic and rely on the caller-provided seed/current state?
