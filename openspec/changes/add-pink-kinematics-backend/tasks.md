## 1. Implementation

- [x] 1.1 Add Pink dependency metadata or documented install support, keeping Pink optional for non-Pink backends. Prefer an optional extra such as `pink` that installs `pin-pink` and one supported `qpsolvers` backend; ensure `uv sync --extra all --extra pink` or the final documented equivalent works.
- [x] 1.2 Add `dimos/manipulation/planning/kinematics/pink_ik.py` with a `PinkIK` backend that satisfies the existing `KinematicsSpec.solve(...)` contract and returns `IKResult` values for success, convergence failure, dependency failure, solver failure, joint-limit failure, and collision rejection.
- [x] 1.3 Localize Pink/Pinocchio/QP imports to the Pink backend or a small adapter layer so `jacobian` and `drake_optimization` remain importable when Pink dependencies are absent.
- [x] 1.4 Build a per-robot Pink context from `RobotModelConfig`: load/cache the Pinocchio model/data, validate `end_effector_link`, derive name-based mappings between DimOS `joint_names` and Pinocchio/Pink joint order, and initialize configuration from the seed or current world joint state.
- [x] 1.5 Implement the iterative Pink solve loop: set the end-effector frame target, optionally add a posture task biased to the seed, call `pink.solve_ik(...)`, integrate the returned velocity, compute position/orientation errors, and stop on tolerance or configured iteration/attempt limits.
- [x] 1.6 Preserve planning safety semantics by applying joint limits and, when `check_collision=True`, calling `world.check_config_collision_free(robot_id, candidate_joint_state)` before returning a successful IK result.
- [x] 1.7 Extend `create_kinematics(name=...)` to accept `"pink"`, update invalid-backend messaging if needed, and update `ManipulationModuleConfig.kinematics_name` comments/docs to include `"pink"`.
- [x] 1.8 Use the existing simulation-only xArm manual QA blueprint and select Pink with CLI config overrides, for example `xarm-perception-sim -o pickandplacemodule.kinematics_name=pink`.
- [x] 1.9 Do not add a dedicated Pink QA blueprint or public registry entry; validate the generated blueprint registry remains current after reverting the temporary blueprint variant.
- [x] 1.10 Add an IK-only `ManipulationModule.solve_ik(...)` RPC and `ik_pose(...)` client helper so manual QA can test the configured IK backend without invoking path planning.
- [x] 1.11 Add an optional IK seed joint state to `solve_ik(...)` and `ik_pose(..., seed_joints=...)` so local IK backends can initialize from a caller-provided joint configuration.

## 2. Tests

- [x] 2.1 Add factory tests proving `create_kinematics("pink")` returns the Pink backend when dependencies are available or raises an actionable dependency error when they are not.
- [x] 2.2 Add tests proving existing `jacobian` and `drake_optimization` factory selection still works without importing Pink at module import time.
- [x] 2.3 Add Pink backend unit tests with a minimal/fake `WorldSpec` or lightweight robot fixture that verify successful `IKResult` shaping, non-convergence reporting, collision rejection, and no successful result when frame/joint mapping fails.
- [x] 2.4 Add joint-order mapping tests using non-trivial joint name order so DimOS `JointState` positions map correctly to and from the Pinocchio/Pink configuration vector.
- [x] 2.5 Add or update manipulation module tests to confirm `kinematics_name="pink"` is wired from module config into the planning stack.

## 3. Documentation

- [x] 3.1 Update user-facing manipulation/planning docs to list `pink` as a selectable backend, describe it as local differential IK, and document dependency/QP solver requirements.
- [x] 3.2 Document the simulation-safe Pink QA command sequence using the existing xArm simulation blueprint plus CLI override:

  ```bash
  uv sync --extra all --extra pink
  uv run dimos --simulation run xarm-perception-sim \
    -o pickandplacemodule.kinematics_name=pink
  uv run python -i -m dimos.manipulation.planning.examples.manipulation_client
  ```

  Then in the client:

  ```python
  robots()
  joints()
  ik_pose(0.45, 0.0, 0.25)
  ik_pose(0.45, 0.0, 0.25, seed_joints=[0.0] * 7)
  plan_pose(0.45, 0.0, 0.25)
  preview()
  ```

- [x] 3.3 Document the baseline comparison command:

  ```bash
  uv run dimos --simulation run xarm-perception-sim
  ```

  Repeat the same `manipulation_client` calls and compare the planned/previewed path.

- [x] 3.4 Update contributor/testing docs only if implementation introduces new test commands, optional extras, or blueprint-regeneration steps not already documented.

## 4. Verification

- [x] 4.1 Run `openspec validate add-pink-kinematics-backend`.
- [x] 4.2 Run focused manipulation tests:

  ```bash
  uv run pytest dimos/manipulation/test_manipulation_unit.py dimos/manipulation/test_manipulation_module.py
  ```

- [x] 4.3 Run the new Pink backend tests added for this change.
- [x] 4.4 Since no public Pink blueprint remains, run registry validation to confirm `dimos/robot/all_blueprints.py` is current:

  ```bash
  uv run pytest dimos/robot/test_all_blueprints_generation.py
  ```

- [x] 4.5 Run type/lint checks for changed manipulation planning files, at minimum the repository's standard targeted mypy/ruff commands for the touched paths.
- [x] 4.6 Run docs validation for any changed docs using the repository's doc tooling where applicable.

## 5. Manual QA

- [ ] 5.1 Install the Pink-enabled environment using the final supported command. Planned command if a `pink` optional extra is added:

  ```bash
  uv sync --extra all --extra pink
  ```

- [ ] 5.2 Start the existing simulation-only xArm QA blueprint with Pink selected in terminal 1:

  ```bash
  uv run dimos --simulation run xarm-perception-sim \
    -o pickandplacemodule.kinematics_name=pink
  ```

- [ ] 5.3 In terminal 2, open the manipulation client and verify the robot state:

  ```bash
  uv run python -i -m dimos.manipulation.planning.examples.manipulation_client
  ```

  ```python
  robots()
  joints()
  ee()
  ```

- [ ] 5.4 Solve IK directly, then plan to a conservative reachable target with Pink and preview without executing hardware:

  ```python
  ik_pose(0.45, 0.0, 0.25)
  ik_pose(0.45, 0.0, 0.25, seed_joints=[0.0] * 7)
  plan_pose(0.45, 0.0, 0.25)
  preview()
  ```

- [ ] 5.5 Stop the Pink override run, start the baseline Jacobian simulation blueprint, and repeat the same target:

  ```bash
  uv run dimos --simulation run xarm-perception-sim
  ```

  ```python
  ik_pose(0.45, 0.0, 0.25)
  ik_pose(0.45, 0.0, 0.25, seed_joints=[0.0] * 7)
  plan_pose(0.45, 0.0, 0.25)
  preview()
  ```

- [ ] 5.6 Record whether Pink succeeds, whether the previewed path is collision-free, and how behavior differs from the Jacobian baseline. Do not run `execute()` on hardware as part of this QA path.
