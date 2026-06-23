## 1. Composition Infrastructure

- [x] 1.1 Add a narrow reference-transform interface for control tasks that can consume an upstream `JointCommandOutput` and return a transformed `JointCommandOutput | None`.
- [x] 1.2 Add a composed control task wrapper that exposes normal coordinator-facing task semantics while internally running one source task followed by ordered transforms.
- [x] 1.3 Validate composed task construction for v1 constraints: one source, linear transforms, compatible joints, and compatible command modes.
- [x] 1.4 Forward lifecycle/preemption behavior from the composed task to the source and transforms, including safe reset of transform state on preemption.

## 2. Joint Compliance Behavior

- [x] 2.1 Add a joint compliance task/configuration with per-joint mass, damping, stiffness, max offset, max offset velocity, deadband/tolerance, and effort-feedback options.
- [x] 2.2 Implement reference-transform behavior for `SERVO_POSITION` joint position references.
- [x] 2.3 Implement bounded admittance-style offset integration using coordinator joint position state, optional velocity state, and explicitly enabled effort feedback.
- [x] 2.4 Define and implement safe behavior for invalid `dt`, missing required joint state, incompatible reference modes, and clamp saturation.
- [x] 2.5 Expose diagnostics or inspectable state for offset, offset velocity, saturation, and feedback mode so tests can verify behavior.

## 3. Compliant Trajectory Integration

- [x] 3.1 Add a compliant joint trajectory task/factory that composes the existing joint trajectory source with the joint compliance transform.
- [x] 3.2 Keep existing rigid joint trajectory behavior unchanged and selectable.
- [x] 3.3 Add optional manipulator task configuration or blueprint helper support for selecting compliant trajectory behavior where appropriate.
- [x] 3.4 If any blueprint variables are added or renamed, regenerate and verify the blueprint registry with `pytest dimos/robot/test_all_blueprints_generation.py`.

## 4. Tests

- [x] 4.1 Add unit tests for composed task output forwarding, transform ordering, validation failures, active-state behavior, and preemption forwarding.
- [x] 4.2 Add unit tests for compliance pass-through behavior in free space.
- [x] 4.3 Add unit tests for bounded offset and offset velocity under tracking resistance.
- [x] 4.4 Add unit tests proving effort feedback is opt-in and zero-valued placeholder effort is not treated as reliable by default.
- [x] 4.5 Add unit tests for invalid `dt`, missing joint state, incompatible reference modes, and saturation diagnostics.
- [x] 4.6 Add integration tests for compliant trajectory sampling followed by compliant output shaping, including completion/cancel/preemption parity with rigid trajectory execution.

## 5. MuJoCo Verification

- [x] 5.1 Identify the canonical MuJoCo manipulator verification entry point, preferably existing xArm7 simulation unless a smaller dedicated scene is warranted.
- [x] 5.2 Add a free-space MuJoCo smoke test or demo that compares rigid and compliant trajectory execution with near-zero compliant offset.
- [x] 5.3 Add or document a rigid table/obstacle contact scenario that compares rigid and compliant trajectory behavior using joint-space metrics such as peak effort, tracking error, offset, and saturation time.
- [x] 5.4 Optionally add a soft cushion/contact demo after rigid contact behavior is stable; do not make soft-contact tuning the only proof of correctness.
- [x] 5.5 Document manual simulation QA commands and expected pass/fail observations.

## 6. Documentation

- [x] 6.1 Add or update user-facing documentation explaining rigid vs compliant trajectory behavior, v1 position-servo semantics, feedback requirements, and safety bounds.
- [x] 6.2 Add contributor documentation or development notes explaining that composed tasks are internal linear pipelines, not coordinator-level task graphs.
- [x] 6.3 Document MuJoCo smoke/contact verification if a simulation blueprint, scene, or demo is added.
- [x] 6.4 Confirm whether `docs/coding-agents/` or `AGENTS.md` need updates; update only if new coding-agent guidance is introduced.

## 7. Validation

- [x] 7.1 Run `openspec validate add-compliant-joint-trajectory-task`.
- [x] 7.2 Run focused pytest targets for changed control task and compliance task code.
- [x] 7.3 Run focused MuJoCo tests or demos for simulation verification when available; use `bin/pytest-mujoco` if the test is marked for MuJoCo.
- [x] 7.4 Run `pytest dimos/robot/test_all_blueprints_generation.py` if blueprint registry inputs change.
- [x] 7.5 Run relevant docs validation commands for changed docs, such as `doclinks`, `md-babel-py run <doc>`, or `bin/gen-diagrams` when applicable.
- [x] 7.6 Run type/lint checks required by the touched area, such as `uv run mypy dimos/` or focused lint/test commands if full checks are too expensive.
