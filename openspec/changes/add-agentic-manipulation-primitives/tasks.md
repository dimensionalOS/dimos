## 1. Agentic Manipulation Module

- [ ] 1.1 Add `dimos/manipulation/agentic_manipulation_spec.py` with a `ManipulationControlSpec` Protocol covering robot state, joint motion, open gripper, and close gripper.
- [ ] 1.2 Add `dimos/manipulation/agentic_manipulation_module.py` with `AgenticManipulationModule` as a `Module` facade using Spec-injected manipulation control.
- [ ] 1.3 Decorate the facade primitives with `@skill`, include required docstrings and type annotations, and return the delegated `SkillResult` values.
- [ ] 1.4 Ensure the new module has no imports from Robosuite, benchmark runtime clients, sidecar packages, or script-only benchmark code.

## 2. Simulator-Free Tests

- [ ] 2.1 Add default unit tests for `AgenticManipulationModule` using a fake injected manipulation provider.
- [ ] 2.2 Verify each primitive delegates the expected arguments to the fake provider and passes the provider result back to the caller.
- [ ] 2.3 Verify the skill methods have schema-safe signatures and docstrings suitable for MCP exposure.
- [ ] 2.4 Run the new unit tests without Robosuite dependencies.

## 3. Robosuite Layer-2 Demo

- [ ] 3.1 Add `scripts/benchmarks/demo_agentic_manipulation_robosuite.py` following the sidecar startup, health wait, describe/reset, artifact, and cleanup structure from `demo_robosuite_panda_lift.py`.
- [ ] 3.2 Build a dynamic pre-blueprint robot configuration from the resolved Robosuite runtime plan and derive the `HardwareComponent`, coordinator `TaskConfig`, and `RobotModelConfig` needed by the stack.
- [ ] 3.3 Construct the in-script blueprint with `ControlCoordinator`, `ManipulationModule`, and `AgenticManipulationModule`, then start it with `ModuleCoordinator.build(...)`.
- [ ] 3.4 Implement a background SHM-to-sidecar stepping loop so blocking manipulation API calls can progress simulator state.
- [ ] 3.5 Call the `AgenticManipulationModule` API directly and fail hard unless robot state, open gripper, close gripper, and a safe small-offset joint motion report success.
- [ ] 3.6 Write episode config, runtime description, resolved runtime plan, API call summary, motor trace, score when available, sidecar log, and cleanup status artifacts.

## 4. Documentation and Validation

- [ ] 4.1 Document the Robosuite demo command and clarify that it is manual/script-hosted layer-2 validation, not a default unit test.
- [ ] 4.2 Document that the universal module is simulator-independent and that MCP filtering, LLM agent execution, Cartesian motion, and higher-level semantic manipulation skills are future work.
- [ ] 4.3 Run `uv run pytest dimos/manipulation/test_agentic_manipulation_module.py -v`.
- [ ] 4.4 If Robosuite dependencies are available, run the new script-hosted Robosuite demo and inspect its artifacts.
- [ ] 4.5 Run OpenSpec validation for this change and fix any proposal, spec, design, or task formatting issues.
