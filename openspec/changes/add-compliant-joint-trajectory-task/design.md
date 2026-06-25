## Context

DimOS control tasks currently expose a simple coordinator contract: each active task claims joints, computes a final `JointCommandOutput`, and the coordinator arbitrates one command per joint before routing to hardware. The existing passive joint trajectory task samples a nominal trajectory each tick and emits `SERVO_POSITION` commands directly. If a separate compliance task were run at the same time on the same joints, the two tasks would compete in arbitration instead of composing.

Manipulator hardware support is also uneven. The common adapter path provides joint positions and may provide velocities/efforts, but torque command output is not available through the generic coordinator-to-manipulator route. xArm, Piper, OpenArm, A750, mock, and MuJoCo-backed adapters all support position commands; velocity and effort feedback quality varies by backend. The first compliant trajectory capability therefore needs to be position-servo based and capability-adaptive.

MuJoCo simulation already supports manipulator scenes, SHM joint state exchange, position/velocity command targets, and effort feedback. It can be used to verify the behavior implemented here, but not as proof of a future torque impedance backend.

## Goals / Non-Goals

**Goals:**

- Preserve the coordinator's existing final-command arbitration model.
- Add a narrow composition pattern where one source control task can be followed by one or more reference-transforming control tasks.
- Add compliant joint trajectory execution by composing trajectory sampling with bounded joint-space compliance.
- Keep rigid trajectory execution unchanged and selectable.
- Support operation without reliable effort feedback by using position/velocity feedback as the baseline signal.
- Provide unit, task-composition, and MuJoCo verification coverage.

**Non-Goals:**

- No coordinator-level task graph, DAG scheduler, or ROS2-style exported reference interface system.
- No torque impedance control in the first implementation.
- No automatic inference that zero-valued effort feedback is reliable.
- No runtime controller rewiring UI or CLI.
- No broad refactor of teleop, whole-body control, navigation, or Cartesian controllers.

## DimOS Architecture

The change should add a small composition layer in the control package, either near `dimos/control/task.py` or in a focused composition module. The coordinator-facing composed task remains a normal `ControlTask`: it exposes one name, one claim set, one active lifecycle, and one final `JointCommandOutput`. Internally, it calls a source task and then ordered transform-capable tasks.

The reference transform interface should be typed around existing coordinator data structures:

```text
compute_from_reference(CoordinatorState, JointCommandOutput) -> JointCommandOutput | None
```

Standalone control tasks continue to implement `compute(CoordinatorState)`. A transform-capable task can implement both standalone behavior and reference-transform behavior. In the compliant trajectory use case, the source is the existing joint trajectory task, and the transform is a joint compliance task that consumes `SERVO_POSITION` positions and emits adjusted `SERVO_POSITION` positions.

The compliant task should use current joint state from `CoordinatorState`:

- joint position is required for each controlled joint;
- velocity is optional but used when available;
- effort feedback is opt-in/configured and must not be assumed merely because an adapter returns a list of zeros.

The compliance state should maintain per-joint offset and offset velocity. Each tick, it updates offset using an admittance-style virtual mass/damping/stiffness model, clamps offset and offset velocity, and emits `q_cmd = q_ref + offset`. The output mode remains `SERVO_POSITION`, so existing manipulator hardware routing continues to call position-command methods.

Blueprint and CLI exposure should be minimal. Existing rigid trajectory tasks remain the default unless a blueprint/configuration explicitly selects compliant trajectory behavior. No skill/MCP tool exposure is required.

If implementation adds or renames blueprint variables, the generated registry must be regenerated with `pytest dimos/robot/test_all_blueprints_generation.py`. If no blueprint registry surface changes, no generated registry update is needed.

## Decisions

1. **Use internal task composition, not coordinator-level chainability.**
   - Rationale: the coordinator currently arbitrates final joint commands. Adding a graph scheduler would mix arbitration semantics with reference transformation semantics.
   - Alternative considered: ROS2-style chainable controllers with exported reference interfaces. This was rejected for v1 because DimOS lacks controller-manager ordering and broad intermediate reference contracts.

2. **Use position-servo compliance first.**
   - Rationale: all target manipulator paths can accept position commands, while torque output is not generally exposed.
   - Alternative considered: torque impedance. This is deferred until adapter protocols and hardware routes support explicit torque command output.

3. **Make effort feedback explicit, not auto-detected.**
   - Rationale: some adapters return zero efforts as a placeholder. Treating zeros as reliable torque data would create misleading contact behavior.
   - Alternative considered: infer effort availability from nonempty effort arrays. This is unsafe with current adapters.

4. **Keep composition narrow and linear.**
   - Rationale: the initial use case is trajectory source plus compliance transform. A generic graph DSL would add complexity before multiple use cases prove the need.

5. **Verify with joint-space metrics first, contact demos second.**
   - Rationale: controller math and safety bounds can be tested deterministically without MuJoCo; contact scenes are useful integration evidence but depend on simulation asset tuning.

## Safety / Simulation / Replay

Safety constraints should be explicit in configuration and tests:

- maximum offset per joint;
- maximum offset velocity per joint;
- optional maximum sustained tracking error or effort threshold;
- behavior for missing joint state;
- behavior for invalid or nonpositive `dt`;
- reset behavior on start, completion, abort, and preemption.

Under missing required joint position state, the compliant transform should fail safe by returning no command or a documented hold/pass-through behavior rather than producing an arbitrary offset. Under clamp saturation, diagnostics should make saturation visible to tests and operators.

MuJoCo verification should use existing manipulator simulation as much as possible. The first smoke test should run without contact and confirm that compliant output tracks similarly to rigid trajectory output when no disturbance exists. A second integration/demo scene should drive an end effector into a rigid table or obstacle and compare rigid trajectory behavior against compliant trajectory behavior using peak effort, tracking error, compliance offset, and saturation metrics. A soft cushion/contact scene can follow after the rigid contact path is stable.

Replay support is not a primary target because this is an active control behavior. Any replay-based tests should be limited to deterministic state/command calculations rather than claiming physical contact validation.

## Risks / Trade-offs

- **Tuning risk:** mass/damping/stiffness defaults may be unstable or ineffective across robots. Mitigation: conservative defaults, per-joint clamps, and tests that verify bounded behavior.
- **Feedback quality risk:** adapters differ in velocity and effort fidelity. Mitigation: require positions, treat velocities/efforts as optional, and make effort use opt-in.
- **Simulation realism risk:** MuJoCo contact behavior depends on scene geometry and contact parameters. Mitigation: use MuJoCo as integration evidence, not the only acceptance criterion.
- **Abstraction risk:** a reusable composition pattern can grow into a graph framework prematurely. Mitigation: keep v1 linear and scoped to one source plus transforms.
- **Safety responsibility risk:** safety may be split across transform, source task, coordinator, and hardware adapter. Mitigation: transform-level clamps are mandatory, while coordinator and hardware routing remain unchanged.

## Migration / Rollout

Roll out in layers:

1. Add composition abstractions and unit tests without changing existing trajectory behavior.
2. Add joint compliance transform with deterministic tests.
3. Add compliant trajectory wrapper/factory and integration tests.
4. Add optional blueprint/configuration exposure for simulation or manipulator stacks.
5. Add documentation for compliant trajectory semantics, safety bounds, and verification approach.

Existing users can keep using rigid trajectory tasks. Rollback is straightforward if the new task is not selected by default: remove the new task/configuration and keep existing trajectory behavior.

## Open Questions

- Should the first implementation pass through the reference or return no command when joint state for a controlled joint is missing?
- Which robot/sim blueprint should be the canonical MuJoCo verification entry point: xArm7 first, Piper second, or a new minimal manipulator verification blueprint?
- Should sustained offset saturation abort the task in v1, or only report diagnostics and continue clamped?
- What configuration format should expose compliance gains and clamps to blueprints without overcommitting to a public API too early?
