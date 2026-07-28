## Context

`TeleopIKTask` currently duplicates Cartesian task concerns around measured joint access, forward kinematics, inverse kinematics, output construction, and joint-delta safety. It constructs a legacy iterative `PinocchioIK` solver from a model file and numeric joint ID, while `CartesianIKTask` and `EEFTwistTask` use `PinkControlIK` configured by the authoritative `RobotModelConfig`. `EEFTwistTask` already demonstrates a target-source specialization of `CartesianIKTask`.

The teleop task must continue interpreting Quest/hosted-controller messages as pose deltas relative to the robot pose at engagement, must remain a declarative coordinator task, and must retain gripper and E-STOP behavior. The migration affects Piper, xArm6, xArm7, and their mixed-arm composition.

## Goals / Non-Goals

**Goals:**

- Give every shipped teleop IK task the same measured-state Pink solve and safety pipeline as Cartesian IK.
- Preserve engage-relative pose semantics, gripper behavior, task-name routing, arbitration, and preemption.
- Make `RobotModelConfig` the sole authority for controlled joints and the named end-effector frame.
- Fail closed across E-STOP and runtime solver failures.
- Migrate every in-repository teleop IK configuration atomically.

**Non-Goals:**

- Refactor Cartesian, twist, and teleop tasks around a new composed control engine.
- Change controller-side delta generation, task registry routing, coordinator arbitration, or gripper units.
- Add collision avoidance to the real-time Pink control task.
- Preserve the legacy model-path and numeric end-effector-joint configuration.
- Tune final hardware-specific control gains without simulation and hardware evidence.

## Decisions

### Teleop is a focused Cartesian IK specialization

`TeleopIKTaskConfig` will extend `CartesianIKTaskConfig` with hand and gripper fields, and `TeleopIKTask` will extend `CartesianIKTask`. The parent will own joint/model validation, Pink construction, measured-state reads, bounded coordinator `dt`, IK execution, result validation, joint-delta checks, measured-state holds, timeout bookkeeping, and the arm resource claim.

The child will own engagement-baseline state, delta-to-absolute target preparation, E-STOP gating, controller buttons, gripper interpolation, the extended gripper claim, and appending the gripper command to the parent output.

Composition was considered because repository guidance generally prefers it. It is deferred because the current parent already exposes target-preparation and timeout hooks, `EEFTwistTask` proves the specialization pattern, and extracting a new engine would broaden this safety migration. A composed module can replace the inheritance seam later if concrete strain appears across multiple specializations.

### The child prepares an absolute target from an engagement-relative delta

On the first compute after a new engaged delta arrives, the task will capture forward kinematics from the current measured coordinator joints as the engagement baseline. It will preserve the established transform:

- target translation = baseline translation + delta translation
- target rotation = delta rotation × baseline rotation

The resulting normalized, finite `SE3` target is passed through the parent's Pink compute pipeline. Disengage, timeout, stop, clear, or E-STOP discards the baseline so the next engagement starts from the then-current measured pose.

### E-STOP discards rather than defers commands

Latching E-STOP will make the task inert and clear the pose target, engagement baseline, and transient engagement state. Pose and gripper commands received while latched will return rejection without changing cached state. Clearing E-STOP will not restore any prior command; a fresh post-clear engagement and baseline are required.

This avoids replaying in-flight or stale commands after a safety discontinuity.

### RobotModelConfig is the only model authority

The teleop factory and blueprint helper will accept the same nested Pink control configuration used by Cartesian and EEF-twist tasks. `model_path` and `ee_joint_id` will be removed from teleop parameters without a compatibility branch.

Pink will target `RobotModelConfig.end_effector_link` and map the ordered coordinator joints through `joint_name_mapping`. Mixed-arm blueprints will construct robot models whose names match their hardware namespaces. Piper will intentionally move from legacy joint 6 to the model's `gripper_base` frame.

### Existing declarative routing remains stable

The task type remains `teleop_ik`. Its task card continues consuming task-name-routed Cartesian commands and broadcast teleop buttons. No coordinator stream or transport changes are required. The blueprint helper changes only how task parameters are assembled.

### Verification is behavior-focused

Task tests will exercise behavior through the task interface with a surgical fake Pink backend. They will cover exact delta composition, measured-state baseline capture and reseeding, bounded `dt`, valid output, measured holds, joint-delta rejection, E-STOP rejection, timeout, and gripper claim/output. Blueprint tests will verify that every shipped teleop task carries an authoritative reconstructable Pink configuration and no legacy model fields.

Simulation smoke tests will validate named frames and operator motion before real Piper or xArm hardware rollout.

## Risks / Trade-offs

- **Piper's controlled point changes from joint 6 to `gripper_base`** → Treat this as intentional, validate translation and rotation behavior in simulation, then perform a low-speed hardware check.
- **Pink's one-step differential response differs from the legacy multi-iteration solver** → Start with conservative existing Pink limits, retain the outer joint-delta guard, and tune gains only from measured simulation/hardware behavior.
- **Inheritance couples teleop to protected Cartesian task state** → Keep overrides limited to documented hooks and teleop policy; defer a composition refactor until concrete additional variation justifies a new seam.
- **Pink becomes mandatory for teleop task construction** → Keep imports actionable when the manipulation extra is absent and cover the failure path in tests.
- **Atomic configuration removal is breaking** → Update all known in-repository callers and blueprint assertions in the same change so no shipped mixed solver configuration remains.
- **Pose and button streams can arrive concurrently** → Guard all task-owned transient state consistently and test E-STOP/engagement transitions rather than relying on stream ordering.

## Migration Plan

1. Refactor the teleop configuration and class onto the Cartesian/Pink pipeline while preserving its registry type and streams.
2. Change the shared teleop blueprint helper to resolve `RobotModelConfig` into Pink control configuration.
3. Update Piper, xArm6, xArm7, and mixed-arm call sites atomically, including hardware-namespace-specific robot models.
4. Replace legacy teleop solver tests with behavior tests at the task interface and extend blueprint configuration coverage.
5. Update manipulation documentation and remove teleop references to model paths or numeric end-effector joint IDs.
6. Run focused unit and blueprint tests, then the relevant broader test suite and type/style checks.
7. Exercise each shipped teleop blueprint in simulation; perform low-speed hardware validation after simulation succeeds.

Rollback is a source-level revert of the task, helper, call-site, test, and documentation changes as one unit. There is no persisted data migration.

## Open Questions

None. Hardware-specific Pink gains remain rollout tuning rather than an unresolved architecture decision.
