## Why

Declarative teleop IK still uses a separate legacy Pinocchio solver and identifies robot models with file paths and numeric end-effector joint IDs. Moving teleop onto the measured-state Pink control pipeline gives Cartesian, twist, and teleop control one model authority and one safety-critical IK implementation.

## What Changes

- Make the teleop IK task a focused specialization of the Cartesian IK task, reusing Pink solving, measured-state anchoring, bounded tick timing, output validation, joint-limit handling, and measured-state holds.
- Preserve teleop-specific engage-relative pose interpretation, controller-button behavior, gripper control, task-name routing, arbitration, and preemption.
- Discard cached targets and engagement baselines on disengage, timeout, stop, clear, or E-STOP; reject pose and gripper commands while E-STOP is latched.
- Use each authoritative `RobotModelConfig` named end-effector frame, including Piper's `gripper_base`.
- Migrate Piper, xArm6, xArm7, and mixed xArm/Piper teleop blueprints together.
- **BREAKING**: Replace teleop IK's `model_path` and numeric `ee_joint_id` parameters with `RobotModelConfig`-backed Pink control configuration; the legacy configuration is not supported concurrently.

## Capabilities

### New Capabilities

- `teleop-ik-control`: Engage-relative manipulator teleoperation through measured-state Pink control IK, including model authority, safety, E-STOP, gripper, and lifecycle behavior.

### Modified Capabilities

None.

## Impact

- Affects the teleop control task and factory, shared manipulator blueprint helpers, Piper and xArm teleop blueprints, the mixed-manipulator coordinator, and their tests and manipulation documentation.
- Makes the existing optional Pink/manipulation dependencies required when constructing a teleop IK task.
- Intentionally changes Piper's controlled frame from legacy joint 6 to the model's named `gripper_base` frame and may require simulation and hardware tuning.
