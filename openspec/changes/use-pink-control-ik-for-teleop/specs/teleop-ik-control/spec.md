## ADDED Requirements

### Requirement: Authoritative teleop robot model
The system SHALL configure every teleop IK task with Pink control IK backed by an authoritative `RobotModelConfig`. The task's ordered controlled joints SHALL match the model's ordered coordinator joint names, and the target frame SHALL be the model's named end-effector frame.

#### Scenario: Construct teleop for a named robot model
- **WHEN** a teleop IK task is constructed with a valid robot model whose coordinator joints match the task joints
- **THEN** the task uses Pink control IK and targets the model's named end-effector frame

#### Scenario: Reject mismatched controlled joints
- **WHEN** a teleop IK task's ordered joints do not match the robot model's ordered coordinator joints
- **THEN** task construction fails with an actionable configuration error

#### Scenario: Reject legacy model parameters
- **WHEN** a teleop IK task is configured only with a model path and numeric end-effector joint ID
- **THEN** task construction rejects the legacy configuration instead of selecting the legacy solver

### Requirement: Engagement-relative Cartesian target
The system SHALL interpret a teleop pose command as an end-effector delta relative to an engagement baseline captured by forward kinematics from the current measured coordinator joints. Translation SHALL be added to the baseline translation, and delta rotation SHALL left-multiply the baseline rotation.

#### Scenario: First command captures the measured baseline
- **WHEN** the first pose delta of an engagement is computed with a complete measured joint state
- **THEN** the task captures the corresponding measured end-effector pose as its engagement baseline and solves for the composed absolute target

#### Scenario: Subsequent commands retain the engagement baseline
- **WHEN** additional pose deltas arrive during the same engagement while measured joints change
- **THEN** each target is composed from the same engagement baseline rather than recapturing it from the latest joint state

#### Scenario: Missing measured joints defer baseline capture
- **WHEN** the task receives an engaged pose delta but the coordinator state lacks any controlled joint position
- **THEN** the task emits no joint command and does not capture a partial engagement baseline

### Requirement: Measured-state Pink control safety
For each active tick, the system SHALL seed Pink control IK from the current finite measured joint state, bound the coordinator timestep to configured limits, validate solver output, and enforce the configured per-tick joint-delta limit. An expected target-preparation or IK runtime failure after a complete measured state is available SHALL produce a measured-state servo-position hold.

#### Scenario: Valid target produces an arbitrated joint command
- **WHEN** Pink returns a finite, correctly shaped solution within configured limits
- **THEN** the task emits a servo-position command for the controlled arm joints

#### Scenario: Coordinator timestep is bounded
- **WHEN** the coordinator timestep is outside the configured positive minimum and maximum
- **THEN** the task uses the nearest configured bound for target preparation and Pink solving

#### Scenario: Solver failure holds measured state
- **WHEN** target preparation or Pink solving raises an expected runtime failure after measured joints are available
- **THEN** the task emits a servo-position hold at the measured arm positions

#### Scenario: Invalid or excessive solution holds measured state
- **WHEN** Pink returns a non-finite, incorrectly shaped, or excessive joint-delta solution
- **THEN** the task rejects the candidate and emits a servo-position hold at the measured arm positions

### Requirement: Fresh baseline after lifecycle discontinuity
The system SHALL discard the current pose target and engagement baseline on disengage, timeout, stop, clear, or E-STOP. The next accepted engagement SHALL capture a fresh baseline from the then-current measured robot state.

#### Scenario: Disengage and re-engage
- **WHEN** an operator disengages after commanding motion and later re-engages
- **THEN** the next computed delta is based on a newly measured engagement baseline

#### Scenario: Command stream times out
- **WHEN** no teleop pose update arrives within the configured nonzero timeout
- **THEN** the task becomes inactive, discards its target and baseline, and emits no command until a new engagement

#### Scenario: Task is stopped or cleared
- **WHEN** the task is stopped or cleared
- **THEN** its target and engagement baseline are discarded and it no longer participates in arbitration

### Requirement: Fail-closed E-STOP behavior
The system SHALL make a teleop IK task inert while E-STOP is latched, SHALL reject pose and gripper commands received while latched, and SHALL NOT replay pre-latch or latched commands when E-STOP is cleared.

#### Scenario: E-STOP while teleop is active
- **WHEN** E-STOP is latched during an active engagement
- **THEN** the task immediately clears its target and baseline and becomes inactive

#### Scenario: Commands arrive during E-STOP
- **WHEN** pose or gripper commands arrive while E-STOP is latched
- **THEN** the commands are rejected without changing the cached target, baseline, or gripper target

#### Scenario: E-STOP is cleared
- **WHEN** E-STOP is cleared after one or more commands were rejected
- **THEN** the task remains free of replayable commands and requires a fresh engagement baseline before arm motion resumes

### Requirement: Teleop gripper and arbitration behavior
The system SHALL preserve teleop gripper interpolation, resource claims, task-name-routed pose delivery, broadcast controller-button delivery, and joint-level arbitration while using Pink control IK.

#### Scenario: Analog trigger commands the gripper
- **WHEN** a configured hand supplies an analog trigger value from zero through one
- **THEN** the task clamps the value, interpolates between configured open and closed positions, and appends the gripper target to an active arm output

#### Scenario: Task claims the gripper
- **WHEN** a teleop task is configured with a gripper joint
- **THEN** its resource claim includes both the controlled arm joints and the gripper joint at the task priority

#### Scenario: Cartesian command is routed by task name
- **WHEN** a pose delta names a registered teleop task
- **THEN** the coordinator delivers it only to that named task

#### Scenario: Higher-priority task wins arbitration
- **WHEN** a higher-priority task claims any of the same joints as an active teleop task
- **THEN** coordinator arbitration gives those joints to the higher-priority task and reports preemption to teleop

### Requirement: Atomic shipped-blueprint migration
The system SHALL configure every shipped Piper, xArm6, xArm7, and mixed xArm/Piper teleop IK task through the authoritative Pink robot-model interface, with no shipped teleop task retaining legacy model-path or numeric end-effector-joint parameters.

#### Scenario: Inspect shipped teleop task configurations
- **WHEN** the shipped teleop blueprints are resolved
- **THEN** each teleop IK task contains a reconstructable Pink control configuration whose robot-model coordinator joints match its hardware joints

#### Scenario: Resolve mixed-arm teleop
- **WHEN** the mixed xArm/Piper teleop coordinator is resolved
- **THEN** each teleop task uses a robot model mapped to its own hardware namespace
