# policy-action-control Specification

## Purpose
TBD - created by archiving change add-live-policy-stream-rollout. Update Purpose after archive.

## Requirements

### Requirement: Policy action chunk coordinator input
The system SHALL allow ControlCoordinator to receive robot policy action chunks as first-class live control inputs and route them to policy-action-aware control tasks without requiring a separate conversion module.

#### Scenario: Coordinator routes policy action chunks to task
- **WHEN** a robot policy module publishes a valid robot policy action chunk to the coordinator policy chunk input
- **THEN** ControlCoordinator delivers the chunk to a configured policy chunk control task that accepts the chunk action-space identity

#### Scenario: Invalid policy chunk is rejected before execution
- **WHEN** a policy chunk has an unsupported action-space id, incompatible shape, non-finite value, or value outside the task's accepted normalized range
- **THEN** the policy chunk control task rejects the chunk and does not emit control commands from it

### Requirement: Index-bounded policy chunk execution
The system SHALL provide a ControlCoordinator policy chunk control task that owns index-bounded execution of robot policy action chunks.

#### Scenario: Task executes configured leading actions
- **WHEN** the policy chunk control task receives a chunk with more actions than its configured execution count
- **THEN** it executes only the configured leading actions before requesting or waiting for a replacement chunk

#### Scenario: Task advances chunk execution by coordinator ticks
- **WHEN** the policy chunk control task is active during ControlCoordinator ticks
- **THEN** it advances through the selected chunk actions according to its configured tick-count execution cadence

#### Scenario: Task stops contributing when chunk is stale
- **WHEN** the policy chunk control task has no fresh chunk or its active chunk exceeds the configured staleness limit
- **THEN** it stops contributing commands to ControlCoordinator arbitration rather than continuing stale policy actions

### Requirement: Policy chunk refill trigger
The system SHALL allow a policy chunk control task to request a new policy action chunk from the robot policy module through a fast inference trigger that does not block ControlCoordinator on backend inference.

#### Scenario: Empty task triggers policy inference
- **WHEN** the policy chunk control task consumes its configured chunk prefix and has no replacement chunk available
- **THEN** it sends a fast inference trigger to the robot policy module and returns no control command while waiting for a stream-delivered chunk

#### Scenario: Trigger returns before backend inference completes
- **WHEN** the policy chunk control task sends the inference trigger from its control execution path
- **THEN** the trigger returns without running the slow backend inference synchronously in the ControlCoordinator tick loop

### Requirement: Policy chunk task maps normalized actions to coordinator commands
The system SHALL let the policy chunk control task own physical execution mapping from normalized robot policy action values to ControlCoordinator joint or gripper commands.

#### Scenario: LIBERO-style end-effector delta action is mapped inside task
- **WHEN** the policy chunk task is configured for normalized end-effector delta plus gripper actions
- **THEN** it maps the selected normalized action into coordinator-compatible arm and gripper commands inside the task rather than requiring an external conversion module

#### Scenario: Policy module remains outside control execution mapping
- **WHEN** the robot policy module emits a normalized robot policy action chunk
- **THEN** it does not denormalize the action into physical controller units or call runtime/control execution APIs directly
