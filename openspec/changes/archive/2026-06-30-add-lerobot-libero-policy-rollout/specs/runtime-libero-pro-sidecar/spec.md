## ADDED Requirements

### Requirement: Native LIBERO action mode
The LIBERO sidecar SHALL support a native LIBERO action mode that follows the official LeRobot LIBERO action setup for relative end-effector delta plus gripper actions.

#### Scenario: Native action mode validates environment action spec
- **WHEN** the sidecar starts in native LIBERO action mode
- **THEN** it inspects the LIBERO environment action spec and requires action dimension `(7,)` with bounds compatible with `[-1, 1]`

#### Scenario: Native action mode is described
- **WHEN** the sidecar description is requested in native LIBERO action mode
- **THEN** it reports the native action surface identifier, action shape, action bounds, action mode metadata, task metadata, language, horizon, and camera configuration

#### Scenario: Native action mode accepts runtime action frame
- **WHEN** DimOS sends a runtime action frame with `space_id` `libero.ee_delta_6d_gripper.normalized.v1` and valid `float32[7]` values
- **THEN** the sidecar maps the values directly to the LIBERO environment step action and returns observations, reward, done, and success metadata

#### Scenario: Native action mode rejects motor frame
- **WHEN** the sidecar is running in native LIBERO action mode and receives a motor action frame
- **THEN** it rejects the step request with a clear protocol error

### Requirement: Native LIBERO observation export for policy rollout
The LIBERO sidecar SHALL export the observations needed by the VLA-JEPA LIBERO policy contract when running native LIBERO action mode.

#### Scenario: Policy observations are available after reset
- **WHEN** the sidecar resets a registered task in native LIBERO action mode
- **THEN** the reset response includes agent-view camera observation metadata, wrist or eye-in-hand camera observation metadata when available, robot state observation metadata, and task language metadata for contract conversion

#### Scenario: Policy observations are available after step
- **WHEN** the sidecar completes a native runtime action step
- **THEN** the step response includes updated camera and robot state observations needed for the next policy inference tick

## MODIFIED Requirements

### Requirement: LIBERO-PRO motor surface validation
The LIBERO-PRO sidecar SHALL expose the full-control motor-frame path only when the selected task and LIBERO action mode provide a Panda joint-position plus gripper whole-body motor surface compatible with DimOS motor action frames.

#### Scenario: Compatible motor surface is described
- **WHEN** the selected LIBERO-PRO environment exposes the expected Panda joint-position plus gripper action surface for motor-frame mode
- **THEN** the runtime description reports a stable ordered motor surface with supported position command mode and the expected motor count

#### Scenario: Incompatible motor mode fails fast
- **WHEN** motor-frame mode is selected but the LIBERO environment exposes only a native end-effector action surface or an action dimension that cannot be mapped to Panda joint-position plus gripper commands
- **THEN** the sidecar rejects the episode setup with a clear protocol error before accepting motor-frame step requests
