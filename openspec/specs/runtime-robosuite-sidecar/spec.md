## Purpose

Define the Robosuite runtime package boundary for baked Robosuite task execution, motor-surface mapping, observation export, and score/artifact reporting through a Simulator Runtime Module.

## Requirements

### Requirement: Robosuite sidecar package
The system SHALL evolve the first-class Robosuite runtime package into an import-safe package that provides a Simulator Runtime Module for Robosuite execution and removes the existing HTTP server boundary as part of migration success.

#### Scenario: Robosuite package imports in main DimOS environment
- **WHEN** a developer imports the Robosuite runtime module class or blueprint helper in the main DimOS environment
- **THEN** the import succeeds without importing Robosuite, MuJoCo renderer state, or backend-only dependencies

#### Scenario: Robosuite runtime installs in isolated environment
- **WHEN** a developer prepares the Robosuite runtime package in a Robosuite-compatible Python project runtime environment
- **THEN** the placed DimOS worker can import DimOS worker runtime support, runtime protocol models, and Robosuite backend dependencies

#### Scenario: HTTP entrypoint is removed
- **WHEN** the Robosuite Simulator Runtime Module covers runtime description, reset, step, observation streams, score, and demo execution
- **THEN** the package no longer exposes an HTTP runtime server entrypoint for benchmark execution

### Requirement: Baked Robosuite task instantiation
The Robosuite runtime package SHALL instantiate baked Robosuite tasks from episode config fields such as env name, robot name, controller profile, control frequency, horizon, renderer options, camera options, and seed.

#### Scenario: Panda Lift task starts
- **WHEN** the episode config requests `env_name: Lift` and `robots: Panda`
- **THEN** the runtime module creates the corresponding Robosuite environment and exposes its runtime description

### Requirement: Runtime-derived motor surface
The Robosuite runtime package SHALL derive robot motor surface metadata from the live Robosuite environment and controller setup rather than requiring every benchmark config to manually enumerate Robosuite action indices.

#### Scenario: Panda motor order is described
- **WHEN** the sidecar creates a Panda Lift environment
- **THEN** it reports a stable ordered motor surface suitable for DimOS whole-body motor control

#### Scenario: Unsupported controller profile fails
- **WHEN** the selected Robosuite controller profile cannot be mapped to a supported DimOS motor command mode
- **THEN** the runtime module rejects the episode setup with a protocol error that identifies the unsupported profile

### Requirement: Robosuite step ownership
The Robosuite runtime package SHALL own backend-native `env.reset()` and `env.step(action)` calls inside its Simulator Runtime Module and SHALL translate between runtime motor action frames, runtime state metadata, and Robosuite action/observation structures.

#### Scenario: Motor position step advances Robosuite through RPC
- **WHEN** DimOS calls module `step()` with a motor position action frame for the described Panda motor surface
- **THEN** the module maps it to the Robosuite action vector, steps the environment on the simulator owner thread, and returns reward, done, success if available, and lightweight step metadata

#### Scenario: Robosuite APIs stay on owner thread
- **WHEN** the module handles reset, step, render, or camera capture
- **THEN** those backend operations run on the simulator owner thread rather than directly on an RPC or stream callback thread

### Requirement: Observation export
The Robosuite runtime package SHALL expose configured Robosuite camera and state observations through DimOS-native typed streams from its Simulator Runtime Module, with RPC responses containing only lightweight observation metadata.

#### Scenario: Agentview camera is published as Image and CameraInfo streams
- **WHEN** the episode config enables the `agentview` camera and a step produces a frame
- **THEN** the module publishes the camera output as `Image` and camera metadata as `CameraInfo` through DimOS streams without requiring DimOS to fetch a `.npy` payload from an HTTP endpoint or receive a raw NumPy array in the step response

#### Scenario: Step response references published observation
- **WHEN** a step produces an observation frame
- **THEN** the step response includes sequence, timestamp, or stream metadata sufficient to correlate the step with stream output without embedding image tensors

### Requirement: Score and artifact export
The Robosuite runtime package SHALL provide score and artifact outputs for each episode, including reward/done/success metadata, backend timing, and runtime logs or trace summaries.

#### Scenario: Score is collected after demo
- **WHEN** the demo completes or times out
- **THEN** the runner can request score output from the runtime module and write it with the episode artifacts
