## ADDED Requirements

### Requirement: Isolated disposable workspace
Every case SHALL execute in a disposable rootless Podman container whose staged input is read-only and whose working directory is writable. Container creation SHALL explicitly override the image entrypoint. After start, the host SHALL verify that the container is running and that sandbox exec is ready, with liveness checked before and after each sandbox exec. Rootless Podman availability SHALL be verified on the host, and run-time validation SHALL fail closed if the runtime, image, entrypoint, mounts, readiness, or isolation requirements are not satisfied. Container death or a container-runtime/control-plane error SHALL be a terminal infrastructure failure, not a recoverable case result. The host SHALL expose no Pi tools directly; only the case-bound tools provided for that run may be callable by the agent.

#### Scenario: Inspect the case boundary
- **WHEN** an agent executes inside a running case container
- **THEN** it can read the staged input, write its working directory, and cannot modify staged input or call host Pi tools; the host has verified running and exec readiness

#### Scenario: Detect an invalid sandbox
- **WHEN** the image entrypoint is not explicitly overridden, the container exits, or a container-runtime/control-plane error occurs before or during an exec
- **THEN** the run is classified terminal `container_runtime_failed` and cannot be accepted as a valid precursor

### Requirement: Evidence export before destruction
The system SHALL attempt to export the working directory and logs before destroying the case container, SHALL retain successful or partial failed-run evidence, and SHALL destroy the case container unconditionally in `finally` cleanup.

#### Scenario: Finish an isolated run
- **WHEN** a case run ends normally or fails
- **THEN** the system attempts to export the working-directory contents and execution logs, retains failed-run evidence if export fails, and destroys the container regardless of export outcome
