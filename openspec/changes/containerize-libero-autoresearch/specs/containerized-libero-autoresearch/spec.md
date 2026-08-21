## ADDED Requirements

### Requirement: Locked local evaluation image
The system SHALL build the outer LIBERO autoresearch environment locally from digest-pinned base images, the checked-in Python lock, the checked-in Pi package lock, and the current candidate source. It SHALL execute the image by the immutable image ID returned by the build.

#### Scenario: Build current candidate
- **WHEN** a user starts a containerized check or benchmark
- **THEN** the launcher builds the current checkout and runs the exact returned image ID without pulling a project image from a registry

### Requirement: Rootless sibling simulator topology
The system SHALL use the host user's rootless Podman socket to launch the existing LIBERO simulator image as sibling containers and SHALL NOT run a nested container daemon.

#### Scenario: Start a LIBERO trial
- **WHEN** the outer evaluation starts a native trial
- **THEN** the existing simulator container is created through the mounted host socket and its loopback RPC endpoints are reachable through host networking

### Requirement: Daemon-visible artifacts and bounded temporary storage
The system SHALL mount outputs, Evo result paths, traces, and the verified asset cache at identical absolute host and outer-container paths. It SHALL provide a short dedicated temporary mount with sufficient host-backed capacity for IPC.

#### Scenario: Simulator consumes cached assets
- **WHEN** the outer runner passes an asset or artifact path to the host Podman service
- **THEN** that absolute path identifies the same host file for the sibling simulator

#### Scenario: Agent creates a Unix socket
- **WHEN** the evaluation creates temporary Jupyter or multiprocessing IPC
- **THEN** it uses the short `/runner-tmp` path rather than host `/tmp` or a long worktree path

### Requirement: Explicit runtime authority
The system SHALL pass the host network, NVIDIA CDI GPU, rootless Podman socket, and only an allowlist of required credential and Evo environment variables. It SHALL generate and use a per-run CDI specification without modifying system CDI configuration.

#### Scenario: Construct runner command
- **WHEN** the launcher constructs the outer container invocation
- **THEN** it includes explicit GPU/socket/network access and excludes unrelated host environment variables

### Requirement: Cheap preflight
The system SHALL provide a check mode that verifies the baked candidate import, locked Pi build, GPU visibility, and connectivity to the rootless Podman service without executing the panel.

#### Scenario: Missing service
- **WHEN** a required runtime service or artifact is unavailable
- **THEN** check mode exits non-zero with an actionable error before any benchmark case starts
