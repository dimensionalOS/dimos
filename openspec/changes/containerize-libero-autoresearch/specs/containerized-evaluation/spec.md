## ADDED Requirements

### Requirement: Locked local evaluation image
The system SHALL build the outer environment for every built-in `dimos eval` locally from digest-pinned base images, checked-in dependency locks, and the current candidate source. It SHALL execute the image by the immutable image ID returned by the build.

#### Scenario: Build current candidate
- **WHEN** a user starts a containerized check or benchmark
- **THEN** the launcher builds the current checkout and runs the exact returned image ID without pulling a project image from a registry

### Requirement: Container-only public execution
The public `dimos eval run` command SHALL execute built-in Evaluations only inside the locked outer image and SHALL NOT expose a host execution fallback.

#### Scenario: Run a built-in Evaluation
- **WHEN** a user invokes `dimos eval run` for LIBERO-PRO or VLN-CE
- **THEN** the dependency-light host command launches the immutable image and the existing Python runner executes only inside it

### Requirement: Rootless sibling simulator topology
The system SHALL use the host user's rootless Podman socket to launch native simulator images as sibling containers and SHALL NOT run a nested container daemon.

#### Scenario: Start a native trial
- **WHEN** an outer evaluation starts LIBERO-PRO or VLN-CE
- **THEN** its existing simulator container is created through the mounted host socket and its endpoints remain reachable through host networking

### Requirement: Daemon-visible artifacts and bounded temporary storage
The system SHALL mount the read-only specification directory, private output staging root, Evo paths, and cache at identical absolute host and outer-container paths. It SHALL provide a short host-backed temporary mount for IPC and atomically publish a complete result directory.

#### Scenario: Simulator consumes cached assets
- **WHEN** the outer runner passes an asset or artifact path to the host Podman service
- **THEN** that absolute path identifies the same host file for the sibling simulator

#### Scenario: Agent creates a Unix socket
- **WHEN** the evaluation creates temporary Jupyter or multiprocessing IPC
- **THEN** it uses the short `/runner-tmp` path rather than host `/tmp` or a long worktree path

#### Scenario: Evaluation exits with a recorded failure
- **WHEN** the inside runner publishes a valid failed or cancelled `run.json`
- **THEN** the host publishes that complete result directory and propagates the container exit status

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

### Requirement: Built-in-only registry
The executable Evaluation registry SHALL resolve only explicitly registered built-ins and SHALL NOT discover installed Python entry points.

#### Scenario: Resolve an unknown Evaluation
- **WHEN** a specification names an Evaluation outside the built-in registry
- **THEN** preflight fails with the available built-in names without importing installed plugin code

### Requirement: Shared autoresearch bootstrap
The LIBERO autoresearch module SHALL use the same outer launcher while retaining its existing command-line interface and Evo publication contract.

#### Scenario: Run the fixed Evo benchmark command
- **WHEN** Evo invokes the autoresearch Python module from the host checkout
- **THEN** the module launches itself in the locked image exactly once and publishes native panel results through the existing Evo paths
