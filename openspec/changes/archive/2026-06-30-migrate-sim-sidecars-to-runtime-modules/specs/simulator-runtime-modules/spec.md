## ADDED Requirements

### Requirement: Simulator Runtime Module boundary
The system SHALL provide import-safe Simulator Runtime Modules that expose benchmark simulator runtimes as first-class DimOS Modules while keeping simulator-heavy imports inside the placed worker runtime environment.

#### Scenario: Coordinator imports module without simulator dependencies
- **WHEN** the main DimOS environment imports a simulator runtime module class or package-local blueprint helper
- **THEN** the import succeeds without importing Robosuite, LIBERO-PRO, Torch, OmniGibson, or backend asset libraries

#### Scenario: Worker owns simulator backend
- **WHEN** a placed simulator runtime module starts in its named runtime environment
- **THEN** backend-specific simulator imports, environment construction, reset, stepping, rendering, and scoring occur inside that worker environment

### Requirement: Simulator runtime blueprint helper
Each simulator runtime package SHALL provide a package-local blueprint helper that registers a named Python project runtime environment and places its Simulator Runtime Module using the standard blueprint-level runtime placement API.

#### Scenario: Default helper places simulator module
- **WHEN** a caller uses the package-local simulator runtime blueprint helper without custom placement arguments
- **THEN** the returned blueprint registers a default `PythonProjectRuntimeEnvironment` and maps the simulator runtime module class to that environment name

#### Scenario: Caller overrides runtime environment
- **WHEN** a caller supplies a compatible runtime environment override to the helper
- **THEN** the helper uses the supplied environment name for module placement without requiring module-local deployment flags

### Requirement: Simulator runtime control RPCs
Simulator Runtime Modules SHALL expose module RPCs for runtime description, episode reset, synchronous step, and score collection.

#### Scenario: Reset establishes episode state
- **WHEN** a benchmark runner calls `reset()` on a Simulator Runtime Module with a valid episode request
- **THEN** the module establishes the new episode state before simulation time advances and returns reset metadata synchronously

#### Scenario: Missing setup fails through reset
- **WHEN** required simulator assets, task files, or backend configuration are missing for the requested episode
- **THEN** `reset()` fails synchronously with an actionable error rather than relying on event-topic timeout behavior

#### Scenario: Step blocks until benchmark tick completes
- **WHEN** a benchmark runner calls `step()` with a valid runtime motor action frame
- **THEN** the call returns only after the simulator has applied the action and completed the corresponding benchmark tick

### Requirement: Simulator runtime owner thread
Simulator Runtime Modules MUST execute backend reset, step, render, and observation capture on a simulator owner thread or main-thread runtime loop, not directly from DimOS RPC or stream callback threads.

#### Scenario: RPC handler marshals simulator mutation
- **WHEN** a DimOS RPC handler receives a reset or step request
- **THEN** it marshals the backend mutation to the simulator owner thread and waits for the owner-thread result

#### Scenario: Backend requires main thread
- **WHEN** a visual simulator backend requires process main-thread ownership for rendering or event handling
- **THEN** the runtime module uses a main-thread worker mode or equivalent owner-loop pattern that preserves backend thread affinity

### Requirement: Runtime motor action frame input
Simulator Runtime Module `step()` SHALL accept the runtime-derived ordered motor action frame for the simulator's declared whole-body motor surface rather than backend-native opaque action vectors.

#### Scenario: Ordered motor action is validated
- **WHEN** `step()` receives a motor action frame
- **THEN** the module validates robot id, command mode, ordered motor names, and field lengths against the runtime description before applying the action

#### Scenario: Backend action vector is internal
- **WHEN** the simulator backend requires an action vector or backend-specific command object
- **THEN** the module translates the validated runtime motor action frame internally without exposing backend-native action vectors at the DimOS boundary

### Requirement: Simulator runtime data streams
Simulator Runtime Modules SHALL publish large and continuous runtime data through DimOS-native typed streams rather than embedding large payloads or raw NumPy arrays in RPC responses.

#### Scenario: Camera observation publishes as Image and CameraInfo streams
- **WHEN** a simulator step produces a configured camera observation
- **THEN** the module publishes image data as `dimos.msgs.sensor_msgs.Image` and camera metadata as `CameraInfo` through typed DimOS output streams using the blueprint-selected transport

#### Scenario: Depth observation avoids lossy image compression by default
- **WHEN** a simulator step produces a depth observation
- **THEN** the module publishes depth as an `Image` with a depth-compatible format and uses a raw typed transport unless the blueprint explicitly selects an acceptable compression strategy

#### Scenario: Step response stays lightweight
- **WHEN** `step()` completes after producing observations
- **THEN** the RPC response contains control/evaluation metadata and observation sequence or timestamp references, not image/depth tensor payloads

#### Scenario: Raw NumPy array stays internal
- **WHEN** simulator backend code produces an image as a NumPy array
- **THEN** the array is wrapped in a DimOS `Image` message before crossing the module stream boundary

### Requirement: HTTP runtime removal gate
The system SHALL treat removal of existing HTTP runtime sidecar servers, clients, payload fetch endpoints, and HTTP-first demos as a success gate for the Simulator Runtime Module migration.

#### Scenario: Migration is incomplete while HTTP runtime boundary remains
- **WHEN** a migrated simulator runtime package still requires an HTTP server, HTTP client, HTTP payload endpoint, or HTTP-first demo path for benchmark execution
- **THEN** the migration is not considered complete for that runtime

#### Scenario: HTTP is removed when module coverage lands
- **WHEN** fake, Robosuite, and selected LIBERO-PRO module-native paths cover import boundaries, runtime preparation, control RPCs, data streams, benchmark parity, and active consumers no longer require HTTP
- **THEN** the system removes HTTP runtime entrypoints, HTTP payload fetch APIs, HTTP client code, and HTTP-first runtime demo paths
