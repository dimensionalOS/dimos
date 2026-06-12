# topic-monitor-sidecar Specification

## Purpose
Specify the attach-style `dimos topic monitor` sidecar that observes live LCM traffic, owns independent Rerun/Reflex services, and logs only explicitly applied topics without modifying the running DimOS stack.
## Requirements
### Requirement: Foreground topic monitor command
The system SHALL provide a `dimos topic monitor` command that starts an interactive foreground topic monitor sidecar without requiring the target robot or simulation blueprint to include selector-specific visualization wiring.

#### Scenario: User starts monitor with active run
- **GIVEN** a DimOS run is active and publishing LCM traffic
- **WHEN** the user runs `dimos topic monitor`
- **THEN** the command starts a foreground topic monitor sidecar
- **AND** the command uses the active run as context for printed metadata
- **AND** the command does not modify the running blueprint or restart any robot modules.

#### Scenario: User starts monitor without active run
- **GIVEN** no DimOS run is active
- **WHEN** the user runs `dimos topic monitor`
- **THEN** the command starts in LCM bus-only mode
- **AND** the command can display live LCM topics produced by external or manual publishers
- **AND** the command makes clear in CLI output that no active run was selected.

#### Scenario: User stops monitor
- **GIVEN** `dimos topic monitor` is running
- **WHEN** the user presses Ctrl-C or sends SIGTERM to the monitor process
- **THEN** the monitor stops its selector UI, selector API, Rerun viewer services, and LCM subscriptions
- **AND** the original DimOS run, if any, continues according to its own lifecycle.

### Requirement: Independent sidecar visualization
The system SHALL run the topic monitor as an independent visualization sidecar whose stage/apply state controls only the monitor's own selected-only Rerun logging path.

#### Scenario: Existing blueprint visualization is present
- **GIVEN** a running DimOS stack already has its own visualization or Rerun bridge
- **WHEN** the user starts `dimos topic monitor`
- **THEN** the monitor starts its own independent sidecar visualization
- **AND** monitor selections do not change what the existing blueprint visualization logs
- **AND** the monitor does not attempt to reuse or control the existing visualization by default.

#### Scenario: User applies selected topics
- **GIVEN** the topic monitor catalog contains a renderable typed LCM topic
- **WHEN** the user stages the topic and applies the selection in the monitor UI
- **THEN** subsequent messages for that topic are decoded, converted, and logged to the monitor's own Rerun viewer
- **AND** unselected topics remain catalog-visible without being converted or logged by the monitor.

### Requirement: Automatic browser workflow and isolated ports
The system SHALL allocate an isolated set of available local ports for monitor-owned services and open the selector web page automatically by default when possible.

#### Scenario: Default ports are occupied
- **GIVEN** the default Rerun or selector ports are already occupied by another process
- **WHEN** the user starts `dimos topic monitor`
- **THEN** the monitor chooses available isolated ports for its own services
- **AND** the monitor prints the actual selector and Rerun viewer URLs.

#### Scenario: Browser open fails
- **GIVEN** the monitor has started successfully but the system cannot open a browser automatically
- **WHEN** browser opening fails
- **THEN** the monitor remains running
- **AND** the CLI output includes the selector URL for manual opening.

#### Scenario: User disables browser opening
- **GIVEN** the user does not want automatic browser launch
- **WHEN** the user starts the monitor with the documented no-open option
- **THEN** the monitor starts without trying to open a browser
- **AND** the CLI output includes the selector URL.

### Requirement: Discover decodable DimOS LCM topics
The topic monitor SHALL discover live decodable DimOS LCM topics by default and let users narrow, inspect, and select topics from the interactive UI rather than requiring CLI include/exclude configuration.

#### Scenario: Mixed topic traffic is present
- **GIVEN** live LCM traffic includes decodable typed DimOS topics, unsupported DimOS message types, and undecodable non-DimOS topics
- **WHEN** the user opens the topic monitor UI
- **THEN** the catalog displays decodable DimOS topics with live/idle, renderability, rate, and bandwidth information where available
- **AND** renderable typed topics can be staged for selected-only logging
- **AND** unsupported decoded topics remain visible but are not selectable for rendering
- **AND** undecodable topics are ignored.

### Requirement: Visualization dependency handling
The topic monitor SHALL require the visualization dependency extra and fail clearly when required visualization/runtime dependencies are unavailable.

#### Scenario: Required dependencies are missing
- **GIVEN** Reflex, Rerun, FastAPI, Uvicorn, or other required monitor runtime dependencies are not installed
- **WHEN** the user runs `dimos topic monitor`
- **THEN** the command fails with an actionable message
- **AND** the message tells the user to install the visualization extra.

### Requirement: Generic LCM rendering scope
The topic monitor SHALL render generic typed LCM topics with native Rerun conversion support and SHALL NOT require blueprint-specific visual configuration in v1.

#### Scenario: Topic has native Rerun conversion
- **GIVEN** a typed LCM topic uses a message type with native Rerun conversion support
- **WHEN** the topic is visible in the monitor catalog
- **THEN** the topic is marked renderable
- **AND** applying it logs subsequent messages to the monitor's Rerun viewer.

#### Scenario: Topic requires blueprint-specific visual override
- **GIVEN** a topic would require blueprint-specific visual override or static scene configuration to render
- **WHEN** the topic is visible in the monitor catalog without native conversion support
- **THEN** the monitor marks it unsupported according to available type information
- **AND** the monitor does not require the user to provide Python visual configuration in v1.
