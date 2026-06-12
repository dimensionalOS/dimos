# rerun-topic-selection Specification

## Purpose
Specify how DimOS Rerun topic selection catalogs live decodable LCM traffic, stages topic choices, and logs only applied topics through the attach-style topic monitor sidecar.
## Requirements
### Requirement: Current-session staged topic selection
The system SHALL let users stage renderable LCM topic choices in the Reflex selector UI and explicitly apply them before those topics are logged to Rerun during a topic monitor session.

#### Scenario: User stages and applies a renderable topic
- **GIVEN** topic monitoring is running and a renderable LCM topic is visible in the Reflex catalog UI
- **WHEN** the user stages the topic and applies the selection
- **THEN** subsequent messages for that topic are logged to Rerun
- **AND** the embedded Rerun viewer can display the selected topic data.

#### Scenario: Staged topic does not log before apply
- **GIVEN** topic monitoring is running and a renderable LCM topic is visible in the Reflex catalog UI
- **WHEN** the user stages the topic but does not apply the selection
- **THEN** subsequent messages for that topic are not logged merely because the staged state changed
- **AND** the UI indicates that the staged selection differs from the applied/logging selection.

#### Scenario: User clears and applies selected topics
- **GIVEN** a renderable LCM topic is selected and being logged to Rerun
- **WHEN** the user clears or removes the topic from the staged selection and applies that change
- **THEN** subsequent messages for that topic are not converted or logged by the selector-managed path
- **AND** other selected topics continue to be logged.

### Requirement: Selected-only logging before expensive conversion
The system SHALL avoid converting or logging unselected topics in managed-selection mode regardless of whether the selector frontend is implemented with Reflex.

#### Scenario: High-bandwidth topic remains unselected
- **GIVEN** topic monitoring is running and a high-bandwidth renderable LCM topic is flowing
- **WHEN** the topic is not in the applied selection
- **THEN** the topic is not converted to Rerun data by the selector-managed path
- **AND** the topic is not logged to Rerun merely because it is renderable.

#### Scenario: Unsupported topic cannot be selected for rendering
- **GIVEN** an observed decodable LCM topic is marked unsupported in the catalog
- **WHEN** the user reviews available topics in the Reflex selector UI
- **THEN** the UI does not present it as a renderable topic selection
- **AND** the UI gives a short reason such as no Rerun converter.

### Requirement: Compatibility with existing visualization behavior
The system SHALL preserve existing automatic Rerun visualization behavior for normal blueprints and SHALL make attach-style topic monitoring available through `dimos topic monitor` rather than requiring ordinary blueprints to embed selector-specific visualization wiring.

#### Scenario: Existing blueprint uses standard visualization
- **GIVEN** a DimOS blueprint uses the standard visualization path
- **WHEN** the blueprint runs
- **THEN** existing automatic Rerun logging behavior remains available
- **AND** topics are not silently hidden by topic monitor selection state.

#### Scenario: User wants selected-only topic visualization after startup
- **GIVEN** a DimOS run is already active and publishing LCM topics
- **WHEN** the user runs `dimos topic monitor`
- **THEN** selected-only topic logging applies to the monitor's independent sidecar viewer
- **AND** the original robot or simulation stack is not modified.

#### Scenario: Dedicated selector demo is used
- **GIVEN** a user wants a self-contained selector smoke test or demonstration
- **WHEN** the user runs the dedicated selector demo blueprint
- **THEN** the demo remains available as a hardware-free way to publish decodable sample LCM topics for topic monitor smoke testing
- **AND** ordinary production blueprints are not required to use selector-specific visualization wiring.

### Requirement: Embedded Rerun viewer feedback
The system SHALL present topic selection and Rerun viewer availability in one Reflex web workflow.

#### Scenario: Rerun viewer is available
- **GIVEN** topic monitoring is running and the Rerun web viewer endpoint is reachable
- **WHEN** the user opens the Reflex selector web app
- **THEN** the page shows the LCM topic catalog and an embedded Rerun viewer area
- **AND** applied renderable topics are reflected in the viewer as data is logged
- **AND** the page shows which topics are currently applied/logging.

#### Scenario: Rerun viewer is unavailable
- **GIVEN** topic monitoring is running but the Rerun web viewer endpoint is unavailable
- **WHEN** the user opens the Reflex selector web app
- **THEN** the page still shows the LCM topic catalog when topic data is available
- **AND** the page shows an actionable connection hint for the Rerun viewer instead of failing silently.

### Requirement: Reflex selector runtime availability
The system SHALL provide a Reflex selector runtime that can be started and stopped with topic monitoring without requiring real robot hardware.

#### Scenario: Demo selector starts on a PC
- **GIVEN** the hardware-free demo publisher is running and `dimos topic monitor` starts
- **WHEN** the Reflex selector runtime starts
- **THEN** the selector web app is reachable from the documented browser URL
- **AND** the embedded Rerun viewer uses a connected `url=rerun%2Bhttp...%2Fproxy` source URL rather than a bare viewer shell.

#### Scenario: Selector startup fails
- **GIVEN** topic monitoring is configured but the Reflex runtime cannot start
- **WHEN** the user opens or inspects the selector workflow
- **THEN** the failure is visible through logs or UI-accessible diagnostics
- **AND** robot command, skill, and MCP behavior remains unaffected.
