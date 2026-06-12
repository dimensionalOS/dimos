# lcm-topic-catalog Specification

## Purpose
Specify the live LCM topic catalog used by topic monitoring, including decodable type discovery, renderability, liveness, rate, bandwidth, and unsupported-topic visibility.
## Requirements
### Requirement: Live LCM topic catalog
The system SHALL provide a runtime catalog of observed decodable DimOS LCM topics for topic monitor sessions.

#### Scenario: Observed typed topic appears in catalog
- **GIVEN** topic monitoring is running for a DimOS stack that publishes an LCM channel with a typed channel name
- **WHEN** messages are observed on that channel
- **THEN** the catalog shows the LCM channel and normalized topic name
- **AND** the catalog shows the decoded message type when it can be determined from the channel or configured decoding support
- **AND** the catalog marks the topic as live.

#### Scenario: Untyped or undecodable topic is ignored
- **GIVEN** topic monitoring is running and an LCM channel is observed without a known message type or decoder
- **WHEN** the catalog is displayed
- **THEN** the channel is not required to appear in the catalog
- **AND** the catalog remains focused on decodable DimOS topics.

### Requirement: Topic renderability status
The system SHALL classify observed LCM topics by whether they can be rendered in Rerun.

#### Scenario: Renderable topic is identified
- **GIVEN** an observed LCM topic has decoded messages that support Rerun rendering or have a configured visual converter
- **WHEN** the topic appears in the catalog
- **THEN** the catalog marks the topic as renderable
- **AND** the catalog exposes enough status for the UI to allow selection.

#### Scenario: Non-renderable topic is identified
- **GIVEN** an observed LCM topic has decoded messages but no Rerun rendering support
- **WHEN** the topic appears in the catalog
- **THEN** the catalog marks the topic as unsupported
- **AND** the UI can explain that no Rerun conversion is available.

### Requirement: Live topic diagnostics
The system SHALL expose live diagnostics for observed decodable LCM topics in topic monitor sessions.

#### Scenario: Diagnostics update while traffic flows
- **GIVEN** an observed LCM topic continues publishing messages
- **WHEN** the catalog refreshes
- **THEN** the catalog reports recent activity such as last seen time, message count, and approximate rate
- **AND** high-bandwidth or high-rate topics can be distinguished from low-rate topics.

#### Scenario: Topic becomes idle
- **GIVEN** an LCM topic was previously observed
- **WHEN** no new messages arrive for that topic within the UI's freshness window
- **THEN** the catalog keeps the topic visible
- **AND** the topic status changes from live to idle or stale.

### Requirement: LCM-only v1 scope disclosure
The system SHALL make the v1 catalog scope clear to users.

#### Scenario: Non-LCM streams are not cataloged
- **GIVEN** a running DimOS stack includes SHM, ROS, DDS, or stored replay streams that are not bridged to LCM
- **WHEN** the topic monitor catalog is shown
- **THEN** those non-LCM streams are not required to appear in the v1 catalog
- **AND** documentation or UI copy explains that v1 discovers live LCM topics only.
